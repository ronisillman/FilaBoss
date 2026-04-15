from picamera2 import Picamera2
from libcamera import controls
import cv2
import numpy as np
import json
from collections import deque
import os
import time

import socket

# Only show OpenCV windows when a display is available AND not running as a headless service.
HAS_DISPLAY = bool(os.environ.get("DISPLAY", "")) and not bool(os.environ.get("FILABOSS_HEADLESS", ""))

# Socket setup
SOCKET_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "sockets",
    "filament_socket.sock",
)
vision_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)


def connect_socket() -> bool:
    try:
        vision_socket.connect(SOCKET_PATH)
        print(f"Connected to UI socket at {SOCKET_PATH}")
        return True
    except OSError as e:
        print(f"Socket connect error: {e}")
        return False


socket_connected = connect_socket()



CONFIG_FILE = "filament_config.json"

REAL_FILAMENT_DIAMETER_MM = 2.63
THRESHOLD = 84
NUM_SCANLINES = 120

# -----------------------------
# Subpixel edge detection
# -----------------------------
def subpixel_edge_top(gray_col, threshold):
    for i in range(len(gray_col) - 1):
        p0 = gray_col[i]
        p1 = gray_col[i + 1]

        if p0 > threshold and p1 <= threshold:
            return i + (threshold - p0) / (p1 - p0)
    return None


def subpixel_edge_bottom(gray_col, threshold):
    for i in range(len(gray_col) - 1, 0, -1):
        p0 = gray_col[i]
        p1 = gray_col[i - 1]

        if p0 > threshold and p1 <= threshold:
            return i + (threshold - p0) / (p1 - p0)
    return None


# -----------------------------
# Load + Save settings
# -----------------------------
def load_settings():
    try:
        with open(CONFIG_FILE) as f:
            data = json.load(f)
    except FileNotFoundError:
        print("Config file not found, using defaults")
        return 0, [], [None, None, None], False

    lens_position = data.get("lens_position", 0)

    rois = []
    for r in data.get("rois", []):
        cx = r["cx"]
        cy = r["cy"]
        w = r["w"]
        h = r["h"]

        x0 = int(cx - w/2)
        x1 = int(cx + w/2)
        y0 = int(cy - h/2)
        y1 = int(cy + h/2)

        rois.append((x0, y0, x1, y1))

    mm_per_pixel = data.get("mm_per_pixel", [None, None, None])
    calibration_locked = data.get("calibration_locked", False)

    return lens_position, rois, mm_per_pixel, calibration_locked


def save_calibration(lens_position, rois, mm_per_pixel, calibration_locked):
    data = {}

    try:
        with open(CONFIG_FILE) as f:
            data = json.load(f)
    except FileNotFoundError:
        pass

    data["lens_position"] = lens_position

    roi_list = []
    for (x0, y0, x1, y1) in rois:
        cx = (x0 + x1) / 2
        cy = (y0 + y1) / 2
        w = (x1 - x0)
        h = (y1 - y0)

        roi_list.append({
            "cx": cx,
            "cy": cy,
            "w": w,
            "h": h
        })

    data["rois"] = roi_list
    data["mm_per_pixel"] = mm_per_pixel
    data["calibration_locked"] = calibration_locked

    with open(CONFIG_FILE, "w") as f:
        json.dump(data, f, indent=4)

    print("Calibration saved")


# -----------------------------
# Load config
# -----------------------------
lens_position, rois, mm_per_pixel, calibration_locked = load_settings()
roi_names = ["Top", "Middle", "Bottom"]

last_socket_send_time = time.time()


# -----------------------------
# Camera setup
# -----------------------------
picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"size": (1080, 720), "format": "RGB888"}
)

picam2.configure(config)
picam2.start()

picam2.set_controls({
    "AfMode": controls.AfModeEnum.Manual,
    "LensPosition": lens_position
})

print("Controls:")
print("C = calibrate scale")
print("R = reset calibration")
print("Q = quit")

# -----------------------------
# Rolling averages
# -----------------------------
history = [deque(maxlen=15), deque(maxlen=15), deque(maxlen=15)]

# -----------------------------
# Main loop
# -----------------------------
while True:

    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    h, w = gray.shape
    pixel_measurements = [None, None, None]

    for idx, (rx0, ry0, rx1, ry1) in enumerate(rois):

        rx0 = max(0, rx0)
        ry0 = max(0, ry0)
        rx1 = min(w, rx1)
        ry1 = min(h, ry1)

        roi = gray[ry0:ry1, rx0:rx1]

        if roi.size == 0:
            continue

        roi = cv2.GaussianBlur(roi, (3,3), 0)

        _, binary = cv2.threshold(
            roi,
            THRESHOLD,
            255,
            cv2.THRESH_BINARY_INV
        )

        binary = cv2.medianBlur(binary, 3)

        if HAS_DISPLAY:
                cv2.imshow(f"Binary ROI - {roi_names[idx]}", binary)

        roi_h, roi_w = roi.shape
        xs = np.linspace(0, roi_w - 1, NUM_SCANLINES).astype(int)

        diameters_px = []
        tops = []
        bottoms = []

        for x in xs:
            col = roi[:, x]

            top = subpixel_edge_top(col, THRESHOLD)
            bottom = subpixel_edge_bottom(col, THRESHOLD)

            if top is not None and bottom is not None and bottom > top:
                tops.append(top)
                bottoms.append(bottom)
                diameters_px.append(bottom - top)

        if diameters_px:
            diameter_px = np.median(diameters_px)
            pixel_measurements[idx] = diameter_px

            if calibration_locked and mm_per_pixel[idx] is not None:
                diameter_mm = diameter_px * mm_per_pixel[idx]

                history[idx].append(diameter_mm)
                smoothed_mm = np.mean(history[idx])

                x_mid = int(rx0 + np.mean(xs))
                y_top = int(ry0 + np.mean(tops))
                y_bottom = int(ry0 + np.mean(bottoms))

                cv2.line(frame, (x_mid, y_top), (x_mid, y_bottom), (0,255,0), 2)

                cv2.putText(
                    frame,
                    f"{roi_names[idx]}: {smoothed_mm:.3f} mm",
                    (40, 40 + idx * 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0,255,0),
                    2
                )

        color = (255,0,0) if idx == 1 else (255,255,0)
        cv2.rectangle(frame, (rx0, ry0), (rx1, ry1), color, 2)

    # Roundness
    if calibration_locked:
        # Collect per-roi smoothed diameters, keeping index (0=top,1=mid,2=bot)
        diameters_by_roi = [
            np.mean(history[i]) if len(history[i]) > 0 else None
            for i in range(3)
        ]
        available = [d for d in diameters_by_roi if d is not None]

        if len(available) >= 1:
            d_mean = np.mean(available)
            # Roundness only meaningful with 2+ measurements
            if len(available) >= 2:
                d_max = np.max(available)
                d_min = np.min(available)
                roundness_index = (d_max - d_min) / d_mean
            else:
                roundness_index = 0.0

            current_time = time.time()

            if current_time - last_socket_send_time >= 0.5:
                # Fill missing ROIs with the mean of available ones
                top_mm    = diameters_by_roi[0] if diameters_by_roi[0] is not None else d_mean
                middle_mm = diameters_by_roi[1] if diameters_by_roi[1] is not None else d_mean
                bottom_mm = diameters_by_roi[2] if diameters_by_roi[2] is not None else d_mean
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                payload = {
                    "timestamp": timestamp,
                    "top_mm": round(top_mm, 3),
                    "middle_mm": round(middle_mm, 3),
                    "bottom_mm": round(bottom_mm, 3),
                    "roundness": round(roundness_index, 3)
                }

                data_line = json.dumps(payload, separators=(",", ":")) + "\n"
                try:
                    if not socket_connected:
                        socket_connected = connect_socket()

                    if socket_connected:
                        vision_socket.sendall(data_line.encode("utf-8"))
                        print("Socket: data sent")
                        last_socket_send_time = current_time
                except OSError as e:
                    print(f"Socket send error: {e}")
                    socket_connected = False
                    try:
                        vision_socket.close()
                    except OSError:
                        pass
                    vision_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

            if HAS_DISPLAY and len(available) >= 2:
                cv2.putText(
                    frame,
                    f"Roundness: {roundness_index:.4f}",
                    (40, 160),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0,200,255),
                    2
                )

    status = "LOCKED" if calibration_locked else "NOT CALIBRATED"

    cv2.putText(
        frame,
        f"Calibration: {status}",
        (40,130),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0,255,255),
        2
    )

    if HAS_DISPLAY:
        cv2.imshow("Filament measurement", frame)

    key = (cv2.waitKey(1) & 0xFF) if HAS_DISPLAY else 0xFF

    # CALIBRATE
    if key == ord('c'):
        if all(p is not None for p in pixel_measurements):

            for i in range(3):
                mm_per_pixel[i] = REAL_FILAMENT_DIAMETER_MM / pixel_measurements[i]

            calibration_locked = True
            save_calibration(lens_position, rois, mm_per_pixel, calibration_locked)

            print("Calibration locked and saved")

    # RESET
    elif key == ord('r'):
        mm_per_pixel = [None, None, None]
        calibration_locked = False
        save_calibration(lens_position, rois, mm_per_pixel, calibration_locked)

        print("Calibration reset")

    elif key == ord('q'):
        break


picam2.stop()
if HAS_DISPLAY:
    cv2.destroyAllWindows()
vision_socket.close()
