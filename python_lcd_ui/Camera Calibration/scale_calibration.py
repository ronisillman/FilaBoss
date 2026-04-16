from picamera2 import Picamera2
from libcamera import controls
import cv2
import numpy as np
import json
import os

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "filament_config.json")

NUM_SCANLINES = 120


# -----------------------------
# Edge detection helpers
# -----------------------------
def subpixel_edge_top(gray_col, threshold):
    for i in range(len(gray_col) - 1):
        if gray_col[i] > threshold and gray_col[i + 1] <= threshold:
            return i + (threshold - gray_col[i]) / (gray_col[i + 1] - gray_col[i])
    return None


def subpixel_edge_bottom(gray_col, threshold):
    for i in range(len(gray_col) - 1, 0, -1):
        if gray_col[i] > threshold and gray_col[i - 1] <= threshold:
            return i + (threshold - gray_col[i]) / (gray_col[i - 1] - gray_col[i])
    return None


# -----------------------------
# Load config (ROIs + lens)
# -----------------------------
def load_settings():
    with open(CONFIG_FILE) as f:
        data = json.load(f)

    lens_position = data["lens_position"]

    rois = []
    for r in data["rois"]:
        cx, cy, w, h = r["cx"], r["cy"], r["w"], r["h"]
        rois.append((int(cx - w/2), int(cy - h/2), int(cx + w/2), int(cy + h/2)))

    threshold = data.get("threshold", 84)

    return lens_position, rois, data, threshold


def save_calibration(data, mm_per_pixel):
    data["mm_per_pixel"] = mm_per_pixel
    data["calibration_locked"] = True

    with open(CONFIG_FILE, "w") as f:
        json.dump(data, f, indent=4)

    print("Calibration saved to", CONFIG_FILE)


# -----------------------------
# Camera setup
# -----------------------------
lens_position, rois, config_data, THRESHOLD = load_settings()
roi_names = ["Top", "Middle", "Bottom"]

while True:
    try:
        REAL_FILAMENT_DIAMETER_MM = float(input("Enter reference filament diameter in mm: "))
        if REAL_FILAMENT_DIAMETER_MM > 0:
            break
        print("Diameter must be greater than 0")
    except ValueError:
        print("Invalid input, please enter a number")

picam2 = Picamera2()
cam_config = picam2.create_video_configuration(
    main={"size": (1080, 720), "format": "RGB888"}
)
picam2.configure(cam_config)
picam2.start()

picam2.set_controls({
    "AfMode": controls.AfModeEnum.Manual,
    "LensPosition": lens_position
})

print("Place a known-diameter filament in the ROIs")
print(f"Reference diameter: {REAL_FILAMENT_DIAMETER_MM} mm")
print("C = capture and save calibration")
print("Q = quit without saving")

# -----------------------------
# Window positions
# -----------------------------
cv2.namedWindow("Scale calibration")
cv2.imshow("Scale calibration", np.zeros((720, 1080, 3), dtype=np.uint8))
cv2.moveWindow("Scale calibration", 0, 0)

# -----------------------------
# Main loop
# -----------------------------
while True:

    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    fh, fw = gray.shape

    pixel_measurements = [None, None, None]

    for idx, (rx0, ry0, rx1, ry1) in enumerate(rois):

        rx0 = max(0, rx0)
        ry0 = max(0, ry0)
        rx1 = min(fw, rx1)
        ry1 = min(fh, ry1)

        roi = gray[ry0:ry1, rx0:rx1]
        if roi.size == 0:
            continue

        roi = cv2.GaussianBlur(roi, (3, 3), 0)
        _, binary = cv2.threshold(roi, THRESHOLD, 255, cv2.THRESH_BINARY_INV)
        binary = cv2.medianBlur(binary, 3)

        roi_h, roi_w = roi.shape
        xs = np.linspace(0, roi_w - 1, NUM_SCANLINES).astype(int)

        diameters_px = []
        for x in xs:
            col = roi[:, x]
            top = subpixel_edge_top(col, THRESHOLD)
            bottom = subpixel_edge_bottom(col, THRESHOLD)
            if top is not None and bottom is not None and bottom > top:
                diameters_px.append(bottom - top)

        if diameters_px:
            pixel_measurements[idx] = np.median(diameters_px)
            cv2.putText(
                frame,
                f"{roi_names[idx]}: {pixel_measurements[idx]:.1f} px",
                (40, 40 + idx * 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

        color = (0, 255, 0) if pixel_measurements[idx] is not None else (0, 0, 255)
        cv2.rectangle(frame, (rx0, ry0), (rx1, ry1), color, 2)

    ready = all(p is not None for p in pixel_measurements)
    status = "Ready - press C to calibrate" if ready else "Waiting for all 3 ROIs..."
    cv2.putText(frame, status, (40, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.imshow("Scale calibration", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):
        if ready:
            mm_per_pixel = [
                REAL_FILAMENT_DIAMETER_MM / pixel_measurements[i]
                for i in range(3)
            ]
            for i, name in enumerate(roi_names):
                print(f"  {name}: {pixel_measurements[i]:.2f} px → {mm_per_pixel[i]:.6f} mm/px")
            save_calibration(config_data, mm_per_pixel)
            break
        else:
            print("Not all ROIs detected - make sure filament is visible in all 3 boxes")

    elif key == ord('q'):
        print("Quit without saving")
        break

picam2.stop()
cv2.destroyAllWindows()
