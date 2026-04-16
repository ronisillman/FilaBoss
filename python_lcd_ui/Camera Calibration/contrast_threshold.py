from picamera2 import Picamera2
from libcamera import controls
import cv2
import numpy as np
import json
import os

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "filament_config.json")

# -----------------------------
# Load calibration
# -----------------------------
def load_settings():
    with open(CONFIG_FILE) as f:
        data = json.load(f)

    lens_position = data["lens_position"]

    rois = []
    for r in data["rois"]:
        cx = r["cx"]
        cy = r["cy"]
        w = r["w"]
        h = r["h"]

        x0 = int(cx - w/2)
        x1 = int(cx + w/2)
        y0 = int(cy - h/2)
        y1 = int(cy + h/2)

        rois.append((x0, y0, x1, y1))

    threshold = data.get("threshold", 106)

    return lens_position, rois, threshold


def save_threshold(threshold):
    with open(CONFIG_FILE) as f:
        data = json.load(f)

    data["threshold"] = threshold

    with open(CONFIG_FILE, "w") as f:
        json.dump(data, f, indent=4)

    print(f"Threshold {threshold} saved to {CONFIG_FILE}")


lens_position, rois, threshold_value = load_settings()
roi_names = ["Top", "Middle", "Bottom"]

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
print("U = increase threshold")
print("J = decrease threshold")
print("P = save threshold")
print("Q = quit")
print(f"Starting threshold: {threshold_value}")

# -----------------------------
# Window positions
# -----------------------------
roi_win_x = 1090
cv2.namedWindow("Threshold tuner")
cv2.moveWindow("Threshold tuner", 0, 0)
for idx, name in enumerate(roi_names):
    win_name = f"Binary ROI - {name}"
    cv2.namedWindow(win_name)
    cv2.moveWindow(win_name, roi_win_x, idx * 200)

# -----------------------------
# Main loop
# -----------------------------
while True:

    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    h, w = gray.shape

    for idx, (rx0, ry0, rx1, ry1) in enumerate(rois):

        rx0 = max(0, rx0)
        ry0 = max(0, ry0)
        rx1 = min(w, rx1)
        ry1 = min(h, ry1)

        roi = gray[ry0:ry1, rx0:rx1]
        if roi.size == 0:
            continue

        # Adjustable threshold
        _, binary = cv2.threshold(
            roi,
            threshold_value,
            255,
            cv2.THRESH_BINARY_INV
        )

        binary = cv2.medianBlur(binary, 3)

        cv2.imshow(f"Binary ROI - {roi_names[idx]}", binary)

        # Draw ROI
        color = (255, 0, 0) if idx == 1 else (255, 255, 0)
        cv2.rectangle(frame, (rx0, ry0), (rx1, ry1), color, 2)

    # Show threshold value
    cv2.putText(
        frame,
        f"Threshold: {threshold_value}",
        (40, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2
    )

    cv2.imshow("Threshold tuner", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('u'):
        threshold_value = min(255, threshold_value + 2)

    elif key == ord('j'):
        threshold_value = max(0, threshold_value - 2)

    elif key == ord('p'):
        save_threshold(threshold_value)

    elif key == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()