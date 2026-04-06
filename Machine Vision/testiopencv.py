from picamera2 import Picamera2
import cv2
import numpy as np

# Initialize camera
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (1080, 720), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

# Parameters
MM_PER_PIXEL = 0.005
NUM_SCANLINES = 30
ROI_GAP_PX = 15   # <<< space between ROIs in pixels

# Base ROI (fractions of frame size)
SCAN_REGION_X = (0.4, 0.6)
SCAN_REGION_Y = (0.3, 0.7)

while True:
    frame = picam2.capture_array()
    h, w, _ = frame.shape

    # Base ROI (middle)
    x0 = int(w * SCAN_REGION_X[0])
    x1 = int(w * SCAN_REGION_X[1])
    y0 = int(h * SCAN_REGION_Y[0])
    y1 = int(h * SCAN_REGION_Y[1])

    # --- Force square ROI ---
    roi_size = x1 - x0
    y_center = (y0 + y1) // 2
    y0 = y_center - roi_size // 2
    y1 = y0 + roi_size

    # Define ROIs with spacing
    rois = [
        (x0, y0 - roi_size - ROI_GAP_PX, x1, y0 - ROI_GAP_PX),  # top
        (x0, y0, x1, y1),                                      # middle
        (x0, y1 + ROI_GAP_PX, x1, y1 + ROI_GAP_PX + roi_size)  # bottom
    ]

    roi_names = ["Top", "Middle", "Bottom"]

    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    for idx, (rx0, ry0, rx1, ry1) in enumerate(rois):
        # Clamp to frame bounds
        rx0 = max(0, rx0)
        ry0 = max(0, ry0)
        rx1 = min(w, rx1)
        ry1 = min(h, ry1)

        roi = gray[ry0:ry1, rx0:rx1]
        if roi.size == 0:
            continue

        # Threshold
        _, binary = cv2.threshold(roi, 140, 255, cv2.THRESH_BINARY_INV)
        binary = cv2.medianBlur(binary, 3)

        # Show binary window for each ROI
        cv2.imshow(f"Binary ROI - {roi_names[idx]}", binary)

        roi_h, roi_w = binary.shape
        xs = np.linspace(0, roi_w - 1, NUM_SCANLINES).astype(int)

        diameters_px = []
        tops = []
        bottoms = []

        for x in xs:
            col = binary[:, x]
            ys = np.where(col > 0)[0]
            if len(ys) > 5:
                tops.append(ys[0])
                bottoms.append(ys[-1])
                diameters_px.append(ys[-1] - ys[0])

        if diameters_px:
            diameter_px = np.mean(diameters_px)
            diameter_mm = diameter_px * MM_PER_PIXEL

            x_mid = int(rx0 + np.mean(xs))
            y_top = int(ry0 + np.mean(tops))
            y_bottom = int(ry0 + np.mean(bottoms))

            cv2.line(frame, (x_mid, y_top), (x_mid, y_bottom), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"{roi_names[idx]}: {diameter_mm:.3f} mm",
                (40, 40 + idx * 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            print(f"{roi_names[idx]} ROI diameter: {diameter_mm:.3f} mm")

        # Draw ROIs
        color = (255, 0, 0) if idx == 1 else (255, 255, 0)
        cv2.rectangle(frame, (rx0, ry0), (rx1, ry1), color, 2)

    cv2.imshow("Filament measurement", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()