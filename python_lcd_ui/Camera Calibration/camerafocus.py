from picamera2 import Picamera2
from libcamera import controls
import cv2
import json
import os

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "filament_config.json")

# -----------------------------
# Load saved settings if exist
# -----------------------------
def load_settings():

    try:
        with open(CONFIG_FILE) as f:
            data = json.load(f)

        lens_position = data["lens_position"]

        rois = []
        for r in data["rois"]:
            rois.append([r["cx"], r["cy"], r["w"], r["h"]])

        print("Loaded saved settings")

    except:
        lens_position = 9.0
        rois = [
            [540, 240, 200, 80],
            [540, 360, 200, 80],
            [540, 480, 200, 80]
        ]
        print("Using default settings")

    return lens_position, rois


# -----------------------------
# Save settings
# -----------------------------
def save_settings(rois, lens_position):

    data = {
        "lens_position": lens_position,
        "rois": [
            {"cx": r[0], "cy": r[1], "w": r[2], "h": r[3]}
            for r in rois
        ]
    }

    with open(CONFIG_FILE, "w") as f:
        json.dump(data, f, indent=4)

    print("Settings saved to", CONFIG_FILE)


# -----------------------------
# Camera setup
# -----------------------------
picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"size": (1080, 720), "format": "RGB888"}
)

picam2.configure(config)
picam2.start()

lens_position, rois = load_settings()

picam2.set_controls({
    "AfMode": controls.AfModeEnum.Manual,
    "LensPosition": lens_position
})

names = ["Top", "Middle", "Bottom"]
selected = 1

print("Controls:")
print("1/2/3 = select ROI")
print("Arrows = move ROI")
print("A/D = width -/+")
print("W/S = height +/-")
print("I/K = focus +/-")
print("P = save settings")
print("Q = quit")

# -----------------------------
# Main loop
# -----------------------------
while True:

    frame = picam2.capture_array()
    h, w, _ = frame.shape

    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    for i,(cx,cy,rw,rh) in enumerate(rois):

        x0 = int(cx - rw/2)
        x1 = int(cx + rw/2)
        y0 = int(cy - rh/2)
        y1 = int(cy + rh/2)

        x0=max(0,x0)
        y0=max(0,y0)
        x1=min(w,x1)
        y1=min(h,y1)

        roi = gray[y0:y1,x0:x1]

        if roi.size > 0:
            zoom = cv2.resize(roi,None,fx=3,fy=3,
                              interpolation=cv2.INTER_NEAREST)
            cv2.imshow(names[i],zoom)

        color = (0,255,0) if i==selected else (255,255,0)
        cv2.rectangle(frame,(x0,y0),(x1,y1),color,2)

    cv2.putText(
        frame,
        f"Selected:{names[selected]}  Focus:{lens_position:.2f}",
        (30,40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0,255,0),
        2
    )

    cv2.imshow("ROI tuner",frame)

    key=cv2.waitKey(1) & 0xFF

    step = 5
    size_step = 10

    if key==ord('1'): selected=0
    elif key==ord('2'): selected=1
    elif key==ord('3'): selected=2

    elif key==81: rois[selected][0]-=step
    elif key==83: rois[selected][0]+=step
    elif key==82: rois[selected][1]-=step
    elif key==84: rois[selected][1]+=step

    elif key==ord('a'): rois[selected][2]-=size_step
    elif key==ord('d'): rois[selected][2]+=size_step
    elif key==ord('w'): rois[selected][3]+=size_step
    elif key==ord('s'): rois[selected][3]-=size_step

    elif key==ord('i'):
        lens_position+=0.2
        picam2.set_controls({"LensPosition":lens_position})

    elif key==ord('k'):
        lens_position=max(0,lens_position-0.2)
        picam2.set_controls({"LensPosition":lens_position})

    elif key==ord('p'):
        save_settings(rois,lens_position)

    elif key==ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()