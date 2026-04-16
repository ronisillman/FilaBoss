#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Stopping machine vision service..."
sudo systemctl stop filaboss-machine-vision

echo ""
echo "=== Step 1/3: Camera Focus & ROI Positioning ==="
echo "Adjust ROI boxes and focus, press P to save, Q to continue."
DISPLAY=:0 python3 "$SCRIPT_DIR/camerafocus.py"

echo ""
echo "=== Step 2/3: Contrast Threshold ==="
echo "Adjust threshold until filament edges are clear, press P to save, Q to continue."
DISPLAY=:0 python3 "$SCRIPT_DIR/contrast_threshold.py"

echo ""
echo "=== Step 3/3: Scale Calibration ==="
echo "Place reference filament in view, press C to calibrate, Q to quit."
DISPLAY=:0 python3 "$SCRIPT_DIR/scale_calibration.py"

echo ""
echo "Calibration complete. Restarting machine vision service..."
sudo systemctl start filaboss-machine-vision
echo "Done."
