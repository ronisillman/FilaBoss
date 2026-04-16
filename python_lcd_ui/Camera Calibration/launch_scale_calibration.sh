#!/bin/bash
echo "Stopping machine vision service..."
sudo systemctl stop filaboss-machine-vision

echo "Starting scale calibration..."
cd "$(dirname "$0")"
DISPLAY=:0 python3 scale_calibration.py

echo "Restarting machine vision service..."
sudo systemctl start filaboss-machine-vision
echo "Done."
