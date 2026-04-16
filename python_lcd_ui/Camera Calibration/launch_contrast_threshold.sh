#!/bin/bash
echo "Stopping machine vision service..."
sudo systemctl stop filaboss-machine-vision

echo "Starting contrast threshold calibration..."
cd "$(dirname "$0")"
DISPLAY=:0 python3 contrast_threshold.py

echo "Restarting machine vision service..."
sudo systemctl start filaboss-machine-vision
echo "Done."
