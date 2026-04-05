# FilaBoss LCD UI Starter (2004A + PCF8574T)

This starter project gives you one UI codebase with two backends:

- Simulator backend on laptop (arrow keys + space)
- Hardware backend on Raspberry Pi 5 (I2C LCD + rotary encoder button)

## Hardware target

- LCD: 2004A (20x4 character display)
- I2C backpack: PCF8574T
- Raspberry Pi I2C bus: `/dev/i2c-1`

## Controls

### Simulator

- Up/Down arrow: move between menus or change the selected value
- Space/Enter: select the current menu field
- Escape: quit

### Hardware (rotary encoder + push button)

- Rotate knob: move between menus or change the selected value
- Push knob: select the current menu field

Default BCM pins in this starter:

- CLK: GPIO17
- DT: GPIO27
- SW: GPIO22

Change with command-line arguments if your wiring is different.

## Quick start on laptop (simulation)

```bash
python -m venv .venv
# Windows
.venv\Scripts\activate
# Linux/macOS
# source .venv/bin/activate

python main.py --mode sim
```

## Menu layout

- MAIN: filament speed, filament diameter, fan speed
- PID: tune P, I, and D for Pulley or Spool
- FAN: change fan speed

The top bar highlights the active menu. On PID, press select to enter field mode, use the rotary or arrows to change the active field, and press select again to move to the next field.

## Quick start on Raspberry Pi (real LCD)

1. Enable I2C:

```bash
sudo raspi-config
# Interface Options -> I2C -> Enable
```

2. Install dependencies:

```bash
sudo apt update
sudo apt install -y python3-pip python3-smbus i2c-tools
pip3 install -r requirements.txt
```

3. Find LCD I2C address:

```bash
i2cdetect -y 1
```

Typical addresses are `0x27` or `0x3F`.

4. Run hardware mode:

```bash
python3 main.py --mode hw --i2c-address 0x27
```

## Pages included

- MAIN dashboard
- PID tuning page
- FAN control page

These are sample pages to help you plug in your real control values later.

## Project files

- `main.py`: app loop and backend selection
- `ui_controller.py`: UI state, pages, navigation logic
- `display_simulator.py`: desktop LCD simulator
- `display_hardware.py`: RPLCD hardware driver
- `input_devices.py`: rotary button input reader
