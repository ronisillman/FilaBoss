# FilaBoss LCD UI Starter (2004A + PCF8574T)

This starter project gives you one UI codebase with two backends:

- Simulator backend on laptop (arrow keys + space)
- Hardware backend on Raspberry Pi 5 (I2C LCD + rotary encoder button)

The hardware runtime can now consume two independent JSON inputs:

- ESP32 serial telemetry/commands (newline-delimited JSON over serial)
- Vision diameter feed (newline-delimited JSON over a local Unix domain socket)

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

2. Create a virtual environment and install dependencies:

```bash
sudo apt update
sudo apt install -y python3-pip python3-smbus python3-venv i2c-tools
cd ~/FilaBoss/python_lcd_ui
python3 -m venv .venv
source .venv/bin/activate
pip3 install -r requirements.txt
```

3. Find LCD I2C address:

```bash
i2cdetect -y 1
```

Typical addresses are `0x27` or `0x3F`.

4. Activate the virtual environment and run hardware mode:

```bash
source .venv/bin/activate
python3 main.py --mode hw --i2c-address 0x27
```

To enable ESP32 JSON serial communication in hardware mode:

```bash
python3 main.py --mode hw --i2c-address 0x27 --serial-port /dev/ttyUSB0 --serial-baudrate 115200
```

Serial communication is newline-delimited JSON in both directions.

To enable local vision diameter input over Unix socket (same Raspberry Pi):

```bash
python3 main.py --mode hw --i2c-address 0x27 --unix-socket-path /tmp/filament_socket
```

Default runtime values in the current code:

- `--serial-port` defaults to `/dev/ttyUSB0`
- `--unix-socket-path` defaults to `/tmp/filament_socket`

If you want to disable one channel explicitly:

```bash
# Disable serial bridge
python3 main.py --mode hw --serial-port ""

# Disable Unix socket bridge
python3 main.py --mode hw --unix-socket-path ""
```

Vision socket payload format (one JSON object per line):

```json
{"timestamp":1712563200.123,"top_mm":1.750,"middle_mm":1.748,"bottom_mm":1.752,"roundness":0.0023}
```

Required keys for diameter update are:

- `top_mm`
- `middle_mm`
- `bottom_mm`

The UI computes `Dia` as the mean of these three values and shows it with three decimals.

In hardware mode, the program prints a small terminal heartbeat once per second so you can confirm the loop is running.

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

Key data models in `main.py`:

- `TelemetryFromEsp32`: serial telemetry from ESP32
- `DiameterFromVision`: Unix socket payload from local vision process
