# Rot32

## Hardware Requirements

- ESP32 board running MicroPython
- LSM303 accelerometer/magnetometer on I2C
- Generic H-bridge motor driver, sharing a ground with the ESP32
- A two-axis (azimuth/elevation) rotator

## Wiring

Defaults from `config.py` for ESP32-S3

| Signal | Pin |
|---|---|
| I2C SDA | GPIO8 |
| I2C SCL | GPIO9 |
| AZ+ / AZ- (PWM) | GPIO4 / GPIO5 |
| EL+ / EL- (PWM) | GPIO6 / GPIO7 |

## Install

1. Flash MicroPython from
   [micropython.org/download](https://micropython.org/download/) for your
   module.
2. Edit `config.py`: set `WIFI_SSID`, `WIFI_PASSWORD`, and the pin numbers.
3. Copy the project files to the device, e.g. `mpremote cp *.py *.html :`.
4. Reset the board. The IP address is recorded in the log, readable at
   `http://<device-ip>/api/log` or over serial with `SERIAL_LOGGING_ENABLED`.

## Web UI

Browse to `http://<device-ip>/` for a simple control interface:

- **Position**: live azimuth/elevation readout, plus the current target
  while a move is in progress.
- **Manual control**: press-and-hold direction buttons plus a speed slider.
- **Go To Position**: numeric azimuth/elevation entry, plus Park and Reset.
- **Motion Tuning**: arrival window, arrival speed, slow-down distance, and
  acceleration.
- **Calibration**: declination, offsets, invert flags, axis selection,
  smoothing, and a magnetometer calibration wizard.

## First-time setup

1. Pick the **azimuth forward axis**: rotate the antenna while level and try
   each of X/Y/Z until the Azimuth readout tracks correctly. Then tilt to a
   nonzero elevation and rotate again to confirm it is still accurate.
2. Pick the **elevation axis pair**: choose whichever of XY/XZ/YZ changes as
   you tilt the sensor.
3. Run the **magnetometer calibration wizard**: click Start, sweep the sensor
   through a full 360 degrees for 30 to 60 seconds, then click Finish.
4. Set **magnetic declination** for your location, then trim the azimuth and
   elevation offsets.
5. Raise **Smoothing** if the readout is jittery. Keep it at 0% while tuning
   axes and offsets so the display reacts immediately.

If azimuth is accurate at one heading but drifts at others, that is magnetic
distortion near the sensor. Use the wizard, not the offset field, which
shifts every heading by the same amount.

## rotctld

TCP server on port `4533`, accepting short and long command forms.

| Command | Long form |
|---|---|
| `P Az El` | `\set_pos` |
| `p` | `\get_pos` |
| `M Dir Speed` | `\move` |
| `S` | `\stop` |
| `K` | `\park` |
| `R 1` | `\reset` |
| `_` | `\get_info` |
| `1` | `\dump_caps` |
| `dump_state` | |

Direction values for `M`: `2` up, `4` down, `8` left, `16` right. Locator
commands are recognised but return unsupported.

```sh
printf 'p\n'         | nc 192.168.1.50 4533
printf 'P 180 30\n'  | nc 192.168.1.50 4533
printf 'M 16 60\n'   | nc 192.168.1.50 4533
printf 'S\n'         | nc 192.168.1.50 4533
```

The extended response protocol is supported by prefixing a command with `+`:

```sh
printf '+\\get_pos\n' | nc 192.168.1.50 4533
```
