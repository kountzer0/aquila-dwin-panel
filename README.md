# Voxelab Aquila DWIN panel for Klipper

Landscape control panel software for the Voxelab Aquila's DWIN T5UIC1 display. It talks to Klipper through Moonraker and uses the Raspberry Pi UART plus GPIO encoder/button inputs. The UI includes print controls, temperature and movement pages, manual leveling, probe capability detection, and a bed-mesh flow guarded by Klipper's configured capabilities.

## Hardware defaults

The code defaults to `/dev/ttyAMA0`, Moonraker at `http://127.0.0.1:7125`, encoder GPIO 19/26, push-button GPIO 13, and buzzer GPIO 16 (BCM numbering). Override the UART and Moonraker address with `DWIN_UART_PORT` and `MOONRAKER_URL`. Review the GPIO assignments for your wiring before powering the panel.

## Install

1. Copy `DWIN_Screen.py` and `aquila_landscape_control.py` to `/home/<user>/aquila-dwin/`.
2. Install Python dependencies `requests`, `pyserial`, and the Raspberry Pi GPIO package.
3. Enable the Pi UART for the display. Add the user to the `dialout` and `gpio` groups.
4. Install `aquila-dwin@.service` into `/etc/systemd/system/`. Its working directory is the user's home directory so `lgpio` can create its notification pipe.
5. Start it with `sudo systemctl enable --now aquila-dwin@<user>.service`.

Use the stock DWIN T5UIC1 display firmware/assets. The software does not configure the printer's mechanics: use a suitable `printer.cfg`, verify endstops and probe wiring, and calibrate Z before printing. The bed-mesh controls remain unavailable unless Klipper exposes a configured probe and `[bed_mesh]`.

This repository contains no personal Aquila `printer.cfg`, calibration values, Wi-Fi credentials, or printer-specific host address.

## License

GPL-3.0-or-later. The serial driver is based on the project's GPL-3.0 DWIN T5UIC1 driver.
