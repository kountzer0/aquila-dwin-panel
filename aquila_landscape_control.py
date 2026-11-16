#!/usr/bin/env python3
"""Aquila landscape DWIN control panel v2.7.14.

Keeps the stock Voxelab T5UIC1 firmware/assets.  Uses the Pi UART directly
and talks to the existing Klipper/Moonraker installation over HTTP.

Implemented:
  * 480x272 landscape UI
  * live hotend/bed/position/print status (change-driven redraws)
  * dedicated print-status page with pause/resume/cancel
  * encoder navigation and push button
  * file browser with print confirmation
  * info screen
  * prepare controls with confirmation: PLA/PETG preheat, filament load/unload, cooldown, home all, motors off
  * filament operations use the existing Klipper macros and state-aware UI takeover
  * filament actions are protected from running during an active print or conflicting machine action
  * bookmarked for later: editable per-filament preset temperatures
  * safe relative-axis jog page with 10 / 1 / 0.1 mm steps
  * print status screen with live progress, pause/resume/cancel
  * automatic entry into print status for externally started jobs
  * manual four-corner bed-tramming wizard
  * manual centre Z-level / paper-test helper
  * probe-aware leveling with safe no-probe movement mode
  * probe/bed-mesh capability detection
  * guarded automatic bed-mesh routine (only enabled when probe + bed_mesh are configured)
  * transient on-screen action feedback with success/warning/error states

Printer-changing actions are gated behind an explicit confirmation screen.
"""
from __future__ import annotations

import json
import time
import threading
import queue
import subprocess
import os
import socket
from pathlib import Path
from collections import deque
from typing import Any

import requests
import RPi.GPIO as GPIO

from DWIN_Screen import T5UIC1_LCD

encoder_lock = threading.Lock()
encoder_state = (1, 1)
encoder_detents = 0
encoder_invalid = 0

# Shared leveling motion queue. Queueing helpers are module-level.
leveling_action_queue: queue.Queue = queue.Queue()

# DWIN stock buzzer wired to Raspberry Pi BCM GPIO16 (physical pin 36).
BUZZER_PIN = 16
COOLDOWN_HOTEND_MAX = 50.0
COOLDOWN_BED_MAX = 40.0

wifi_display = {
    "connected": False,
    "ssid": "",
    "signal": 0,
    "ip": "",
}
wifi_display_lock = threading.Lock()

process_display_state = "idle"
process_display_name = ""

PORT = os.environ.get("DWIN_UART_PORT", "/dev/ttyAMA0")
BASE = os.environ.get("MOONRAKER_URL", "http://127.0.0.1:7125")
PIN_A = 19
PIN_B = 26
PIN_ENT = 13

# Optional remote encoder bridge for development/testing.
# The companion aquila_virtual_encoder helper sends LEFT, RIGHT and CLICK
# over a local UNIX datagram socket. Physical GPIO remains fully active.
VIRTUAL_INPUT_SOCKET = "/tmp/aquila_encoder.sock"
virtual_input_lock = threading.Lock()
virtual_input_events: deque[str] = deque()

# RGB565 colours
BLACK = 0x0000
WHITE = 0xFFFF
BLUE = 0x001F
CYAN = 0x07FF
GREEN = 0x07E0
YELLOW = 0xFFE0
RED = 0xF800
GREY = 0x7BEF
DARK = 0x2104
MID = 0x4208

# Cohesive UI styling palette for the 480x272 Aquila DWIN panel.
ACCENT = CYAN
PANEL = 0x18C3
PANEL2 = 0x2945
TEXT_DIM = 0xBDF7
HOT = 0xFD20
OK = GREEN
WARN = YELLOW
DANGER = RED

MAIN_ITEMS = ["PRINT", "PREPARE", "MOVE", "TEMPERATURE", "CONTROL", "STATUS", "LEVELING", "SETTINGS", "INFO", "PRINT DOCTOR"]
LEVEL_ITEMS = ["BACK", "MANUAL BED LEVEL", "Z ENDSTOP (CONSOLE)", "PROBE SETUP", "BED MESH"]

# Conservative stock-Aquila travel points. These are deliberately inside the
# 235 x 235 bed rather than assuming the exact bed-screw coordinates.
BED_LEVEL_POINTS = [
    ("FRONT LEFT", 25.0, 25.0),
    ("FRONT RIGHT", 210.0, 25.0),
    ("REAR RIGHT", 210.0, 210.0),
    ("REAR LEFT", 25.0, 210.0),
]

# Aquila uses four manual bed screws. Keep the physical order explicit so the
# wizard, UI labels and motion worker cannot silently disagree about corners.
LEVEL_POINT_COUNT = len(BED_LEVEL_POINTS)
BED_LEVEL_ACTIONS = ["Z -", "Z +", "STEP", "NEXT", "DONE"]
Z_LEVEL_ACTIONS = ["Z -", "Z +", "STEP", "SAVE", "BACK"]
LEVEL_TRAVEL_Z = 5.0
LEVEL_MIN_Z = 0.20
LEVEL_MAX_Z = 20.0
LEVEL_STEPS = [1.0, 0.1, 0.01]
PREPARE_ITEMS = ["BACK", "PLA PREHEAT", "PETG PREHEAT", "LOAD FILAMENT", "UNLOAD FILAMENT", "COOLDOWN", "HOME ALL", "MOTORS OFF"]
TEMP_ITEMS = ["BACK", "HOTEND", "BED", "FILAMENT"]
# Native temperature editing. One encoder detent = one degree.
# This deliberately gives fine control rather than reusing the coarser
# preparation presets. Zero is allowed and means heater OFF.
TEMP_STEP = 1.0
HOTEND_MAX = 250.0
BED_MAX = 100.0
# v27.1 persistent filament presets.  Defaults are deliberately conservative
# and remain inside the temperature safety limits enforced by the UI.
FILAMENT_PRESETS_DEFAULT = {
    "PLA": {"hotend": 200.0, "bed": 55.0},
    "PETG": {"hotend": 230.0, "bed": 75.0},
    "ASA": {"hotend": 250.0, "bed": 100.0},
}
FILAMENT_PRESET_NAMES = list(FILAMENT_PRESETS_DEFAULT)
CONTROL_ITEMS = ["BACK", "FAN", "PRINT SPEED"]
FAN_STEP = 5.0
SPEED_STEP = 5.0
SPEED_MIN = 50.0
SPEED_MAX = 150.0

SETTINGS_ITEMS = ["BACK", "BRIGHTNESS", "SLEEP TIMEOUT", "LOGGING", "WI-FI"]
WIFI_ACTIONS = ["BACK", "REFRESH"]

MOVE_AXES = ["BACK", "HOME ALL", "MOTORS OFF", "X", "Y", "Z"]
MOVE_STEPS = [10.0, 1.0, 0.1]
MOVE_FEED = {"X": 3000, "Y": 3000, "Z": 300}

# Display power-saving settings.  These control the DWIN backlight only.
DISPLAY_TIMEOUT_IDLE = 60.0       # default idle timeout
DISPLAY_TIMEOUT_PRINTING = 300.0  # default printing timeout
# Transient operator feedback. This is deliberately UI-only: it never blocks
# printer operations and automatically expires after a short period.
feedback_lock = threading.Lock()
feedback_message = ""
feedback_kind = "info"
feedback_until = 0.0

def set_feedback(message: str, kind: str = "info", duration: float = 3.5) -> None:
    global feedback_message, feedback_kind, feedback_until
    with feedback_lock:
        feedback_message = str(message)[:58]
        feedback_kind = kind if kind in ("info", "ok", "warn", "error") else "info"
        feedback_until = time.monotonic() + max(0.5, float(duration))

def get_feedback() -> tuple[str, str]:
    global feedback_message
    with feedback_lock:
        if feedback_message and time.monotonic() < feedback_until:
            return feedback_message, feedback_kind
        feedback_message = ""
        return "", "info"

def feedback_colour(kind: str) -> int:
    return {"ok": OK, "warn": WARN, "error": DANGER}.get(kind, ACCENT)

SETTINGS_FILE = os.environ.get("AQUILA_DWIN_SETTINGS_FILE", str(Path.home() / ".aquila_dwin_settings.json"))
BRIGHTNESS_DEFAULT = 100
BRIGHTNESS_LEVELS = [25, 50, 75, 100]
TIMEOUT_LEVELS = [30, 60, 120, 300, 0]  # 0 = never



def load_settings() -> dict[str, Any]:
    settings = {
        "brightness": BRIGHTNESS_DEFAULT,
        "timeout": int(DISPLAY_TIMEOUT_IDLE),
        "logging": True,
        "filament_presets": {name: dict(values) for name, values in FILAMENT_PRESETS_DEFAULT.items()},
    }
    try:
        with open(SETTINGS_FILE, "r", encoding="utf-8") as fh:
            saved = json.load(fh)
        if int(saved.get("brightness", BRIGHTNESS_DEFAULT)) in BRIGHTNESS_LEVELS:
            settings["brightness"] = int(saved["brightness"])
        timeout = int(saved.get("timeout", int(DISPLAY_TIMEOUT_IDLE)))
        if timeout in TIMEOUT_LEVELS:
            settings["timeout"] = timeout
        settings["logging"] = bool(saved.get("logging", True))
        saved_presets = saved.get("filament_presets", {})
        if isinstance(saved_presets, dict):
            for name, defaults in FILAMENT_PRESETS_DEFAULT.items():
                candidate = saved_presets.get(name, {})
                if isinstance(candidate, dict):
                    try:
                        hotend = float(candidate.get("hotend", defaults["hotend"]))
                        bed = float(candidate.get("bed", defaults["bed"]))
                        settings["filament_presets"][name] = {
                            "hotend": max(0.0, min(HOTEND_MAX, hotend)),
                            "bed": max(0.0, min(BED_MAX, bed)),
                        }
                    except (ValueError, TypeError):
                        pass
    except (OSError, ValueError, TypeError):
        pass
    return settings


def save_settings(settings: dict[str, Any]) -> None:
    tmp = SETTINGS_FILE + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(settings, fh)
        Path(tmp).replace(SETTINGS_FILE)
    except (OSError, ValueError, TypeError) as exc:
        print(f"Settings save failed: {exc}")


def api_get(path: str) -> dict[str, Any]:
    r = requests.get(BASE + path, timeout=2)
    r.raise_for_status()
    return r.json()


def api_post(path: str, payload: Any = None, timeout: float = 3.0) -> dict[str, Any]:
    r = requests.post(BASE + path, json=payload, timeout=timeout)
    r.raise_for_status()
    if not r.content:
        return {}
    return r.json()


def dwin_backlight(lcd: T5UIC1_LCD, value: int) -> None:
    """Set T5UIC1 backlight brightness directly (0x00=off, 0xFF=max)."""
    lcd.Byte(0x30)
    lcd.Byte(max(0, min(0xFF, int(value))))
    lcd.Send()


def screen_sleep(lcd: T5UIC1_LCD) -> None:
    # T5UIC1 command 0x30 with 0x00 turns the backlight off.
    dwin_backlight(lcd, 0x00)


def screen_wake(lcd: T5UIC1_LCD, brightness: int = BRIGHTNESS_DEFAULT) -> None:
    # Restore the configured brightness after sleep or a bridge restart.
    dwin_backlight(lcd, round(max(0, min(100, brightness)) * 255 / 100))

def dwin_wake_and_settle(lcd: T5UIC1_LCD, brightness: int = BRIGHTNESS_DEFAULT) -> None:
    """Wake the DWIN backlight and allow the panel to settle before drawing."""
    screen_wake(lcd, brightness)
    time.sleep(1.0)

def get_status() -> dict[str, Any]:
    q = ("/printer/objects/query?extruder&heater_bed&toolhead&"
         "print_stats&virtual_sdcard&fan&gcode_move&"
         "gcode_macro LOAD_FILAMENT&gcode_macro UNLOAD_FILAMENT&"
         "gcode_macro PREHEAT_PLA&gcode_macro PREHEAT_PETG&"
         "gcode_macro COOLDOWN&gcode_macro HOME_ALL&"
         "gcode_macro MOTORS_OFF")
    return api_get(q).get("result", {}).get("status", {})


def get_files() -> list[str]:
    data = api_get("/server/files/list").get("result", [])
    return [x.get("path", "") for x in data if x.get("path", "").lower().endswith(".gcode")]


def get_info() -> dict[str, Any]:
    info = api_get("/printer/info").get("result", {})
    try:
        tool = api_get("/printer/objects/query?toolhead").get("result", {}).get("status", {}).get("toolhead", {})
    except Exception:
        tool = {}
    return {
        "software": info.get("software_version", "unknown"),
        "machine": tool.get("axis_maximum", [0, 0, 0]),
    }


def send_text(lcd: T5UIC1_LCD, x: int, y: int, text: str, size: int = 1,
              fg: int = WHITE, bg: int = BLACK, show_bg: bool = True) -> None:
    lcd.Draw_String(False, show_bg, size, fg, bg, x, y, text[:42])


def draw_wifi_icon(lcd: T5UIC1_LCD, x: int, y: int, bars: int) -> None:
    """Draw a compact Wi-Fi signal glyph."""
    bars = max(0, min(4, int(bars)))

    # Three stepped signal bars plus the centre dot.
    for i, height in enumerate((4, 7, 10)):
        bx = x + i * 5
        by = y + 13 - height
        if i < bars:
            lcd.Draw_Rectangle(1, ACCENT, bx, by, bx + 3, y + 13)
        else:
            lcd.Draw_Rectangle(1, GREY, bx, by, bx + 3, y + 13)

    lcd.Draw_Circle(ACCENT if bars else GREY, x + 7, y + 15, 2)


def header(lcd: T5UIC1_LCD, title: str) -> None:
    lcd.Draw_Rectangle(1, PANEL2, 0, 0, 479, 31)
    lcd.Draw_Rectangle(1, ACCENT, 0, 30, 479, 31)
    send_text(lcd, 10, 7, f"AQUILA  {title}", 2, WHITE, PANEL2)

    with wifi_display_lock:
        connected = bool(wifi_display.get("connected", False))
        signal = int(wifi_display.get("signal", 0))

    if not connected:
        bars = 0
    elif signal >= 75:
        bars = 4
    elif signal >= 50:
        bars = 3
    elif signal >= 25:
        bars = 2
    else:
        bars = 1

    draw_wifi_icon(lcd, 457, 6, bars)

def footer(lcd: T5UIC1_LCD, text: str = "TURN = SELECT   PRESS = ENTER") -> None:
    lcd.Draw_Rectangle(1, DARK, 0, 247, 479, 271)
    msg, kind = get_feedback()
    if msg:
        # Feedback takes over the footer temporarily, so the operator gets
        # confirmation without needing to watch the terminal.
        fg = feedback_colour(kind)
        lcd.Draw_Rectangle(1, fg, 0, 247, 479, 249)
        prefix = {"ok": "OK: ", "warn": "! ", "error": "ERROR: "}.get(kind, "")
        send_text(lcd, 10, 254, (prefix + msg)[:58], 1, WHITE, DARK)
    else:
        lcd.Draw_Rectangle(1, ACCENT, 0, 247, 479, 249)
        send_text(lcd, 10, 254, text, 1, TEXT_DIM, DARK)

def panel(lcd: T5UIC1_LCD, x1: int, y1: int, x2: int, y2: int,
          fill: int = PANEL, border: int = PANEL2) -> None:
    lcd.Draw_Rectangle(1, fill, x1, y1, x2, y2)
    lcd.Draw_Rectangle(0, border, x1, y1, x2, y2)



def draw_menu_icon(lcd: T5UIC1_LCD, kind: str, x: int, y: int,
                   fg: int) -> None:
    """Draw a tiny low-bandwidth vector glyph, avoiding the mismatched stock ICOs."""
    cx = x + 8
    cy = y + 10

    if kind == "PRINT":
        lcd.Draw_Rectangle(0, fg, x + 3, y + 5, x + 14, y + 16)
        lcd.Draw_Rectangle(1, fg, x + 6, y + 2, x + 11, y + 6)
        lcd.Draw_Line(fg, x + 6, y + 9, x + 11, y + 9)
        lcd.Draw_Line(fg, x + 6, y + 12, x + 11, y + 12)
    elif kind == "PREPARE":
        lcd.Draw_Rectangle(0, fg, x + 2, y + 4, x + 14, y + 16)
        lcd.Draw_Line(fg, x + 1, y + 8, x + 15, y + 8)
        lcd.Draw_Line(fg, x + 1, y + 12, x + 15, y + 12)
        lcd.Draw_Line(fg, x + 5, y + 1, x + 5, y + 18)
        lcd.Draw_Line(fg, x + 11, y + 1, x + 11, y + 18)
    elif kind == "MOVE":
        lcd.Draw_Line(fg, cx, y + 1, cx, y + 18)
        lcd.Draw_Line(fg, x + 1, cy, x + 15, cy)
        lcd.Draw_Line(fg, cx, y + 1, x + 5, y + 5)
        lcd.Draw_Line(fg, cx, y + 1, x + 11, y + 5)
        lcd.Draw_Line(fg, cx, y + 18, x + 5, y + 14)
        lcd.Draw_Line(fg, cx, y + 18, x + 11, y + 14)
        lcd.Draw_Line(fg, x + 1, cy, x + 5, y + 6)
        lcd.Draw_Line(fg, x + 1, cy, x + 5, y + 14)
        lcd.Draw_Line(fg, x + 15, cy, x + 11, y + 6)
        lcd.Draw_Line(fg, x + 15, cy, x + 11, y + 14)
    elif kind == "STATUS":
        lcd.Draw_Rectangle(1, fg, x + 2, y + 11, x + 5, y + 17)
        lcd.Draw_Rectangle(1, fg, x + 7, y + 7, x + 10, y + 17)
        lcd.Draw_Rectangle(1, fg, x + 12, y + 3, x + 15, y + 17)
    elif kind == "INFO":
        lcd.Draw_Circle(fg, cx, cy, 7)
        lcd.Draw_Rectangle(1, fg, x + 7, y + 8, x + 9, y + 9)
        lcd.Draw_Rectangle(1, fg, x + 7, y + 11, x + 9, y + 16)
    elif kind == "DOCTOR":
        lcd.Draw_Circle(fg, cx, cy, 7)
        lcd.Draw_Line(fg, x + 2, y + 5, x + 2, y + 12)
        lcd.Draw_Line(fg, x + 2, y + 12, x + 7, y + 17)
        lcd.Draw_Line(fg, x + 7, y + 17, x + 13, y + 17)
        lcd.Draw_Circle(fg, x + 14, y + 17, 2)
    else:
        lcd.Draw_Rectangle(0, fg, x + 3, y + 3, x + 14, y + 17)


def menu_card(lcd: T5UIC1_LCD, x: int, y: int, w: int, h: int,
              label: str, selected: bool, accent: int = ACCENT,
              icon: str | None = None) -> None:
    """Compact control-panel card used by secondary menus."""
    bg = PANEL2 if selected else BLACK
    border = accent if selected else GREY
    lcd.Draw_Rectangle(1, bg, x, y, x + w, y + h)
    lcd.Draw_Rectangle(0, border, x, y, x + w, y + h)
    if icon is None:
        icon = label.split()[0]
    draw_menu_icon(lcd, icon, x + 8, y + max(0, (h - 20) // 2),
                   accent if selected else TEXT_DIM)
    send_text(lcd, x + 31, y + (h // 2) - 7, label[:24], 1, WHITE, bg)


def menu_grid(lcd: T5UIC1_LCD, items: list[str], selected: int,
              y: int = 44, row_h: int = 40, gap: int = 8,
              accent_for: Any = None) -> None:
    """Two-column menu grid, preserving linear encoder order."""
    for i, item in enumerate(items):
        col = i % 2; row = i // 2
        x = 10 + col * 235
        accent = accent_for(item, i) if accent_for else ACCENT
        menu_card(lcd, x, y + row * (row_h + gap), 224, row_h, item, i == selected, accent)
    if len(items) > 4:
        send_text(lcd, 435, 225, f"{selected + 1}/{len(items)}", 1, TEXT_DIM, BLACK)


def menu_row(lcd: T5UIC1_LCD, y: int, label: str, selected: bool,
             accent: int = ACCENT) -> None:
    bg = PANEL2 if selected else BLACK
    border = accent if selected else GREY
    lcd.Draw_Rectangle(1, bg, 10, y, 469, y + 17)
    lcd.Draw_Rectangle(0, border, 10, y, 469, y + 17)

    icon_fg = accent if selected else TEXT_DIM
    draw_menu_icon(lcd, label.split()[0], 17, y, icon_fg)

    # Keep the text clear of the icon and make the selected row read instantly.
    send_text(lcd, 43, y + 2, label[:31], 1, WHITE, bg)


def value_card(lcd: T5UIC1_LCD, x1: int, x2: int, label: str,
               value: str, fg: int = WHITE) -> None:
    panel(lcd, x1, 38, x2, 88)
    send_text(lcd, x1 + 10, 43, label, 1, TEXT_DIM, PANEL)
    send_text(lcd, x1 + 10, 61, value, 2, fg, PANEL)


def draw_status(lcd: T5UIC1_LCD, status: dict[str, Any]) -> None:
    ext = status.get("extruder", {})
    bed = status.get("heater_bed", {})
    tool = status.get("toolhead", {})
    stats = status.get("print_stats", {})
    vsd = status.get("virtual_sdcard", {})

    e_t = float(ext.get("temperature", 0))
    e_s = float(ext.get("target", 0))
    b_t = float(bed.get("temperature", 0))
    b_s = float(bed.get("target", 0))
    pos = tool.get("position", [0, 0, 0, 0])
    raw = str(stats.get("state", "unknown")).lower()
    state = "PRINTING" if raw == "printing" else "PAUSED" if raw == "paused" else "READY"
    progress = float(vsd.get("progress", 0)) * 100

    value_card(lcd, 6, 151, "HOTEND", f"{e_t:4.1f}/{e_s:3.0f} C", HOT)
    value_card(lcd, 158, 303, "BED", f"{b_t:4.1f}/{b_s:3.0f} C", WHITE)
    panel(lcd, 310, 38, 473, 88)
    send_text(lcd, 320, 43, "POSITION", 1, TEXT_DIM, PANEL)
    send_text(lcd, 320, 59, f"X {float(pos[0]):.1f}  Y {float(pos[1]):.1f}", 1, WHITE, PANEL)
    send_text(lcd, 320, 73, f"Z {float(pos[2]):.2f}", 1, WHITE, PANEL)
    send_text(lcd, 390, 73, state, 1, OK if state == "PRINTING" else WARN if state == "PAUSED" else OK, PANEL)

    panel(lcd, 6, 95, 473, 119)
    lcd.Draw_Rectangle(1, DARK, 16, 101, 463, 112)
    fill = 16 + int(447 * max(0, min(100, progress)) / 100)
    if fill > 16:
        lcd.Draw_Rectangle(1, OK if state == "PRINTING" else ACCENT, 16, 101, fill, 112)
    send_text(lcd, 20, 99, f"{progress:5.1f}%", 1, WHITE, DARK)

def format_elapsed(seconds: float) -> str:
    seconds = max(0, int(seconds))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    return f"{h}:{m:02d}:{s:02d}" if h else f"{m}:{s:02d}"



def _doctor_axis_line(status: dict[str, Any], axis: str) -> tuple[str, int]:
    homed = axis_is_homed(status, axis)
    return ("HOMED", OK) if homed else ("NOT HOMED", WARN)


def draw_print_doctor(
    lcd: T5UIC1_LCD,
    status: dict[str, Any],
    probe_ok: bool,
    mesh_ok: bool,
    klipper_ok: bool = True,
) -> None:
    """Read-only machine health dashboard. No printer-changing commands."""
    lcd.Frame_Clear(BLACK)

    stats = status.get("print_stats", {})
    raw_state = str(stats.get("state", "unknown")).lower()
    if raw_state == "printing":
        machine_state, machine_fg = "PRINTING", OK
    elif raw_state == "paused":
        machine_state, machine_fg = "PAUSED", WARN
    elif raw_state == "error":
        machine_state, machine_fg = "ERROR", DANGER
    else:
        machine_state, machine_fg = "READY", OK

    header(lcd, f"PRINT DOCTOR  {machine_state}")

    tool = status.get("toolhead", {})
    pos = tool.get("position", [0, 0, 0, 0])
    ext = status.get("extruder", {})
    bed = status.get("heater_bed", {})
    fan = get_fan_percent(status)
    speed = get_speed_percent(status)

    panel(lcd, 8, 38, 236, 99)
    send_text(lcd, 19, 45, "MACHINE", 1, TEXT_DIM, PANEL)
    send_text(lcd, 19, 63, machine_state, 2, machine_fg, PANEL)
    send_text(lcd, 143, 68, "KLIPPER", 1, TEXT_DIM, PANEL)
    send_text(lcd, 143, 84, "CONNECTED" if klipper_ok else "OFFLINE",
              1, OK if klipper_ok else DANGER, PANEL)

    panel(lcd, 245, 38, 471, 99)
    send_text(lcd, 256, 45, "AXES", 1, TEXT_DIM, PANEL)
    for i, axis in enumerate(("X", "Y", "Z")):
        label, fg = _doctor_axis_line(status, axis)
        x = 256 + i * 69
        send_text(lcd, x, 64, axis, 1, ACCENT, PANEL)
        send_text(lcd, x, 80, "YES" if label == "HOMED" else "NO", 1, fg, PANEL)

    panel(lcd, 8, 106, 236, 157)
    send_text(lcd, 19, 113, "POSITION", 1, TEXT_DIM, PANEL)
    try:
        send_text(lcd, 19, 130, f"X {float(pos[0]):.1f}  Y {float(pos[1]):.1f}", 1, WHITE, PANEL)
        send_text(lcd, 19, 145, f"Z {float(pos[2]):.2f}", 1, WHITE, PANEL)
    except (TypeError, ValueError, IndexError):
        send_text(lcd, 19, 130, "POSITION UNKNOWN", 1, WARN, PANEL)

    panel(lcd, 245, 106, 471, 157)
    send_text(lcd, 256, 113, "HEATERS", 1, TEXT_DIM, PANEL)
    send_text(lcd, 256, 130,
              f"H {float(ext.get('temperature', 0)):.1f}/{float(ext.get('target', 0)):.0f} C",
              1, HOT, PANEL)
    send_text(lcd, 256, 145,
              f"B {float(bed.get('temperature', 0)):.1f}/{float(bed.get('target', 0)):.0f} C",
              1, WHITE, PANEL)

    panel(lcd, 8, 164, 236, 215)
    send_text(lcd, 19, 171, "CALIBRATION", 1, TEXT_DIM, PANEL)
    send_text(lcd, 19, 189, "PROBE", 1, TEXT_DIM, PANEL)
    send_text(lcd, 75, 189, "READY" if probe_ok else "NOT FITTED",
              1, OK if probe_ok else WARN, PANEL)
    send_text(lcd, 19, 204, "MESH", 1, TEXT_DIM, PANEL)
    send_text(lcd, 75, 204, "READY" if (probe_ok and mesh_ok) else "UNAVAILABLE",
              1, OK if probe_ok and mesh_ok else WARN, PANEL)

    panel(lcd, 245, 164, 471, 215)
    send_text(lcd, 256, 171, "CONTROL", 1, TEXT_DIM, PANEL)
    send_text(lcd, 256, 189, f"FAN {fan:3.0f} %", 1, WHITE, PANEL)
    send_text(lcd, 370, 189, f"SPEED {speed:3.0f} %", 1, WHITE, PANEL)
    filename = str(stats.get("filename", ""))
    send_text(lcd, 256, 204,
              filename.split("/")[-1][:31] if filename else "NO ACTIVE FILE",
              1, TEXT_DIM, PANEL)

    progress = max(
        0.0,
        min(100.0, float(status.get("virtual_sdcard", {}).get("progress", 0)) * 100.0),
    )
    panel(lcd, 8, 222, 471, 244)
    send_text(lcd, 19, 229, "PRINT PROGRESS", 1, TEXT_DIM, PANEL)
    send_text(lcd, 151, 228, f"{progress:5.1f} %", 1, ACCENT, PANEL)
    footer(lcd, "READ ONLY   TURN = NAVIGATE   PRESS = BACK")
    lcd.UpdateLCD()


def draw_print_status(
    lcd: T5UIC1_LCD,
    status: dict[str, Any],
    selected: int,
) -> None:
    lcd.Frame_Clear(BLACK)
    stats = status.get("print_stats", {})
    raw_state = str(stats.get("state", "standby")).lower()
    state = "PRINTING" if raw_state == "printing" else "PAUSED" if raw_state == "paused" else "READY"
    header(lcd, f"PRINT  {state}")

    ext = status.get("extruder", {})
    bed = status.get("heater_bed", {})
    tool = status.get("toolhead", {})
    vsd = status.get("virtual_sdcard", {})
    e_t = float(ext.get("temperature", 0))
    e_s = float(ext.get("target", 0))
    b_t = float(bed.get("temperature", 0))
    b_s = float(bed.get("target", 0))
    pos = tool.get("position", [0, 0, 0, 0])
    name = str(stats.get("filename", ""))
    progress = max(0.0, min(100.0, float(vsd.get("progress", 0)) * 100.0))
    elapsed = float(stats.get("print_duration", 0))

    panel(lcd, 8, 37, 471, 62)
    send_text(
        lcd, 18, 43,
        name.split("/")[-1] if state in ("PRINTING", "PAUSED") and name else "No active print",
        1, WHITE, PANEL
    )

    value_card(lcd, 8, 155, "HOTEND", f"{e_t:4.1f}/{e_s:3.0f} C", HOT)
    value_card(lcd, 164, 311, "BED", f"{b_t:4.1f}/{b_s:3.0f} C", WHITE)

    panel(lcd, 320, 67, 471, 96)
    send_text(lcd, 329, 72, f"X {float(pos[0]):.1f}  Y {float(pos[1]):.1f}", 1, WHITE, PANEL)
    send_text(lcd, 329, 85, f"Z {float(pos[2]):.2f}  {format_elapsed(elapsed)}", 1, ACCENT, PANEL)

    panel(lcd, 8, 101, 471, 127)
    lcd.Draw_Rectangle(1, DARK, 17, 108, 462, 119)
    fill = 17 + int(445 * progress / 100.0)
    if fill > 17:
        lcd.Draw_Rectangle(1, OK if state == "PRINTING" else ACCENT, 17, 108, fill, 119)
    send_text(lcd, 22, 104, f"{progress:5.1f}%", 1, WHITE, DARK)

    if state == "PAUSED":
        actions = ["RESUME", "CANCEL", "BACK"]
    elif state == "PRINTING":
        actions = ["PAUSE", "CANCEL", "BACK"]
    else:
        actions = ["BACK"]

    for i, label in enumerate(actions):
        x = 18 + i * 150
        sel = i == selected
        bg = PANEL2 if sel else BLACK
        border = DANGER if label == "CANCEL" else (OK if sel else GREY)
        lcd.Draw_Rectangle(1, bg, x, 139, x + 130, 181)
        lcd.Draw_Rectangle(0, border, x, 139, x + 130, 181)
        send_text(lcd, x + 28, 151, label, 1, WHITE, bg)

    footer(lcd)
    lcd.UpdateLCD()


def action_state(status: dict[str, Any]) -> str:
    return str(status.get("print_stats", {}).get("state", "standby")).lower()


def draw_settings_item(lcd: T5UIC1_LCD, index: int, selected: int, logging_enabled: bool) -> None:
    row = index // 2
    col = index % 2
    x = 10 + col * 235
    y = 157 + row * 45
    w, h = 225, 40
    item = SETTINGS_ITEMS[index]
    bg = PANEL2 if index == selected else BLACK
    border = ACCENT if index == selected else GREY
    lcd.Draw_Rectangle(1, bg, x, y, x + w, y + h)
    lcd.Draw_Rectangle(0, border, x, y, x + w, y + h)

    if item == "LOGGING":
        send_text(lcd, x + 10, y + 11, "DIAG LOG", 1, WHITE, bg)
        # Simple hardware-style toggle: coloured track with a sliding knob.
        tx1, ty1, tx2, ty2 = x + 158, y + 10, x + 210, y + 30
        track = OK if logging_enabled else GREY
        lcd.Draw_Rectangle(1, track, tx1, ty1, tx2, ty2)
        knob_x = tx2 - 10 if logging_enabled else tx1 + 10
        lcd.Draw_Rectangle(1, WHITE, knob_x - 7, ty1 + 3, knob_x + 7, ty2 - 3)
        send_text(lcd, x + 116, y + 11, "ON" if logging_enabled else "OFF", 1,
                  OK if logging_enabled else TEXT_DIM, bg)
    else:
        icon = item.split()[0]
        draw_menu_icon(lcd, icon, x + 8, y + 10, ACCENT if index == selected else TEXT_DIM)
        send_text(lcd, x + 31, y + 11, item, 1, WHITE, bg)


def draw_settings(
    lcd: T5UIC1_LCD,
    brightness: int,
    timeout: int,
    selected: int,
    logging_enabled: bool,
) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "SETTINGS")

    timeout_text = "NEVER" if timeout == 0 else f"{timeout} SEC"

    panel(lcd, 8, 38, 471, 98)
    send_text(lcd, 20, 45, "BRIGHTNESS", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 63, f"{brightness:3d} %", 2, ACCENT, PANEL)

    panel(lcd, 8, 106, 471, 149)
    send_text(lcd, 20, 113, "SLEEP TIMEOUT", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 131, timeout_text, 2, WHITE, PANEL)

    for i in range(len(SETTINGS_ITEMS)):
        draw_settings_item(lcd, i, selected, logging_enabled)

    footer(lcd, "TURN = SELECT   PRESS = EDIT / TOGGLE")
    lcd.UpdateLCD()

def draw_settings_selection(lcd: T5UIC1_LCD, old_selected: int, selected: int, logging_enabled: bool) -> None:
    for i in {old_selected, selected}:
        draw_settings_item(lcd, i, selected, logging_enabled)
    lcd.UpdateLCD()

def draw_settings_edit(lcd: T5UIC1_LCD, kind: str, value: int) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, f"SET {kind}")

    if kind == "BRIGHTNESS":
        shown = f"{int(value):3d} %"
        detail = "DISPLAY BACKLIGHT"
        limits = "25 / 50 / 75 / 100 %"
    else:
        shown = "NEVER" if int(value) == 0 else f"{int(value)} SEC"
        detail = "IDLE BACKLIGHT TIMEOUT"
        limits = "30 / 60 / 120 / 300 / NEVER"

    panel(lcd, 8, 45, 471, 113)
    send_text(lcd, 20, 52, detail, 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 72, shown, 3, ACCENT, PANEL)

    panel(lcd, 8, 125, 471, 172)
    send_text(lcd, 20, 134, "AVAILABLE", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 151, limits, 1, WHITE, PANEL)

    footer(lcd, "TURN = ADJUST   PRESS = APPLY")
    lcd.UpdateLCD()

def draw_wifi_page(
    lcd: T5UIC1_LCD,
    wifi: dict[str, Any],
    selected: int,
) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "WI-FI")

    connected = bool(wifi.get("connected", False))
    ssid = str(wifi.get("ssid", "")) or "NOT CONNECTED"
    signal = int(wifi.get("signal", 0))
    ip = str(wifi.get("ip", "")) or "NO ADDRESS"

    panel(lcd, 8, 40, 471, 82)
    send_text(lcd, 20, 47, "STATUS", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 64, "CONNECTED" if connected else "DISCONNECTED",
              2, OK if connected else DANGER, PANEL)

    panel(lcd, 8, 90, 471, 132)
    send_text(lcd, 20, 97, "NETWORK", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 114, ssid, 1, WHITE, PANEL)

    panel(lcd, 8, 140, 231, 182)
    send_text(lcd, 20, 147, "SIGNAL", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 164, f"{signal:3d} %", 2, ACCENT if connected else TEXT_DIM, PANEL)

    panel(lcd, 240, 140, 471, 182)
    send_text(lcd, 252, 147, "IP ADDRESS", 1, TEXT_DIM, PANEL)
    send_text(lcd, 252, 164, ip, 1, WHITE, PANEL)

    actions = ("BACK", "REFRESH")
    for i, action in enumerate(actions):
        x = 10 + i * 235
        bg = PANEL2 if i == selected else BLACK
        border = ACCENT if i == selected else GREY
        lcd.Draw_Rectangle(1, bg, x, 195, x + 225, 235)
        lcd.Draw_Rectangle(0, border, x, 195, x + 225, 235)
        send_text(lcd, x + 10, 207, action, 1, WHITE, bg)

    footer(lcd, "TURN = SELECT   PRESS = ENTER")
    lcd.UpdateLCD()


def draw_settings_brightness_card(lcd: T5UIC1_LCD, brightness: int) -> None:
    panel(lcd, 8, 38, 471, 98)
    send_text(lcd, 20, 45, "BRIGHTNESS", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 63, f"{brightness:3d} %", 2, ACCENT, PANEL)


def draw_settings_timeout_card(lcd: T5UIC1_LCD, timeout: int) -> None:
    timeout_text = "NEVER" if timeout == 0 else f"{timeout} SEC"
    panel(lcd, 8, 106, 471, 149)
    send_text(lcd, 20, 113, "SLEEP TIMEOUT", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 131, timeout_text, 2, WHITE, PANEL)

def main_menu_window(selected: int) -> tuple[int, int]:
    """Return (first_index, visible_count) for a five-row scrolling home menu."""
    visible = 5
    count = len(MAIN_ITEMS)
    if count <= visible:
        return 0, count

    first = max(0, selected - visible + 1)
    first = min(first, count - visible)
    return first, visible


def draw_main(lcd: T5UIC1_LCD, status: dict[str, Any], selected: int) -> None:
    lcd.Frame_Clear(BLACK)
    raw = str(status.get("print_stats", {}).get("state", "standby")).lower()
    state = "PRINTING" if raw == "printing" else "PAUSED" if raw == "paused" else "READY"

    header(lcd, f"MAIN  {state}")
    draw_status(lcd, status)
    lcd.Draw_Line(PANEL2, 8, 121, 471, 121)

    # Two-column home layout: all nine functions remain directly visible,
    # while the encoder still walks them in the existing linear order.
    # This gives the home screen a proper control-panel feel without changing
    # any navigation semantics.
    card_w = 225
    card_h = 20
    gap_x = 9
    x_positions = (10, 244)
    row_y = 126
    for idx, item in enumerate(MAIN_ITEMS):
        col = idx % 2
        row = idx // 2
        x = x_positions[col]
        y = row_y + row * 22
        selected_row = idx == selected
        bg = PANEL2 if selected_row else BLACK
        border = ACCENT if selected_row else MID
        icon_fg = ACCENT if selected_row else TEXT_DIM
        lcd.Draw_Rectangle(1, bg, x, y, x + card_w, y + card_h)
        lcd.Draw_Rectangle(0, border, x, y, x + card_w, y + card_h)
        icon_kind = "DOCTOR" if item == "PRINT DOCTOR" else item.split()[0]
        draw_menu_icon(lcd, icon_kind, x + 7, y, icon_fg)
        send_text(lcd, x + 29, y + 2, item[:27], 1, WHITE, bg)

    # Position indicator doubles as a useful reminder that the encoder order
    # is linear even though the presentation is a grid.
    send_text(lcd, 407, 235, f"{selected + 1}/{len(MAIN_ITEMS)}", 1, TEXT_DIM, BLACK)

    footer(lcd)
    lcd.UpdateLCD()

def draw_main_status(lcd: T5UIC1_LCD, status: dict[str, Any]) -> None:
    """Redraw only the live header/status band on the home screen."""
    raw = str(status.get("print_stats", {}).get("state", "standby")).lower()
    state = "PRINTING" if raw == "printing" else "PAUSED" if raw == "paused" else "READY"
    header(lcd, f"MAIN  {state}")
    draw_status(lcd, status)
    lcd.UpdateLCD()


def draw_main_selection(lcd: T5UIC1_LCD, old_selected: int, selected: int) -> None:
    """Redraw only the two changed home-menu cards and position indicator."""
    card_w = 225
    card_h = 20
    x_positions = (10, 244)
    row_y = 126

    for idx in {old_selected, selected}:
        item = MAIN_ITEMS[idx]
        col = idx % 2
        row = idx // 2
        x = x_positions[col]
        y = row_y + row * 22
        selected_row = idx == selected
        bg = PANEL2 if selected_row else BLACK
        border = ACCENT if selected_row else MID
        icon_fg = ACCENT if selected_row else TEXT_DIM
        lcd.Draw_Rectangle(1, bg, x, y, x + card_w, y + card_h)
        lcd.Draw_Rectangle(0, border, x, y, x + card_w, y + card_h)
        draw_menu_icon(lcd, item.split()[0], x + 7, y, icon_fg)
        send_text(lcd, x + 29, y + 2, item[:27], 1, WHITE, bg)

    # The indicator has its own small area, so it can be refreshed without
    # touching the rest of the home screen.
    send_text(lcd, 407, 235, f"{selected + 1}/{len(MAIN_ITEMS)}", 1, TEXT_DIM, BLACK)
    lcd.UpdateLCD()

def draw_prepare_selection(lcd: T5UIC1_LCD, old_selected: int, selected: int) -> None:
    for i in {old_selected, selected}:
        item = PREPARE_ITEMS[i]
        accent = (WARN if item in ("PLA PREHEAT", "PETG PREHEAT", "LOAD FILAMENT", "UNLOAD FILAMENT")
                  else DANGER if item == "MOTORS OFF" else ACCENT)
        row = i // 2
        col = i % 2
        x = 10 + col * 235
        y = 40 + row * (37 + 4)
        menu_card(lcd, x, y, 224, 37, item, i == selected, accent)
    lcd.UpdateLCD()


def draw_prepare(lcd: T5UIC1_LCD, selected: int) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "PREPARE")
    menu_grid(lcd, PREPARE_ITEMS, selected, y=40, row_h=37, gap=4,
              accent_for=lambda item, i:
                  WARN if item in ("PLA PREHEAT", "PETG PREHEAT", "LOAD FILAMENT", "UNLOAD FILAMENT")
                  else DANGER if item == "MOTORS OFF" else ACCENT)
    panel(lcd, 10, 209, 469, 238)
    send_text(lcd, 22, 216, "MACHINE PREPARATION", 1, ACCENT, PANEL)
    send_text(lcd, 22, 229, "PREHEAT  •  FILAMENT  •  HOME  •  COOLDOWN", 1, TEXT_DIM, PANEL)
    footer(lcd, "TURN = SELECT   PRESS = ENTER")
    lcd.UpdateLCD()

def draw_leveling_selection(lcd: T5UIC1_LCD, old_selected: int, selected: int) -> None:
    for i in {old_selected, selected}:
        item = LEVEL_ITEMS[i]
        accent = WARN if item in ("MANUAL BED LEVEL", "BED MESH") else ACCENT
        row = i // 2
        col = i % 2
        x = 10 + col * 235
        y = 44 + row * (42 + 6)
        menu_card(lcd, x, y, 224, 42, item, i == selected, accent)
    lcd.UpdateLCD()


def draw_info(lcd: T5UIC1_LCD, info: dict[str, Any]) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "INFO")
    panel(lcd, 8, 41, 471, 99)
    send_text(lcd, 20, 49, "KLIPPER", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 66, str(info.get("software", "unknown")), 2, WHITE, PANEL)

    machine = info.get("machine", [0, 0, 0])
    panel(lcd, 8, 108, 471, 166)
    send_text(lcd, 20, 116, "BUILD VOLUME", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 136, f"{machine[0]:g} x {machine[1]:g} x {machine[2]:g} mm", 2, WHITE, PANEL)

    panel(lcd, 8, 176, 471, 223)
    send_text(lcd, 20, 185, "DISPLAY", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 203, "DWIN T5UIC1  |  480 x 272", 1, ACCENT, PANEL)
    footer(lcd, "PRESS = BACK")
    lcd.UpdateLCD()

def draw_confirm(lcd: T5UIC1_LCD, filename: str, yes: bool) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "PRINT")
    panel(lcd, 8, 42, 471, 105)
    send_text(lcd, 20, 50, "START PRINT?", 2, WARN, PANEL)
    send_text(lcd, 20, 78, filename.split("/")[-1], 1, WHITE, PANEL)
    for i, label in enumerate(("YES", "NO")):
        sel = (i == 0 and yes) or (i == 1 and not yes)
        x = 55 + i * 220
        bg = PANEL2 if sel else BLACK
        border = OK if sel else GREY
        lcd.Draw_Rectangle(1, bg, x, 127, x + 165, 185)
        lcd.Draw_Rectangle(0, border, x, 127, x + 165, 185)
        send_text(lcd, x + 58, 147, label, 2, WHITE, bg)
    footer(lcd, "TURN = SELECT   PRESS = CONFIRM")
    lcd.UpdateLCD()

def draw_action_confirm(lcd: T5UIC1_LCD, action: str, yes: bool) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "CONFIRM")
    panel(lcd, 8, 42, 471, 111)
    send_text(lcd, 20, 51, action, 2, WARN if action != "MOTORS OFF" else DANGER, PANEL)
    prompt = {
        "LOAD FILAMENT": "Load filament? Nozzle may heat to 200 C.",
        "UNLOAD FILAMENT": "Unload filament? Nozzle may heat to 200 C.",
    }.get(action, "Execute this printer action?")
    send_text(lcd, 20, 82, prompt, 1, WHITE, PANEL)
    for i, label in enumerate(("YES", "NO")):
        sel = (i == 0 and yes) or (i == 1 and not yes)
        x = 55 + i * 220
        bg = PANEL2 if sel else BLACK
        border = OK if sel else GREY
        lcd.Draw_Rectangle(1, bg, x, 136, x + 165, 194)
        lcd.Draw_Rectangle(0, border, x, 136, x + 165, 194)
        send_text(lcd, x + 58, 156, label, 2, WHITE, bg)
    footer(lcd, "TURN = SELECT   PRESS = CONFIRM")
    lcd.UpdateLCD()

def draw_files(lcd: T5UIC1_LCD, files: list[str], selected: int) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "PRINT FILES")
    items = ["BACK"] + files
    visible = 7
    start_idx = max(0, min(max(0, len(items) - visible), selected - visible + 1))
    for row, idx in enumerate(range(start_idx, min(start_idx + visible, len(items)))):
        menu_row(lcd, 38 + row * 29, items[idx].split("/")[-1], idx == selected)
    if len(items) > visible:
        send_text(lcd, 406, 232, f"{selected + 1}/{len(items)}", 1, TEXT_DIM, BLACK)
    footer(lcd, "TURN = SELECT   PRESS = ENTER")
    lcd.UpdateLCD()

def quadrature_step(last_ab: tuple[int, int], ab: tuple[int, int]) -> int:
    # +1 = CW, -1 = CCW, 0 = no completed detent
    cw = {(1, 1): (0, 1), (0, 1): (0, 0), (0, 0): (1, 0), (1, 0): (1, 1)}
    ccw = {(1, 1): (1, 0), (1, 0): (0, 0), (0, 0): (0, 1), (0, 1): (1, 1)}
    if cw.get(last_ab) == ab and ab == (1, 1):
        return 1
    if ccw.get(last_ab) == ab and ab == (1, 1):
        return -1
    return 0



def encoder_edge(channel: int) -> None:
    global encoder_state, encoder_detents, encoder_invalid
    ab = (GPIO.input(PIN_A), GPIO.input(PIN_B))
    with encoder_lock:
        previous = encoder_state
        if ab == previous:
            return
        step = quadrature_step(previous, ab)
        if step == 0:
            valid_next = (
                ((previous, ab) in {((1, 1), (0, 1)), ((0, 1), (0, 0)),
                                   ((0, 0), (1, 0)), ((1, 0), (1, 1))})
                or
                ((previous, ab) in {((1, 1), (1, 0)), ((1, 0), (0, 0)),
                                   ((0, 0), (0, 1)), ((0, 1), (1, 1))})
            )
            if not valid_next:
                encoder_invalid += 1
        encoder_state = ab
        if step:
            encoder_detents += step

def virtual_input_poll(stop_event: threading.Event) -> None:
    """Listen for LEFT/RIGHT/CLICK events from the remote encoder helper."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        try:
            os.unlink(VIRTUAL_INPUT_SOCKET)
        except FileNotFoundError:
            pass

        sock.bind(VIRTUAL_INPUT_SOCKET)
        sock.settimeout(0.25)
        print(f"Virtual encoder socket: {VIRTUAL_INPUT_SOCKET}", flush=True)

        while not stop_event.is_set():
            try:
                data = sock.recv(64)
            except socket.timeout:
                continue
            except OSError as exc:
                if stop_event.is_set():
                    break
                print(f"Virtual encoder socket error: {exc}", flush=True)
                continue

            event = data.decode("ascii", errors="ignore").strip().upper()
            if event not in ("LEFT", "RIGHT", "CLICK"):
                continue

            with virtual_input_lock:
                virtual_input_events.append(event)

    finally:
        sock.close()
        try:
            os.unlink(VIRTUAL_INPUT_SOCKET)
        except FileNotFoundError:
            pass


def drain_virtual_input() -> tuple[int, bool]:
    """Convert queued remote encoder events into the main loop's input model."""
    global virtual_input_events

    step = 0
    button = False
    with virtual_input_lock:
        while virtual_input_events:
            event = virtual_input_events.popleft()
            if event == "RIGHT":
                step += 1
            elif event == "LEFT":
                step -= 1
            elif event == "CLICK":
                button = True

    return step, button


def run_gcode(script: str, timeout: float = 3.0) -> None:
    # Current Moonraker GCode API. See /printer/gcode/script.
    # Homing can legitimately take longer than the short UI command timeout.
    api_post("/printer/gcode/script", {"script": script}, timeout=timeout)


def start_print(filename: str) -> None:
    r = requests.post(BASE + "/printer/print/start",
                      params={"filename": filename},
                      timeout=3)
    r.raise_for_status()


def pause_print() -> None:
    api_post("/printer/print/pause")


def resume_print() -> None:
    api_post("/printer/print/resume")


def cancel_print() -> None:
    api_post("/printer/print/cancel")


def get_homed_axes(status: dict[str, Any]) -> str:
    """Return Klipper's current homed-axis bitmap, e.g. 'xyz'."""
    return str(status.get("toolhead", {}).get("homed_axes", "")).lower()


def axis_is_homed(status: dict[str, Any], axis: str) -> bool:
    return axis.lower() in get_homed_axes(status)


def axis_position(status: dict[str, Any], axis: str) -> float:
    pos = status.get("toolhead", {}).get("position", [0, 0, 0, 0])
    index = {"X": 0, "Y": 1, "Z": 2}.get(axis.upper())
    if index is None or index >= len(pos):
        return 0.0
    try:
        return float(pos[index])
    except (TypeError, ValueError):
        return 0.0


def axis_limits(status: dict[str, Any], axis: str) -> tuple[float | None, float | None]:
    """Return Klipper-reported travel limits when available."""
    tool = status.get("toolhead", {})
    axis_min = tool.get("axis_minimum", [])
    axis_max = tool.get("axis_maximum", [])
    index = {"X": 0, "Y": 1, "Z": 2}.get(axis.upper())
    if index is None:
        return None, None

    try:
        low = float(axis_min[index])
        high = float(axis_max[index])
        if high < low:
            return None, None
        return low, high
    except (IndexError, TypeError, ValueError):
        return None, None


# Shared with the motion helpers.  It is initialised by main() before
# the UI event loop starts, so jog/home operations can request an immediate
# status refresh without depending on main()'s local scope.
status_refresh_requested: threading.Event | None = None


def jog_axis(axis: str, distance: float, status: dict[str, Any]) -> float:
    """Issue a bounded relative jog without allowing a stale position to crash the UI."""
    if axis not in MOVE_FEED:
        return 0.0

    position = axis_position(status, axis)
    low, high = axis_limits(status, axis)

    # Clamp against the limits reported by Klipper. This is the fast UI-side
    # guard; Klipper remains the final authority on whether a move is legal.
    if low is not None and high is not None:
        target = max(low, min(high, position + distance))
        distance = target - position

    if abs(distance) < 0.0001:
        print(
            f"Jog {axis} blocked at travel limit "
            f"(position={position:g}, limits={low:g}..{high:g})."
            if low is not None and high is not None
            else f"Jog {axis} blocked at travel limit.",
            flush=True,
        )
        return 0.0

    script = f"G91\nG1 {axis}{distance:g} F{MOVE_FEED[axis]}\nG90"

    try:
        run_gcode(script)
    except requests.HTTPError as exc:
        # Position can change between the status sample and G-code submission.
        # Treat Klipper's 400 Move out of range as a harmless limit race.
        response = getattr(exc, "response", None)
        body = response.text if response is not None else str(exc)
        if response is not None and response.status_code == 400 and "Move out of range" in body:
            print(
                f"Jog {axis} blocked by Klipper travel limit "
                f"(position={position:g}, limits={low:g}..{high:g}).",
                flush=True,
            )
            if status_refresh_requested is not None:
                status_refresh_requested.set()
            return 0.0
        raise
    except requests.RequestException as exc:
        # A transient Moonraker HTTP failure must not kill the DWIN UI.
        print(f"Encoder jog request failed: {exc}", flush=True)
        if status_refresh_requested is not None:
            status_refresh_requested.set()
        return 0.0

    return distance


def home_axis(axis: str, status: dict[str, Any]) -> bool:
    """Home one axis through Klipper, with UI-side state checks."""
    axis = axis.upper()
    if axis not in ("X", "Y", "Z"):
        return False

    state = action_state(status)
    if state in ("printing", "paused"):
        print(f"Home {axis} blocked while printer state is {state}.", flush=True)
        return False

    try:
        # A G28 response is not guaranteed within the normal 3 s UI timeout,
        # especially on a cold start or when the machine is mechanically slow.
        # Give homing its own generous API window so the UI thread does not
        # mistake a slow but healthy homing cycle for a fatal error.
        run_gcode(f"G28 {axis}", timeout=30.0)
    except requests.exceptions.Timeout:
        # Moonraker may still be executing the G28 even if its HTTP response
        # exceeded our window. Do not tear down the entire DWIN control panel.
        print(
            f"HOME {axis} API response timed out; homing may still be running.",
            flush=True,
        )
        if status_refresh_requested is not None:
            status_refresh_requested.set()
    except Exception as exc:
        print(f"HOME {axis} failed: {exc}", flush=True)
        return False

    print(f"HOME {axis} requested", flush=True)
    if status_refresh_requested is not None:
        status_refresh_requested.set()
    return True


def draw_move_axes(lcd: T5UIC1_LCD, selected: int) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "MOVE")

    items = MOVE_AXES
    # Keep the movement page symmetrical: machine actions occupy the first
    # row, followed by the three physical axes.
    menu_grid(lcd, items, selected, y=44, row_h=42, gap=8,
              accent_for=lambda item, i:
                  DANGER if item == "MOTORS OFF"
                  else WARN if item == "HOME ALL"
                  else ACCENT)

    panel(lcd, 10, 183, 469, 220)
    send_text(lcd, 22, 191, "MOTION CONTROL", 1, ACCENT, PANEL)
    send_text(lcd, 22, 207, "HOME ALL  •  MOTORS OFF  •  X / Y / Z  •  SAFE JOG", 1, TEXT_DIM, PANEL)
    footer(lcd, "TURN = SELECT   PRESS = ENTER")
    lcd.UpdateLCD()

def motion_action_locked(status: dict[str, Any]) -> tuple[bool, str]:
    """Return whether movement actions should currently be blocked."""
    if process_display_state in ("heating", "cooling", "homing", "running"):
        return True, process_display_name.upper() or "BUSY"
    print_state = str(status.get("print_stats", {}).get("state", "standby")).lower()
    if print_state in ("printing", "paused"):
        return True, print_state.upper()
    return False, "READY"


def draw_jog(
    lcd: T5UIC1_LCD,
    axis: str,
    step_index: int,
    selected: int,
    status: dict[str, Any],
    encoder_mode: bool = True,
) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, f"JOG {axis}")

    step = MOVE_STEPS[step_index]
    position = axis_position(status, axis)
    homed = axis_is_homed(status, axis)
    homed_axes = get_homed_axes(status).upper() or "NONE"
    low, high = axis_limits(status, axis)

    panel(lcd, 8, 38, 471, 91)
    send_text(lcd, 20, 44, "X", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 62, f"{axis_position(status, 'X'):7.2f}", 2, ACCENT if axis_is_homed(status, 'X') else WARN, PANEL)
    send_text(lcd, 145, 44, "Y", 1, TEXT_DIM, PANEL)
    send_text(lcd, 145, 62, f"{axis_position(status, 'Y'):7.2f}", 2, ACCENT if axis_is_homed(status, 'Y') else WARN, PANEL)
    send_text(lcd, 270, 44, "Z", 1, TEXT_DIM, PANEL)
    send_text(lcd, 270, 62, f"{axis_position(status, 'Z'):7.2f}", 2, ACCENT if axis_is_homed(status, 'Z') else WARN, PANEL)
    send_text(lcd, 390, 47, "HOME", 1, TEXT_DIM, PANEL)
    send_text(lcd, 390, 64, homed_axes, 1, OK if homed else WARN, PANEL)

    panel(lcd, 8, 98, 471, 124)
    send_text(lcd, 20, 105, "STEP SIZE", 1, TEXT_DIM, PANEL)
    send_text(lcd, 110, 103, f"{step:g} mm", 2, ACCENT, PANEL)
    if low is not None and high is not None:
        limit_text = f"{low:g} .. {high:g}"
    else:
        limit_text = "KLIPPER LIMITS"
    send_text(lcd, 210, 106, limit_text, 1, TEXT_DIM, PANEL)
    send_text(
        lcd, 382, 106,
        "READY" if homed else "HOME",
        1,
        OK if homed else WARN,
        PANEL,
    )

    actions = ("- MOVE", "+ MOVE", "STEP", "HOME", "BACK")
    for i, label in enumerate(actions):
        x = 7 + i * 95
        sel = (not encoder_mode) and i == selected
        bg = PANEL2 if sel else BLACK
        if label == "HOME":
            accent = WARN if not homed else ACCENT
        elif label == "BACK":
            accent = TEXT_DIM
        else:
            accent = ACCENT
        border = accent if sel else GREY
        lcd.Draw_Rectangle(1, bg, x, 137, x + 88, 195)
        lcd.Draw_Rectangle(0, border, x, 137, x + 88, 195)
        send_text(lcd, x + 12, 158, label, 1, WHITE, bg)

    if encoder_mode:
        mode_text = f"ENCODER JOG  {axis}   ±{step:g} mm / DETENT"
        footer(lcd, mode_text + "   PRESS = MENU")
    else:
        footer(lcd, "MENU MODE   TURN = SELECT   PRESS = ACTION")

    lcd.UpdateLCD()

def draw_jog_live_overlay(
    lcd: T5UIC1_LCD,
    axis: str,
    step_index: int,
    status: dict[str, Any],
) -> None:
    """Update only the live position/homing areas on the jog page."""
    step = MOVE_STEPS[step_index]
    position = axis_position(status, axis)
    homed = axis_is_homed(status, axis)
    homed_axes = get_homed_axes(status).upper() or "NONE"

    lcd.Draw_Rectangle(1, PANEL, 8, 38, 471, 91)
    send_text(lcd, 20, 44, "X", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 62, f"{axis_position(status, 'X'):7.2f}", 2, ACCENT if axis_is_homed(status, 'X') else WARN, PANEL)
    send_text(lcd, 145, 44, "Y", 1, TEXT_DIM, PANEL)
    send_text(lcd, 145, 62, f"{axis_position(status, 'Y'):7.2f}", 2, ACCENT if axis_is_homed(status, 'Y') else WARN, PANEL)
    send_text(lcd, 270, 44, "Z", 1, TEXT_DIM, PANEL)
    send_text(lcd, 270, 62, f"{axis_position(status, 'Z'):7.2f}", 2, ACCENT if axis_is_homed(status, 'Z') else WARN, PANEL)
    send_text(lcd, 390, 47, "HOME", 1, TEXT_DIM, PANEL)
    send_text(lcd, 390, 64, homed_axes, 1, OK if homed else WARN, PANEL)

    lcd.Draw_Rectangle(1, PANEL, 8, 98, 471, 124)
    send_text(lcd, 20, 105, "STEP SIZE", 1, TEXT_DIM, PANEL)
    send_text(lcd, 110, 103, f"{step:g} mm", 2, ACCENT, PANEL)
    send_text(
        lcd, 210, 106,
        "READY" if homed else "HOME AXIS FIRST",
        1,
        OK if homed else WARN,
        PANEL,
    )
    lcd.UpdateLCD()



def draw_leveling(lcd: T5UIC1_LCD, selected: int, probe_ready: bool = False, mesh_ready: bool = False) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "LEVELING")
    menu_grid(lcd, LEVEL_ITEMS, selected, y=44, row_h=42, gap=6,
              accent_for=lambda item, i: WARN if item in ("MANUAL BED LEVEL", "BED MESH") else ACCENT)
    panel(lcd, 10, 191, 469, 229)
    send_text(lcd, 22, 198, "CALIBRATION TOOLS", 1, ACCENT, PANEL)
    probe_text = "PROBE READY" if probe_ready else "NO PROBE: MANUAL MODE"
    mesh_text = "MESH READY" if (probe_ready and mesh_ready) else "MESH LOCKED"
    send_text(lcd, 22, 215, f"{probe_text}  •  {mesh_text}", 1, OK if probe_ready and mesh_ready else WARN, PANEL)
    footer(lcd, "TURN = SELECT   PRESS = ENTER")
    lcd.UpdateLCD()

def draw_bed_level(
    lcd: T5UIC1_LCD,
    point_index: int,
    action_selected: int,
    z_position: float,
    step_index: int,
    level_state: str = "ready",
    level_error: str = "",
) -> None:
    lcd.Frame_Clear(BLACK)
    point_name, x, y = BED_LEVEL_POINTS[point_index]
    header(lcd, f"BED LEVEL  {point_index + 1}/4")

    panel(lcd, 8, 39, 471, 91)
    send_text(lcd, 20, 46, point_name, 2, ACCENT, PANEL)
    send_text(lcd, 250, 50, f"X {x:g}  Y {y:g}", 1, WHITE, PANEL)
    send_text(lcd, 250, 68, f"Z {z_position:.2f} mm", 1, WHITE, PANEL)

    panel(lcd, 8, 100, 471, 127)
    if level_state == "starting":
        level_hint = "MOVING TO START POSITION..."
        level_hint_fg = WARN
    elif level_state == "moving":
        level_hint = "MOVING..."
        level_hint_fg = WARN
    elif level_state == "finishing":
        level_hint = "RETURNING TO SAFE HEIGHT..."
        level_hint_fg = WARN
    elif level_state == "error":
        level_hint = "MOVE ERROR - CHECK KLIPPER"
        level_hint_fg = DANGER
    elif level_state == "complete":
        level_hint = "LEVELING FINISHED"
        level_hint_fg = OK
    else:
        level_hint = "ADJUST THIS SCREW WITH PAPER / FEELER"
        level_hint_fg = TEXT_DIM
    send_text(lcd, 20, 107, level_hint, 1, level_hint_fg, PANEL)
    if level_error:
        send_text(lcd, 250, 107, "ERROR", 1, DANGER, PANEL)

    for i, item in enumerate(BED_LEVEL_ACTIONS):
        row = i % 3
        col = i // 3
        x1 = 10 + col * 230
        y1 = 136 + row * 30
        sel = i == action_selected
        bg = PANEL2 if sel else BLACK
        border = OK if sel else GREY
        lcd.Draw_Rectangle(1, bg, x1, y1, x1 + 215, y1 + 24)
        lcd.Draw_Rectangle(0, border, x1, y1, x1 + 215, y1 + 24)
        send_text(lcd, x1 + 82, y1 + 5, item, 1, WHITE, bg)

    send_text(lcd, 20, 225, f"STEP {LEVEL_STEPS[step_index]:g} mm   CORNER {point_index + 1}/{LEVEL_POINT_COUNT}", 1, TEXT_DIM, BLACK)
    footer(lcd, "Z-/Z+ ADJUST   STEP CHANGE   NEXT CORNER   DONE")
    lcd.UpdateLCD()


def draw_z_level(
    lcd: T5UIC1_LCD,
    action_selected: int,
    z_position: float,
    step_index: int,
    level_state: str = "ready",
    level_error: str = "",
) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "Z LEVEL (PAPER TEST)")
    panel(lcd, 8, 39, 471, 94)
    send_text(lcd, 20, 46, "LIVE Z OFFSET", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 64, f"{z_position:+.3f} mm", 3, ACCENT, PANEL)

    panel(lcd, 8, 103, 471, 137)
    level_hint = (
        "MOVING..." if level_state in ("starting", "moving", "finishing")
        else "MOVE ERROR - CHECK KLIPPER" if level_state == "error"
        else "CENTRE OF BED - LIVE OFFSET"
    )
    level_hint_fg = (
        WARN if level_state in ("starting", "moving", "finishing")
        else DANGER if level_state == "error"
        else TEXT_DIM
    )
    send_text(lcd, 20, 111, level_hint, 1, level_hint_fg, PANEL)
    send_text(lcd, 170, 111, "PAPER / FEELER TEST", 1, WHITE, PANEL)
    send_text(lcd, 20, 139, "SAVE COMMITS THIS OFFSET TO Z ENDSTOP.", 1, TEXT_DIM, BLACK)

    for i, item in enumerate(Z_LEVEL_ACTIONS):
        x1 = 10 + i * 115
        sel = i == action_selected
        bg = PANEL2 if sel else BLACK
        border = OK if sel else GREY
        lcd.Draw_Rectangle(1, bg, x1, 153, x1 + 103, 205)
        lcd.Draw_Rectangle(0, border, x1, 153, x1 + 103, 205)
        send_text(lcd, x1 + 34, 173, item, 1, WHITE, bg)

    send_text(lcd, 20, 219, f"STEP {LEVEL_STEPS[step_index]:g} mm", 1, TEXT_DIM, BLACK)
    footer(lcd, "TURN = SELECT   PRESS = ACTION   BACK = EXIT")
    lcd.UpdateLCD()


def probe_detected() -> bool:
    """Detect a configured Klipper probe without enabling any probe motion."""
    try:
        objects = api_get("/printer/objects/list").get("result", {}).get("objects", [])
        return any(name in objects for name in ("bltouch", "probe"))
    except Exception:
        return False


def bed_mesh_configured() -> bool:
    """Detect Klipper's bed_mesh object without issuing any motion command."""
    try:
        objects = api_get("/printer/objects/list").get("result", {}).get("objects", [])
        return "bed_mesh" in objects
    except Exception:
        return False


def probe_capabilities() -> tuple[bool, bool]:
    """Return (probe_configured, bed_mesh_configured). This is read-only."""
    return probe_detected(), bed_mesh_configured()


def draw_probe_setup(lcd: T5UIC1_LCD, detected: bool, mesh_ready: bool = False) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "PROBE SETUP")
    panel(lcd, 8, 42, 471, 112)
    state = "PROBE DETECTED" if detected else "NOT INSTALLED"
    fg = OK if detected else WARN
    send_text(lcd, 20, 51, "BLTOUCH / 3DTOUCH", 2, WHITE, PANEL)
    send_text(lcd, 20, 82, state, 2, fg, PANEL)

    panel(lcd, 8, 123, 471, 202)
    if detected:
        send_text(lcd, 20, 134, "PROBE CONFIGURATION FOUND.", 1, OK, PANEL)
        send_text(lcd, 20, 153, "BED MESH: READY" if mesh_ready else "BED MESH: NOT CONFIGURED", 1, OK if mesh_ready else WARN, PANEL)
        send_text(lcd, 20, 172, "NO PROBE MOTION IS TESTED HERE.", 1, TEXT_DIM, PANEL)
    else:
        send_text(lcd, 20, 134, "NO PROBE CONFIGURATION FOUND.", 1, WARN, PANEL)
        send_text(lcd, 20, 153, "MANUAL LEVELING REMAINS AVAILABLE.", 1, WHITE, PANEL)
        send_text(lcd, 20, 172, "NO PROBE COMMANDS ARE SENT.", 1, WHITE, PANEL)
    footer(lcd, "PRESS = BACK")
    lcd.UpdateLCD()


def draw_bed_mesh(lcd: T5UIC1_LCD, level_state: str = "ready", level_error: str = "", probe_ok: bool = False, mesh_ok: bool = False) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "BED MESH")
    panel(lcd, 8, 42, 471, 111)
    send_text(lcd, 20, 51, "AUTOMATIC PROBING", 2, ACCENT, PANEL)
    if level_state == "starting":
        state_text, state_fg = "HOMING / STARTING...", WARN
    elif level_state == "moving":
        state_text, state_fg = "PROBING BED...", WARN
    elif level_state == "finishing":
        state_text, state_fg = "FINISHING MESH...", WARN
    elif level_state == "complete":
        state_text, state_fg = "MESH COMPLETE", OK
    elif level_state == "error":
        state_text, state_fg = "MESH FAILED", DANGER
    elif not probe_ok:
        state_text, state_fg = "LOCKED: NO PROBE", WARN
    elif not mesh_ok:
        state_text, state_fg = "LOCKED: BED_MESH MISSING", WARN
    else:
        state_text, state_fg = "READY", OK
    send_text(lcd, 20, 87, state_text, 2, state_fg, PANEL)

    panel(lcd, 8, 123, 471, 205)
    if not probe_ok:
        send_text(lcd, 20, 134, "A PROBE IS REQUIRED.", 1, WARN, PANEL)
        send_text(lcd, 20, 153, "MANUAL 4-POINT LEVELING IS STILL SAFE.", 1, WHITE, PANEL)
        send_text(lcd, 20, 172, "NO PROBE COMMANDS WILL BE SENT.", 1, WHITE, PANEL)
    elif not mesh_ok:
        send_text(lcd, 20, 134, "BED_MESH IS NOT CONFIGURED.", 1, WARN, PANEL)
        send_text(lcd, 20, 153, "ADD [bed_mesh] TO PRINTER.CFG FIRST.", 1, WHITE, PANEL)
        send_text(lcd, 20, 172, "NO MESH COMMAND WILL BE SENT.", 1, WHITE, PANEL)
    elif level_state == "error":
        send_text(lcd, 20, 134, "CHECK KLIPPER / PROBE CONFIGURATION.", 1, DANGER, PANEL)
        send_text(lcd, 20, 153, level_error[:48] if level_error else "MESH OPERATION FAILED", 1, WHITE, PANEL)
    else:
        send_text(lcd, 20, 134, "G28 + SAFE Z + BED_MESH_CALIBRATE", 1, OK, PANEL)
        send_text(lcd, 20, 153, "MESH RUNS ONLY WHEN PROBE + MESH EXIST.", 1, TEXT_DIM, PANEL)
        send_text(lcd, 20, 172, "KEEP HANDS CLEAR WHILE PROBING.", 1, WHITE, PANEL)

    footer(lcd, "PRESS = BACK")
    lcd.UpdateLCD()


def start_manual_bed_level() -> tuple[int, float]:
    """Queue a complete, safe start sequence for the four-corner wizard."""
    leveling_action_queue.put(("start_bed",))
    return 0, LEVEL_TRAVEL_Z


def queue_bed_level_point(point_index: int, z_position: float) -> None:
    leveling_action_queue.put(("bed_point", point_index, z_position))


def queue_bed_level_z(delta: float) -> None:
    leveling_action_queue.put(("bed_z", delta))


def queue_finish_bed_level() -> None:
    leveling_action_queue.put(("bed_finish",))


def start_z_level() -> float:
    leveling_action_queue.put(("start_z",))
    return LEVEL_TRAVEL_Z


def queue_finish_z_level() -> None:
    leveling_action_queue.put(("z_finish",))


def queue_bed_mesh() -> None:
    leveling_action_queue.put(("bed_mesh",))


def bed_level_z_move(delta: float, z_position: float) -> float:
    new_z = max(LEVEL_MIN_Z, min(LEVEL_MAX_Z, z_position + delta))
    actual_delta = new_z - z_position
    if abs(actual_delta) > 1e-9:
        queue_bed_level_z(actual_delta)
    return new_z


def finish_leveling() -> None:
    queue_finish_bed_level()


def get_fan_percent(status: dict[str, Any]) -> float:
    return max(0.0, min(100.0, float(status.get("fan", {}).get("speed", 0.0)) * 100.0))


def get_speed_percent(status: dict[str, Any]) -> float:
    return max(
        0.0,
        min(200.0, float(status.get("gcode_move", {}).get("speed_factor", 1.0)) * 100.0)
    )


def set_fan_percent(value: float) -> None:
    value = max(0.0, min(100.0, value))
    pwm = round(value * 255.0 / 100.0)
    run_gcode(f"M106 S{pwm}")


def set_speed_percent(value: float) -> None:
    value = max(SPEED_MIN, min(SPEED_MAX, value))
    run_gcode(f"M220 S{value:g}")


def draw_control(lcd: T5UIC1_LCD, status: dict[str, Any], selected: int) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "CONTROL")

    fan = get_fan_percent(status)
    speed = get_speed_percent(status)

    panel(lcd, 8, 38, 235, 91)
    send_text(lcd, 20, 45, "FAN", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 62, f"{fan:3.0f} %", 2, ACCENT, PANEL)
    send_text(lcd, 150, 67, "PART COOLING", 1, WHITE, PANEL)

    panel(lcd, 245, 38, 471, 91)
    send_text(lcd, 257, 45, "PRINT SPEED", 1, TEXT_DIM, PANEL)
    send_text(lcd, 257, 62, f"{speed:3.0f} %", 2, WARN, PANEL)
    send_text(lcd, 257, 67, "M220", 1, WHITE, PANEL)

    for i, item in enumerate(CONTROL_ITEMS):
        x = 10 + i * 153
        accent = WARN if item == "PRINT SPEED" else ACCENT
        menu_card(lcd, x, 108, 143, 54, item, i == selected, accent)
    footer(lcd, "TURN = SELECT   PRESS = EDIT / ENTER")
    lcd.UpdateLCD()


def draw_control_selection(lcd: T5UIC1_LCD, old_selected: int, selected: int) -> None:
    """Redraw only the changed CONTROL selection cards."""
    for i in {old_selected, selected}:
        item = CONTROL_ITEMS[i]
        x = 10 + i * 153
        accent = WARN if item == "PRINT SPEED" else ACCENT
        menu_card(lcd, x, 108, 143, 54, item, i == selected, accent)
    lcd.UpdateLCD()


def draw_control_edit(
    lcd: T5UIC1_LCD,
    kind: str,
    value: float,
    status: dict[str, Any],
) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, f"SET {kind}")

    if kind == "FAN":
        current = get_fan_percent(status)
        step = FAN_STEP
        accent = ACCENT
        limits = "0 - 100 %"
    else:
        current = get_speed_percent(status)
        step = SPEED_STEP
        accent = WARN
        limits = "50 - 150 %"

    panel(lcd, 8, 40, 471, 101)
    send_text(lcd, 20, 47, "CURRENT", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 67, f"{current:3.0f} %", 2, WHITE, PANEL)

    panel(lcd, 8, 111, 471, 172)
    send_text(lcd, 22, 120, "TARGET", 1, TEXT_DIM, PANEL)
    send_text(lcd, 22, 137, f"{value:3.0f} %", 2, accent, PANEL)
    send_text(lcd, 170, 141, f"STEP {step:g} %", 1, ACCENT, PANEL)
    lcd.Draw_Rectangle(0, OK, 25, 185, 454, 217)
    send_text(lcd, 45, 194, f"ROTATE = ADJUST   LIMIT {limits}", 1, WHITE, BLACK)
    footer(lcd, "PRESS = APPLY TARGET")
    lcd.UpdateLCD()


def temp_limit(kind: str) -> float:
    return HOTEND_MAX if kind == "HOTEND" else BED_MAX


def get_current_target(status: dict[str, Any], kind: str) -> float:
    if kind == "HOTEND":
        return float(status.get("extruder", {}).get("target", 0))
    return float(status.get("heater_bed", {}).get("target", 0))


def set_temperature(kind: str, value: float) -> None:
    value = max(0.0, min(temp_limit(kind), value))
    if kind == "HOTEND":
        run_gcode(f"M104 S{value:g}")
    else:
        run_gcode(f"M140 S{value:g}")


def draw_filament_presets(lcd: T5UIC1_LCD, settings: dict[str, Any], selected: int) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "FILAMENT PRESETS")
    presets = settings.get("filament_presets", FILAMENT_PRESETS_DEFAULT)
    for i, name in enumerate(FILAMENT_PRESET_NAMES):
        x = 10 + i * 153
        values = presets.get(name, FILAMENT_PRESETS_DEFAULT[name])
        menu_card(lcd, x, 50, 143, 105, name, i == selected, ACCENT)
        send_text(lcd, x + 10, 82, f"NOZ {float(values.get('hotend', 0)):3.0f} C", 1, WHITE, PANEL)
        send_text(lcd, x + 10, 106, f"BED {float(values.get('bed', 0)):3.0f} C", 1, WHITE, PANEL)
    menu_card(lcd, 10, 170, 143, 54, "BACK", selected == len(FILAMENT_PRESET_NAMES), ACCENT)
    menu_card(lcd, 163, 170, 143, 54, "EDIT", selected == len(FILAMENT_PRESET_NAMES) + 1, WARN)
    footer(lcd, "TURN = SELECT   PRESS = APPLY / EDIT")
    lcd.UpdateLCD()


def draw_filament_edit(lcd: T5UIC1_LCD, name: str, kind: str, value: float) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, f"EDIT {name}")
    panel(lcd, 8, 42, 471, 112)
    send_text(lcd, 20, 52, "NOZZLE" if kind == "HOTEND" else "BED", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 75, f"{value:3.0f} C", 3, HOT if kind == "HOTEND" else WHITE, PANEL)
    send_text(lcd, 20, 120, "TURN = ADJUST  PRESS = SAVE", 1, WHITE, PANEL)
    footer(lcd, "PRESS = SAVE   BACK = CANCEL")
    lcd.UpdateLCD()


def draw_temperature(lcd: T5UIC1_LCD, status: dict[str, Any], selected: int) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, "TEMPERATURE")

    ext = status.get("extruder", {})
    bed = status.get("heater_bed", {})

    e_actual = float(ext.get("temperature", 0))
    e_target = float(ext.get("target", 0))
    b_actual = float(bed.get("temperature", 0))
    b_target = float(bed.get("target", 0))

    panel(lcd, 8, 38, 235, 103)
    send_text(lcd, 20, 45, "HOTEND", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 64, f"{e_actual:5.1f} / {e_target:3.0f} C", 2, HOT, PANEL)

    panel(lcd, 245, 38, 471, 103)
    send_text(lcd, 257, 45, "BED", 1, TEXT_DIM, PANEL)
    send_text(lcd, 257, 64, f"{b_actual:5.1f} / {b_target:3.0f} C", 2, WHITE, PANEL)

    temp_card_w = 112
    temp_gap = 7
    for i, item in enumerate(TEMP_ITEMS):
        x = 10 + i * (temp_card_w + temp_gap)
        accent = WARN if item in ("HOTEND", "BED") else ACCENT
        menu_card(lcd, x, 116, temp_card_w, 54, item, i == selected, accent)

    footer(lcd, "TURN = SELECT   PRESS = EDIT / ENTER")
    lcd.UpdateLCD()


def draw_temperature_selection(lcd: T5UIC1_LCD, old_selected: int, selected: int) -> None:
    temp_card_w = 112
    temp_gap = 7
    for i in {old_selected, selected}:
        item = TEMP_ITEMS[i]
        x = 10 + i * (temp_card_w + temp_gap)
        accent = WARN if item in ("HOTEND", "BED") else ACCENT
        menu_card(lcd, x, 116, temp_card_w, 54, item, i == selected, accent)
    lcd.UpdateLCD()


def draw_temperature_edit(
    lcd: T5UIC1_LCD,
    kind: str,
    value: float,
    status: dict[str, Any],
) -> None:
    lcd.Frame_Clear(BLACK)
    header(lcd, f"SET {kind}")

    actual = (
        float(status.get("extruder", {}).get("temperature", 0))
        if kind == "HOTEND"
        else float(status.get("heater_bed", {}).get("temperature", 0))
    )

    panel(lcd, 8, 40, 471, 101)
    send_text(lcd, 20, 47, "CURRENT", 1, TEXT_DIM, PANEL)
    send_text(lcd, 20, 67, f"{actual:5.1f} C", 2, WHITE, PANEL)

    panel(lcd, 8, 111, 471, 172)
    send_text(lcd, 22, 120, "TARGET", 1, TEXT_DIM, PANEL)
    send_text(lcd, 22, 137, f"{value:5.0f} C", 2, HOT if kind == "HOTEND" else WHITE, PANEL)
    send_text(lcd, 245, 141, f"STEP {TEMP_STEP:g} C", 1, ACCENT, PANEL)

    lcd.Draw_Rectangle(0, OK, 25, 185, 454, 217)
    send_text(lcd, 45, 194, "ROTATE = ADJUST", 1, WHITE, BLACK)

    footer(lcd, "TURN = ADJUST  PRESS = APPLY   MAX 250 / 100 C")
    lcd.UpdateLCD()


def start_filament_action(action: str) -> None:
    """Start a filament macro without blocking the UI on a long M109-style wait."""
    command = {
        "LOAD FILAMENT": "LOAD_FILAMENT",
        "UNLOAD FILAMENT": "UNLOAD_FILAMENT",
    }.get(action)
    if command is None:
        return

    def worker() -> None:
        try:
            # The existing Klipper macros own the actual heating/motion
            # sequence and expose filament_state for the status worker.
            run_gcode(command, timeout=2.0)
            print(f"Filament command accepted: {command}", flush=True)
        except Exception as exc:
            # A macro may legitimately keep the HTTP request open while it
            # waits for M109. A timeout therefore does NOT mean the macro
            # failed, so leave its state alone in that case.
            #
            # A HTTP 400 is different: Klipper has rejected the submitted
            # G-code (for example a travel-limit violation). The macro can
            # already have changed filament_state before the bad move is
            # encountered, leaving the UI permanently on the filament page.
            # Clear that stale state only for a definite HTTP 400 rejection.
            print(f"Filament command transport ended: {exc}", flush=True)

            if isinstance(exc, requests.exceptions.HTTPError):
                response = getattr(exc, "response", None)
                if response is not None and response.status_code == 400:
                    reset_macro = (
                        f'SET_GCODE_VARIABLE MACRO={command} '
                        'VARIABLE=filament_state VALUE=\'"idle"\''
                    )
                    try:
                        api_post(
                            "/printer/gcode/script",
                            {"script": reset_macro},
                            timeout=2.0,
                        )
                        print(
                            f"Filament command rejected by Klipper; "
                            f"{command} state reset to idle",
                            flush=True,
                        )
                    except Exception as reset_exc:
                        print(
                            f"Filament state reset failed: {reset_exc}",
                            flush=True,
                        )

    threading.Thread(target=worker, name=f"filament-{command.lower()}", daemon=True).start()


def prepare_action_locked(status: dict[str, Any], filament_state: str = "idle") -> tuple[bool, str]:
    """Block preparation actions that could conflict with an active print/action."""
    if process_display_state in ("heating", "cooling", "homing", "running"):
        return True, process_display_name.upper() or "BUSY"

    if filament_state != "idle":
        return True, "FILAMENT"

    print_state = str(status.get("print_stats", {}).get("state", "standby")).lower()
    if print_state in ("printing", "paused"):
        return True, print_state.upper()
    return False, "READY"


def perform_prepare_action(action: str) -> None:
    commands = {
        "PLA PREHEAT": "PREHEAT_PLA",
        "PETG PREHEAT": "PREHEAT_PETG",
        "COOLDOWN": "COOLDOWN",
        "HOME ALL": "HOME_ALL",
        "MOTORS OFF": "MOTORS_OFF",
    }
    if action in ("LOAD FILAMENT", "UNLOAD FILAMENT"):
        start_filament_action(action)
        return
    command = commands.get(action)
    if command is None:
        return

    # Prepare macros can legitimately contain M109/M190-style waits.
    # Moonraker keeps the HTTP request open while Klipper executes those
    # waits, so a short client timeout is not proof that the command failed.
    # Treat a transport timeout as "accepted / let the status worker decide"
    # rather than allowing the UI action path to report a failure.
    try:
        run_gcode(command, timeout=2.0)
        print(f"Prepare command accepted: {command}", flush=True)
    except requests.exceptions.Timeout:
        print(
            f"Prepare command response timed out: {command}; "
            "Klipper may still be executing it.",
            flush=True,
        )
        if status_refresh_requested is not None:
            status_refresh_requested.set()
    except requests.exceptions.RequestException as exc:
        print(f"Prepare command transport failed: {command}: {exc}", flush=True)
        if status_refresh_requested is not None:
            status_refresh_requested.set()
    except Exception as exc:
        print(f"Prepare command failed: {command}: {exc}", flush=True)


def start_prepare_action(action: str) -> None:
    """Run a prepare macro outside the UI loop so long G-code waits never freeze navigation."""
    threading.Thread(
        target=perform_prepare_action,
        args=(action,),
        name=f"prepare-{action.lower().replace(' ', '-')}",
        daemon=True,
    ).start()


def get_wifi_status() -> dict[str, Any]:
    """Return a small snapshot of the Pi's Wi-Fi connection."""
    result = {
        "connected": False,
        "ssid": "",
        "signal": 0,
        "ip": "",
    }

    try:
        state_proc = subprocess.run(
            [
                "nmcli", "-t",
                "-f", "GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS",
                "device", "show", "wlan0",
            ],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )

        if state_proc.returncode == 0:
            fields = {}
            for line in state_proc.stdout.splitlines():
                if ":" in line:
                    key, value = line.split(":", 1)
                    fields[key] = value

            result["connected"] = fields.get(
                "GENERAL.STATE", ""
            ).startswith("100")

            ip = fields.get("IP4.ADDRESS[1]", "")
            result["ip"] = ip.split("/", 1)[0] if ip else ""

        wifi_proc = subprocess.run(
            [
                "nmcli", "-t",
                "-f", "IN-USE,SSID,SIGNAL",
                "device", "wifi", "list",
            ],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )

        if wifi_proc.returncode == 0:
            for line in wifi_proc.stdout.splitlines():
                parts = line.split(":", 2)
                if len(parts) != 3:
                    continue
                in_use, ssid, signal = parts

                if in_use.strip() == "*":
                    result["ssid"] = ssid
                    try:
                        result["signal"] = int(signal)
                    except ValueError:
                        result["signal"] = 0
                    break

    except (OSError, subprocess.SubprocessError):
        pass

    return result


def main() -> None:
    global encoder_state, encoder_detents
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PIN_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PIN_ENT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    GPIO.setup(BUZZER_PIN, GPIO.OUT, initial=GPIO.LOW)


    GPIO.add_event_detect(PIN_A, GPIO.BOTH, callback=encoder_edge)
    GPIO.add_event_detect(PIN_B, GPIO.BOTH, callback=encoder_edge)

    lcd = T5UIC1_LCD(PORT)

    settings = load_settings()
    brightness = int(settings["brightness"])
    idle_timeout = int(settings["timeout"])

    # Always wake and settle the DWIN panel when the bridge process starts/restarts.
    #screen_wake(lcd, brightness)
    dwin_wake_and_settle(lcd, brightness)
    # Allow the DWIN panel to settle after handshake/wake before the
    # first framebuffer operation.  This is particularly important when
    # launched automatically by systemd during boot.
    #time.sleep(1.0)

    lcd.Frame_SetDir(0)
    time.sleep(0.05)

    page = "main"
    selected = 0
    wifi_selected = 0
    files: list[str] = []
    last_ent = GPIO.input(PIN_ENT)
    last_button_ms = 0.0
    redraw = True
    redraw_reason = "startup"
    last_refresh = 0.0
    last_status_signature: tuple[Any, ...] | None = None

    # V1.8c diagnostics: measure encoder-to-redraw latency and the time
    # spent actually drawing/updating the DWIN. These are console-only and
    # deliberately do not alter navigation or printer behaviour.
    diag_events = 0
    diag_latency_total = 0.0
    diag_latency_max = 0.0
    diag_raw_changes = 0
    diag_invalid_transitions = 0
    diag_detents = 0
    diag_pending_events: deque[float] = deque()
    diag_draws = 0
    diag_draw_total = 0.0
    diag_draw_max = 0.0
    diag_status_total = 0.0
    diag_status_max = 0.0
    diag_last_report = time.monotonic()
    diag_logging_enabled = bool(settings.get("logging", True))
    diag_log_path = "/tmp/aquila_dwin_diag.log" if diag_logging_enabled else None
    if diag_log_path:
        try:
            with open(diag_log_path, "a", encoding="utf-8") as f:
                f.write(f"\n--- V2.0 diagnostics started {time.strftime('%Y-%m-%d %H:%M:%S')} ---\n")
        except OSError:
            diag_log_path = None
    diag_pending_event_at: float | None = None
    diag_last_loop = time.monotonic()
    diag_loop_max = 0.0
    diag_status_calls = 0
    diag_draw_by_page: dict[str, list[float]] = {}
    diag_partial_draws = 0
    diag_full_draws = 0
    diag_full_reasons = {}
    status: dict[str, Any] = {}
    info: dict[str, Any] = {}

    # V1.8h: Moonraker status polling runs outside the UI loop.  The worker
    # owns the HTTP wait; the display loop only takes a short lock to copy
    # the latest cached status.  This prevents slow HTTP responses from
    # blocking encoder polling and DWIN redraws.
    status_lock = threading.Lock()
    status_stop = threading.Event()
    status_cache: dict[str, Any] = {}
    status_cache_updated = 0.0
    status_worker_total = 0.0
    status_worker_max = 0.0
    status_worker_calls = 0
    global status_refresh_requested
    status_refresh_requested = threading.Event()

    filament_state_lock = threading.Lock()
    filament_state = "idle"
    filament_previous_state = "idle"
    filament_event = ""
    filament_event_time = 0.0
    filament_previous_page = "main"

    wifi_lock = threading.Lock()
    wifi_cache: dict[str, Any] = {
        "connected": False,
        "ssid": "",
        "signal": 0,
        "ip": "",
    }
    wifi_cache_updated = 0.0
    wifi_refresh_requested = False
    klipper_was_unavailable = False
    dwin_wake_requested = False

    def status_worker() -> None:
        global process_display_state, process_display_name
        nonlocal status_cache, status_cache_updated
        nonlocal status_worker_total, status_worker_max, status_worker_calls
        nonlocal wifi_cache, wifi_cache_updated, wifi_refresh_requested
        nonlocal klipper_was_unavailable, dwin_wake_requested
        nonlocal filament_state, filament_previous_state
        nonlocal filament_event, filament_event_time
        next_poll = 0.0
        next_wifi_poll = 0.0

        while not status_stop.is_set():
            now_worker = time.monotonic()

            with wifi_lock:
                refresh_wifi = wifi_refresh_requested
                if refresh_wifi:
                    wifi_refresh_requested = False

            if refresh_wifi or now_worker >= next_wifi_poll:
                try:
                    fetched_wifi = get_wifi_status()
                    with wifi_lock:
                        wifi_cache = fetched_wifi
                        wifi_cache_updated = time.monotonic()

                    with wifi_display_lock:
                        wifi_display.update(fetched_wifi)
                except Exception as exc:
                    print(f"Background Wi-Fi failed: {exc}")
                next_wifi_poll = time.monotonic() + 10.0

            if status_refresh_requested.is_set():
                status_refresh_requested.clear()
                next_poll = 0.0

            if now_worker < next_poll:
                wait_time = min(0.05, next_poll - now_worker)
                if next_wifi_poll > now_worker:
                    wait_time = min(wait_time, next_wifi_poll - now_worker)
                status_stop.wait(max(0.001, wait_time))
                continue

            started = time.monotonic()
            try:
                fetched = get_status()
                elapsed = time.monotonic() - started
                with status_lock:
                    status_cache = fetched
                    status_cache_updated = time.monotonic()
                    status_worker_total += elapsed
                    status_worker_max = max(status_worker_max, elapsed)
                    status_worker_calls += 1

                # Reliable cooldown completion detection.
                # COOLDOWN itself remains in "cooling" until the actual
                # heater temperatures have fallen below safe thresholds.
                cooldown_state = str(
                    fetched.get("gcode_macro COOLDOWN", {}).get(
                        "process_state", "idle"
                    )
                )

                if cooldown_state == "cooling":
                    hotend_temp = float(
                        fetched.get("extruder", {}).get(
                            "temperature", 999.0
                        )
                    )
                    bed_temp = float(
                        fetched.get("heater_bed", {}).get(
                            "temperature", 999.0
                        )
                    )

                    if (
                        hotend_temp <= COOLDOWN_HOTEND_MAX
                        and bed_temp <= COOLDOWN_BED_MAX
                    ):
                        print(
                            "Cooldown complete: "
                            f"hotend={hotend_temp:.1f} C, "
                            f"bed={bed_temp:.1f} C"
                        )

                        beep(0.35, 2000.0)
                        print("Buzzer: cooldown complete")

                        api_post(
                            "/printer/gcode/script",
                            {
                                "script": 'SET_GCODE_VARIABLE MACRO=COOLDOWN VARIABLE=process_state VALUE=\'"idle"\''
                            },
                        )

                load_state = str(
                    fetched.get("gcode_macro LOAD_FILAMENT", {}).get(
                        "filament_state", "idle"
                    )
                )
                unload_state = str(
                    fetched.get("gcode_macro UNLOAD_FILAMENT", {}).get(
                        "filament_state", "idle"
                    )
                )

                # General Prepare/process states.
                process_macros = (
                    ("PREHEAT_PLA", "preheat"),
                    ("PREHEAT_PETG", "preheat"),
                    ("COOLDOWN", "cooling"),
                    ("HOME_ALL", "homing"),
                    ("MOTORS_OFF", "motors"),
                )

                process_state = "idle"
                process_name = ""

                for macro_name, display_name in process_macros:
                    macro_state = str(
                        fetched.get(
                            f"gcode_macro {macro_name}", {}
                        ).get("process_state", "idle")
                    )
                    if macro_state != "idle":
                        process_state = macro_state
                        process_name = display_name
                        break

                active_state = "idle"
                if load_state != "idle":
                    active_state = load_state
                elif unload_state != "idle":
                    active_state = unload_state

                # Process macros have their own state namespace.
                if process_state != "idle":
                    previous_process = process_display_state
                    process_display_state = process_state
                    process_display_name = process_name

                    if process_state != previous_process:
                        print(
                            f"Process state: {previous_process} -> "
                            f"{process_state} ({process_name})"
                        )

                        if process_state in (
                            "heating",
                            "cooling",
                            "homing",
                            "running",
                        ):
                            beep(0.15, 2000.0)
                            print("Buzzer: process started")

                        elif process_state in (
                            "ready",
                            "complete",
                        ):
                            beep(0.35, 2000.0)
                            print("Buzzer: process complete")

                        # HOME_ALL remains in "complete" until the
                        # UI acknowledges the completion event.
                        # Only acknowledge after the completion state has
                        # actually been observed, never while still homing.
                        if (
                            process_name == "homing"
                            and process_state == "complete"
                        ):
                            api_post(
                                "/printer/gcode/script",
                                {
                                    "script": 'SET_GCODE_VARIABLE MACRO=HOME_ALL VARIABLE=process_state VALUE=\'"idle"\''
                                },
                            )
                else:
                    # Only return to idle after the worker has observed the
                    # process macro become idle.
                    if process_display_state != "idle":
                        print(
                            f"Process state: {process_display_state} -> idle"
                        )
                        process_display_state = "idle"
                        process_display_name = ""

                with filament_state_lock:
                    previous = filament_state
                    filament_previous_state = previous
                    filament_state = active_state

                    if active_state != previous:
                        filament_event = active_state
                        filament_event_time = time.monotonic()
                        print(
                            f"Filament state: {previous} -> {active_state}"
                        )

            except Exception as exc:
                elapsed = time.monotonic() - started
                with status_lock:
                    status_worker_total += elapsed
                    status_worker_max = max(status_worker_max, elapsed)
                    status_worker_calls += 1

                if not klipper_was_unavailable:
                    klipper_was_unavailable = True
                    print("Klipper status unavailable - watching for recovery")

                print(f"Background status failed: {exc}")

            else:
                if klipper_was_unavailable:
                    klipper_was_unavailable = False
                    dwin_wake_requested = True
                    print("Klipper status recovered - DWIN wake requested")

            next_poll = time.monotonic() + 1.0

    def get_cached_status() -> dict[str, Any]:
        with status_lock:
            return dict(status_cache)

    def beep(duration: float = 0.15, frequency: float = 2000.0) -> None:
        try:
            pwm = GPIO.PWM(BUZZER_PIN, frequency)
            pwm.start(50)
            time.sleep(duration)
            pwm.stop()
            GPIO.output(BUZZER_PIN, GPIO.LOW)
        except Exception as exc:
            print(f"Buzzer error: {exc}")

    def get_filament_state() -> tuple[str, str, float]:
        with filament_state_lock:
            return filament_state, filament_event, filament_event_time


    def get_cached_wifi() -> dict[str, Any]:
        with wifi_lock:
            return dict(wifi_cache)

    status_thread = threading.Thread(target=status_worker, name="moonraker-status", daemon=True)
    status_thread.start()

    # Non-blocking Control command queue.  Moonraker never gets to stall the
    # DWIN event loop while applying FAN or PRINT SPEED changes.
    control_action_queue = queue.Queue()

    def control_action_worker() -> None:
        while True:
            item = control_action_queue.get()
            if item is None:
                return
            kind, value, script = item
            try:
                api_post("/printer/gcode/script", {"script": script}, timeout=3.0)
                print(f"Control applied: {kind} = {value:g} %")
            except requests.exceptions.Timeout:
                # A HTTP timeout does not prove that Klipper failed to receive
                # the command. Status polling will reconcile the live value.
                print(f"Control command timed out (may have applied): {kind} = {value:g} %")
            except Exception as exc:
                print(f"Control command failed: {kind}: {exc}")

    control_action_thread = threading.Thread(
        target=control_action_worker,
        name="control-actions",
        daemon=True,
    )
    control_action_thread.start()

    # Leveling uses a dedicated motion queue so homing and bed-position moves
    # never block the DWIN event loop.  The UI remains alive while Klipper
    # performs the physical move.
    # Use the shared module-level leveling queue.
    leveling_state_lock = threading.Lock()
    leveling_operation = "idle"
    leveling_error = ""
    leveling_return_page = ""

    def set_leveling_state(state: str, error: str = "") -> None:
        nonlocal leveling_operation, leveling_error
        with leveling_state_lock:
            leveling_operation = state
            leveling_error = error

    def get_leveling_state() -> tuple[str, str]:
        with leveling_state_lock:
            return leveling_operation, leveling_error

    def leveling_action_worker() -> None:
        while True:
            item = leveling_action_queue.get()
            if item is None:
                return

            kind = item[0]
            try:
                if kind == "start_bed":
                    set_leveling_state("starting")
                    print("Leveling: homing all axes before bed-level sequence", flush=True)
                    run_gcode(
                        "G28\n"
                        f"G90\nG1 Z{LEVEL_TRAVEL_Z:g} F300\n"
                        f"G1 X{BED_LEVEL_POINTS[0][1]:g} Y{BED_LEVEL_POINTS[0][2]:g} F3000\n"
                        f"G1 Z{LEVEL_TRAVEL_Z:g} F300",
                        timeout=30.0,
                    )
                    set_leveling_state("ready")

                elif kind == "bed_point":
                    point_index, z_position = item[1], item[2]
                    set_leveling_state("moving")
                    point = BED_LEVEL_POINTS[point_index]
                    run_gcode(
                        f"G90\nG1 Z{LEVEL_TRAVEL_Z:g} F300\n"
                        f"G1 X{point[1]:g} Y{point[2]:g} F3000\n"
                        f"G1 Z{z_position:g} F300",
                        timeout=20.0,
                    )
                    set_leveling_state("ready")

                elif kind == "bed_z":
                    delta = float(item[1])
                    set_leveling_state("moving")
                    run_gcode(
                        f"G91\nG1 Z{delta:g} F120\nG90",
                        timeout=10.0,
                    )
                    set_leveling_state("ready")

                elif kind == "bed_finish":
                    set_leveling_state("finishing")
                    run_gcode(
                        f"G90\nG1 Z{LEVEL_TRAVEL_Z:g} F300",
                        timeout=10.0,
                    )
                    set_leveling_state("complete")

                elif kind == "start_z":
                    status = get_status()
                    homed_axes = get_homed_axes(status)
                    if not all(axis in homed_axes for axis in ("x", "y", "z")):
                        raise RuntimeError(
                            f"Z PAPER TEST BLOCKED - HOME ALL AXES FIRST (homed: {homed_axes.upper() or 'NONE'})"
                        )
                    set_leveling_state("starting")
                    print("Z-level: axes already homed; starting safe paper-test sequence", flush=True)
                    run_gcode(
                        "G90\n"
                        "SET_GCODE_OFFSET Z=0 MOVE=0\n"
                        f"G1 Z{LEVEL_TRAVEL_Z:g} F300\n"
                        "G1 X117.5 Y117.5 F3000",
                        timeout=20.0,
                    )
                    set_leveling_state("ready")

                elif kind == "bed_mesh":
                    # Final capability check immediately before motion. This
                    # prevents a stale UI state from ever issuing a probe/mesh
                    # command after the probe is removed or configuration changes.
                    probe_ok, mesh_ok = probe_capabilities()
                    if not probe_ok:
                        raise RuntimeError("NO PROBE CONFIGURED - MESH BLOCKED")
                    if not mesh_ok:
                        raise RuntimeError("BED_MESH NOT CONFIGURED - MESH BLOCKED")
                    set_leveling_state("starting")
                    print("Bed mesh: probe + bed_mesh detected; homing before mesh", flush=True)
                    run_gcode(
                        "G28\n"
                        "G90\n"
                        f"G1 Z{LEVEL_TRAVEL_Z:g} F300\n"
                        "BED_MESH_CALIBRATE",
                        timeout=300.0,
                    )
                    set_leveling_state("complete")

                elif kind == "z_finish":
                    status = get_status()
                    homed_axes = get_homed_axes(status)
                    if not all(axis in homed_axes for axis in ("x", "y", "z")):
                        raise RuntimeError(
                            f"Z SAVE BLOCKED - HOME ALL AXES FIRST (homed: {homed_axes.upper() or 'NONE'})"
                        )
                    set_leveling_state("finishing")
                    run_gcode(
                        f"G90\nG1 Z{LEVEL_TRAVEL_Z:g} F300\n"
                        "M400\n"
                        "Z_OFFSET_APPLY_ENDSTOP\n"
                        "SAVE_CONFIG",
                        timeout=15.0,
                    )
                    set_leveling_state("complete")

            except Exception as exc:
                set_leveling_state("error", str(exc))
                print(f"Leveling action failed: {exc}", flush=True)

    leveling_action_thread = threading.Thread(
        target=leveling_action_worker,
        name="leveling-actions",
        daemon=True,
    )
    leveling_action_thread.start()


    # Remote encoder helper. This supplements GPIO input rather than replacing it.
    virtual_stop = threading.Event()
    virtual_thread = threading.Thread(
        target=virtual_input_poll,
        args=(virtual_stop,),
        name="virtual-encoder",
        daemon=True,
    )
    virtual_thread.start()
    confirm_yes = True
    confirm_file = ""
    confirm_action = ""
    confirm_job_action = ""
    main._action_confirm_origin = "prepare"
    running = True

    sleeping = False
    last_activity = time.monotonic()

    filament_active = False
    filament_previous_page = "main"
    filament_complete_until = 0.0
    filament_last_state = "idle"

    move_axis = "X"
    move_step_index = 1  # 1.0 mm default
    jog_selected = 0
    jog_encoder_mode = True  # True = encoder directly jogs the selected axis

    temp_selected = 0
    temp_kind = "HOTEND"
    temp_value = 0.0
    filament_selected = 0
    filament_edit_selected = 0
    filament_edit_kind = "HOTEND"
    filament_edit_value = 0.0

    control_selected = 0
    partial_redraw: tuple[str, int, int] | None = None
    control_kind = "FAN"
    control_value = 0.0

    settings_selected = 0
    logging_enabled = bool(settings.get("logging", True))
    settings_kind = "BRIGHTNESS"
    settings_value = BRIGHTNESS_DEFAULT

    leveling_selected = 0
    level_confirm_action = ""
    bed_level_point = 0
    bed_level_action_selected = 0
    bed_level_z = LEVEL_TRAVEL_Z
    bed_level_step_index = 1
    z_level_action_selected = 0
    z_level_z = 0.0
    z_level_step_index = 1
    probe_is_detected = False
    bed_mesh_is_configured = False

    # Per-page selection counts. Print-status has dynamic PAUSE/RESUME + CANCEL + BACK.
    print_status_selected = 0

    print("Aquila landscape control v2.7.14 Safety Revision")
    try:
        probe_is_detected, bed_mesh_is_configured = probe_capabilities()
        print(f"Probe capability: {'DETECTED' if probe_is_detected else 'NOT FOUND'}; "
              f"bed_mesh: {'CONFIGURED' if bed_mesh_is_configured else 'NOT CONFIGURED'}", flush=True)
    except Exception as exc:
        print(f"Probe capability check failed: {exc}", flush=True)
    if diag_log_path:
        print(f"Diagnostics log: {diag_log_path}")
    print("Ctrl+C to stop")

    def draw_filament_page(
        lcd: T5UIC1_LCD,
        state: str,
        complete: bool = False,
    ) -> None:
        lcd.Frame_Clear(BLACK)
        header(lcd, "FILAMENT")

        if complete:
            title = "COMPLETE"
            subtitle = "FILAMENT OPERATION"
            colour = OK
        elif state == "heating":
            title = "HEATING..."
            subtitle = "WAITING FOR 200 C"
            colour = YELLOW
        elif state == "loading":
            title = "LOADING..."
            subtitle = "FEEDING FILAMENT"
            colour = ACCENT
        elif state == "unloading":
            title = "UNLOADING..."
            subtitle = "RETRACTING FILAMENT"
            colour = ACCENT
        else:
            title = "FILAMENT"
            subtitle = "STANDBY"
            colour = TEXT_DIM

        panel(lcd, 8, 48, 471, 137)
        send_text(lcd, 20, 64, title, 3, colour, PANEL)
        send_text(lcd, 20, 111, subtitle, 1, WHITE, PANEL)

        panel(lcd, 8, 149, 471, 190)
        if complete:
            send_text(lcd, 20, 163, "Operation finished", 1, OK, PANEL)
        elif state == "heating":
            send_text(lcd, 20, 163, "Nozzle will heat to 200 C", 1, TEXT_DIM, PANEL)
        elif state == "loading":
            send_text(lcd, 20, 163, "Please wait...", 1, TEXT_DIM, PANEL)
        elif state == "unloading":
            send_text(lcd, 20, 163, "Please wait...", 1, TEXT_DIM, PANEL)

        footer(lcd, "FILAMENT OPERATION IN PROGRESS" if not complete else "PLEASE WAIT")
        lcd.UpdateLCD()

    try:
        while running:
            with encoder_lock:
                step = encoder_detents
                encoder_detents = 0

            virtual_step, virtual_button = drain_virtual_input()
            step += virtual_step

            ent = GPIO.input(PIN_ENT)
            now = time.monotonic()
            diag_loop_max = max(diag_loop_max, now - diag_last_loop)
            diag_last_loop = now

            current_filament_state, filament_event, filament_event_time = get_filament_state()

            # Filament operations temporarily take over the display.
            if current_filament_state != filament_last_state:
                previous_filament_state = filament_last_state
                filament_last_state = current_filament_state

                # Beep when heating finishes and filament movement begins.
                if (
                    previous_filament_state == "heating"
                    and current_filament_state in ("loading", "unloading")
                ):
                    beep(0.18, 2000.0)
                    print("Buzzer: filament operation started")

                # Beep when the filament operation completes.
                elif (
                    previous_filament_state in ("loading", "unloading")
                    and current_filament_state == "idle"
                ):
                    beep(0.35, 2000.0)
                    print("Buzzer: filament operation complete")

                if current_filament_state != "idle":
                    if not filament_active:
                        filament_previous_page = page
                        filament_active = True

                        if sleeping:
                            screen_wake(lcd, brightness)
                            time.sleep(1.0)
                            sleeping = False

                        last_activity = now
                        draw_filament_page(lcd, current_filament_state)
                        redraw = False
                        partial_redraw = None
                        print(
                            f"Filament UI: {current_filament_state}"
                        )

                elif filament_active:
                    filament_complete_until = now + 2.0
                    draw_filament_page(lcd, "idle", complete=True)
                    print("Filament UI: complete")

            if filament_active and filament_complete_until:
                if now >= filament_complete_until:
                    filament_active = False
                    filament_complete_until = 0.0
                    page = filament_previous_page
                    redraw = True
                    partial_redraw = None
                    print(f"Filament UI: returning to {page}")

            if filament_active:
                last_ent = ent
                time.sleep(0.02)
                continue
            physical_button_press = last_ent == 1 and ent == 0 and (now - last_button_ms) > 0.25
            button_press = physical_button_press or virtual_button

            if step:
                diag_detents += 1
                diag_pending_events.append(now)
            if button_press:
                # Buttons are useful activity markers but are not included in
                # encoder latency statistics.
                pass

            # A Klipper recovery must wake the DWIN before either sleeping
            # path can consume the loop with a continue.
            if dwin_wake_requested:
                dwin_wake_requested = False
                try:
                    dwin_wake_and_settle(lcd, brightness)
                    sleeping = False
                    last_activity = now
                    last_refresh = 0.0
                    last_status_signature = None
                    redraw = True
                    redraw_reason = "klipper_recovery"
                    print("DWIN wake sent after Klipper recovery")
                except Exception as exc:
                    print(f"DWIN recovery wake failed: {exc}")

            # Any encoder movement or button press wakes the display.
            # The event that wakes it is deliberately consumed, so the first
            # movement does not also change a menu item or trigger an action.
            if sleeping and (step or button_press):
                screen_wake(lcd, brightness)

                # Allow the DWIN panel to settle after waking before
                # issuing framebuffer commands.
                time.sleep(1.0)

                sleeping = False
                last_activity = now
                last_refresh = 0.0
                last_status_signature = None
                redraw = True
                redraw_reason = "wake"
                print("DWIN wake")
                with encoder_lock:
                    encoder_detents = 0
                last_ent = ent
                time.sleep(0.005)
                continue

            if sleeping:
                last_ent = ent
                time.sleep(0.02)
                continue

            if step:
                last_activity = now
                if page == "main":
                    old_selected = selected
                    selected = (selected + step) % len(MAIN_ITEMS)
                    if selected != old_selected and not redraw:
                        partial_redraw = ("main", old_selected, selected)
                elif page == "prepare":
                    old_selected = selected
                    selected = (selected + step) % len(PREPARE_ITEMS)
                    if selected != old_selected and not redraw:
                        partial_redraw = ("prepare", old_selected, selected)
                elif page == "action_confirm":
                    confirm_yes = not confirm_yes
                elif page == "files":
                    selected = max(0, min(len(files), selected + step))
                elif page == "confirm":
                    confirm_yes = not confirm_yes
                elif page == "job_confirm":
                    confirm_yes = not confirm_yes
                elif page == "print_status":
                    state_for_actions = action_state(status)
                    action_count = 3 if state_for_actions in ("printing", "paused") else 1
                    print_status_selected = (print_status_selected + step) % action_count
                elif page == "move_axes":
                    selected = (selected + step) % len(MOVE_AXES)
                elif page == "jog":
                    if jog_encoder_mode:
                        # Native movement mode: every completed detent becomes
                        # a bounded jog of the already-selected axis.
                        locked, reason = motion_action_locked(status)
                        if locked:
                            print(f"Encoder jog blocked: printer busy ({reason}).", flush=True)
                        elif not axis_is_homed(status, move_axis):
                            print(
                                f"Encoder jog blocked: {move_axis} axis is not homed.",
                                flush=True,
                            )
                        else:
                            distance = step * MOVE_STEPS[move_step_index]
                            try:
                                actual_distance = jog_axis(
                                    move_axis, distance, status
                                )
                                if actual_distance:
                                    print(
                                        f"ENCODER JOG -> {move_axis} "
                                        f"{actual_distance:+g} mm",
                                        flush=True,
                                    )
                                    # Request a fresh status sample so the
                                    # position display follows the physical jog.
                                    status_refresh_requested.set()
                                    last_refresh = 0.0
                            except Exception as exc:
                                print(
                                    f"Encoder jog failed: {exc}",
                                    flush=True,
                                )
                    else:
                        # The jog menu has five actions:
                        # - MOVE, + MOVE, STEP, HOME, BACK.
                        # Keep the selector range identical to the rendered
                        # action list so BACK (index 4) is always reachable.
                        jog_selected = (jog_selected + step) % 5
                elif page == "temperature":
                    old_temp_selected = temp_selected
                    temp_selected = (temp_selected + step) % len(TEMP_ITEMS)
                    if temp_selected != old_temp_selected and not redraw:
                        partial_redraw = ("temperature", old_temp_selected, temp_selected)
                elif page == "filament_presets":
                    filament_selected = (filament_selected + step) % (len(FILAMENT_PRESET_NAMES) + 2)
                elif page == "filament_edit":
                    filament_edit_value = max(
                        0.0,
                        min(temp_limit(filament_edit_kind), filament_edit_value + step)
                    )
                elif page == "temperature_edit":
                    temp_value = max(
                        0.0,
                        min(temp_limit(temp_kind), temp_value + (step * TEMP_STEP))
                    )
                elif page == "control":
                    old_control_selected = control_selected
                    control_selected = (control_selected + step) % len(CONTROL_ITEMS)
                    if control_selected != old_control_selected and not redraw:
                        partial_redraw = ("control", old_control_selected, control_selected)
                elif page == "control_edit":
                    if control_kind == "FAN":
                        control_value = max(0.0, min(100.0, control_value + step * FAN_STEP))
                    else:
                        control_value = max(
                            SPEED_MIN,
                            min(SPEED_MAX, control_value + step * SPEED_STEP)
                        )
                elif page == "settings":
                    old_settings_selected = settings_selected
                    settings_selected = (settings_selected + step) % len(SETTINGS_ITEMS)
                    if settings_selected != old_settings_selected and not redraw:
                        partial_redraw = ("settings", old_settings_selected, settings_selected)
                elif page == "wifi":
                    wifi_selected = (wifi_selected + step) % len(WIFI_ACTIONS)
                elif page == "leveling":
                    old_leveling_selected = leveling_selected
                    leveling_selected = (leveling_selected + step) % len(LEVEL_ITEMS)
                    if leveling_selected != old_leveling_selected and not redraw:
                        partial_redraw = ("leveling", old_leveling_selected, leveling_selected)
                elif page == "level_confirm":
                    confirm_yes = not confirm_yes
                elif page == "bed_level":
                    bed_level_action_selected = (bed_level_action_selected + step) % len(BED_LEVEL_ACTIONS)
                elif page == "z_level":
                    z_level_action_selected = (z_level_action_selected + step) % len(Z_LEVEL_ACTIONS)
                elif page == "settings_edit":
                    if settings_kind == "BRIGHTNESS":
                        idx = BRIGHTNESS_LEVELS.index(int(settings_value))
                        idx = (idx + step) % len(BRIGHTNESS_LEVELS)
                        settings_value = BRIGHTNESS_LEVELS[idx]
                    else:
                        idx = TIMEOUT_LEVELS.index(int(settings_value))
                        idx = (idx + step) % len(TIMEOUT_LEVELS)
                        settings_value = TIMEOUT_LEVELS[idx]

                # Most pages need a complete repaint. CONTROL is the first
                # partial-redraw experiment and only needs its selection cards.
                redraw = True
                redraw_reason = "encoder"


            if button_press:
                last_activity = now
                last_button_ms = now

                if page == "main":
                    item = MAIN_ITEMS[selected]
                    if item == "PRINT":
                        try:
                            files = get_files()
                        except Exception as exc:
                            set_feedback("File list unavailable", "error")
                            print(f"File list failed: {exc}")
                            files = []
                        selected = 0
                        page = "files"
                    elif item == "PREPARE":
                        selected = 0
                        page = "prepare"
                    elif item == "MOVE":
                        selected = 0
                        page = "move_axes"
                    elif item == "TEMPERATURE":
                        temp_selected = 0
                        page = "temperature"
                    elif item == "CONTROL":
                        control_selected = 0
                        page = "control"
                    elif item == "LEVELING":
                        leveling_selected = 0
                        page = "leveling"
                    elif item == "SETTINGS":
                        settings_selected = 0
                        page = "settings"
                    elif item == "STATUS":
                        status = get_cached_status()
                        print_status_selected = 0
                        page = "print_status"
                    elif item == "INFO":
                        try:
                            info = get_info()
                        except Exception as exc:
                            set_feedback("Info unavailable", "error")
                            print(f"Info failed: {exc}")
                            info = {"software": "error", "machine": [0, 0, 0]}
                        page = "info"
                    elif item == "PRINT DOCTOR":
                        try:
                            probe_is_detected, bed_mesh_is_configured = probe_capabilities()
                        except Exception as exc:
                            print(f"Print Doctor capability check failed: {exc}", flush=True)
                            probe_is_detected = False
                            bed_mesh_is_configured = False
                        page = "print_doctor"

                elif page == "files":
                    if selected == 0:
                        page = "main"
                        selected = 0
                    elif files:
                        confirm_file = files[selected - 1]
                        confirm_yes = True
                        page = "confirm"

                elif page == "confirm":
                    if confirm_yes:
                        try:
                            start_print(confirm_file)
                            set_feedback("Print started", "ok")
                            print(f"Started: {confirm_file}")
                            confirm_file = ""
                        except Exception as exc:
                            set_feedback("Print start failed", "error")
                            print(f"Print start failed: {exc}")
                    print_status_selected = 0
                    page = "print_status"

                elif page == "prepare":
                    if selected == 0:
                        page = "main"
                        selected = 0
                    else:
                        candidate = PREPARE_ITEMS[selected]
                        locked, reason = prepare_action_locked(status, get_filament_state()[0])
                        if locked:
                            set_feedback(f"Blocked: printer busy ({reason})", "warn")
                            print(f"Prepare action blocked: printer busy ({reason}).", flush=True)
                        else:
                            confirm_action = candidate
                            confirm_yes = True
                            page = "action_confirm"

                elif page == "action_confirm":
                    action_origin = getattr(main, "_action_confirm_origin", "prepare")
                    if confirm_yes:
                        locked, reason = prepare_action_locked(status, get_filament_state()[0])
                        if locked:
                            set_feedback(f"Blocked: printer busy ({reason})", "warn")
                            print(f"Action blocked: printer busy ({reason}).", flush=True)
                        else:
                            start_prepare_action(confirm_action)
                            set_feedback(f"Started: {confirm_action}", "ok")
                            print(f"Action started: {confirm_action}", flush=True)

                    if action_origin == "move":
                        page = "move_axes"
                        selected = MOVE_AXES.index(confirm_action)
                    else:
                        page = "prepare"
                        selected = 0
                    main._action_confirm_origin = "prepare"

                elif page == "print_status":
                    state = action_state(status)
                    if state == "paused":
                        actions = ["RESUME", "CANCEL", "BACK"]
                    elif state == "printing":
                        actions = ["PAUSE", "CANCEL", "BACK"]
                    else:
                        actions = ["BACK"]

                    print_status_selected %= len(actions)
                    action = actions[print_status_selected]
                    if action == "BACK":
                        page = "main"
                        selected = 0
                    else:
                        confirm_job_action = action
                        confirm_yes = False if action == "CANCEL" else True
                        page = "job_confirm"

                elif page == "job_confirm":
                    if confirm_yes:
                        try:
                            if confirm_job_action == "PAUSE":
                                pause_print()
                            elif confirm_job_action == "RESUME":
                                resume_print()
                            elif confirm_job_action == "CANCEL":
                                cancel_print()
                            set_feedback(f"{confirm_job_action} sent", "ok")
                            print(f"Print action: {confirm_job_action}")
                        except Exception as exc:
                            set_feedback("Print action failed", "error")
                            print(f"Print action failed: {exc}")
                    page = "print_status"
                    print_status_selected = 0

                elif page == "temperature":
                    item = TEMP_ITEMS[temp_selected]
                    if item == "BACK":
                        page = "main"
                        selected = MAIN_ITEMS.index("TEMPERATURE")
                    elif item == "FILAMENT":
                        filament_selected = 0
                        page = "filament_presets"
                    else:
                        temp_kind = item
                        try:
                            temp_value = get_current_target(status, temp_kind)
                        except Exception:
                            temp_value = 0.0
                        page = "temperature_edit"

                elif page == "filament_presets":
                    if filament_selected < len(FILAMENT_PRESET_NAMES):
                        name = FILAMENT_PRESET_NAMES[filament_selected]
                        values = settings["filament_presets"][name]
                        try:
                            set_temperature("HOTEND", float(values["hotend"]))
                            set_temperature("BED", float(values["bed"]))
                            set_feedback(f"{name} preset applied", "ok")
                            print(f"Filament preset applied: {name} HOTEND={values['hotend']:g} BED={values['bed']:g}")
                        except Exception as exc:
                            set_feedback("Preset apply failed", "error")
                            print(f"Filament preset apply failed: {exc}")
                        page = "temperature"
                        temp_selected = TEMP_ITEMS.index("FILAMENT")
                    elif filament_selected == len(FILAMENT_PRESET_NAMES):
                        page = "temperature"
                        temp_selected = TEMP_ITEMS.index("FILAMENT")
                    else:
                        filament_edit_selected = 0
                        filament_edit_kind = "HOTEND"
                        name = FILAMENT_PRESET_NAMES[filament_edit_selected]
                        filament_edit_value = float(settings["filament_presets"][name]["hotend"])
                        page = "filament_edit"
                elif page == "filament_edit":
                    name = FILAMENT_PRESET_NAMES[filament_edit_selected]
                    if filament_edit_kind == "HOTEND":
                        settings["filament_presets"][name]["hotend"] = filament_edit_value
                        filament_edit_kind = "BED"
                        filament_edit_value = float(settings["filament_presets"][name]["bed"])
                        save_settings(settings)
                        set_feedback(f"{name} hotend saved", "ok")
                        print(f"Filament preset {name}: HOTEND saved")
                    else:
                        settings["filament_presets"][name]["bed"] = filament_edit_value
                        save_settings(settings)
                        set_feedback(f"{name} bed saved", "ok")
                        print(f"Filament preset {name}: BED saved = {filament_edit_value:g} C")
                        page = "filament_presets"
                        filament_selected = filament_edit_selected
                elif page == "temperature_edit":
                    # Apply exactly the value the encoder editor is showing.
                    # The helper clamps again at the command boundary, so the
                    # safety limit cannot be bypassed by UI state corruption.
                    try:
                        set_temperature(temp_kind, temp_value)
                        set_feedback(f"{temp_kind} target {temp_value:g} C", "ok")
                        print(f"Temperature target set: {temp_kind} = {temp_value:g} C")
                    except Exception as exc:
                        set_feedback("Temperature set failed", "error")
                        print(f"Temperature set failed: {exc}")
                    page = "temperature"
                    temp_selected = TEMP_ITEMS.index(temp_kind)

                elif page == "control":
                    item = CONTROL_ITEMS[control_selected]
                    if item == "BACK":
                        page = "main"
                        selected = MAIN_ITEMS.index("CONTROL")
                    else:
                        control_kind = item
                        control_value = (
                            get_fan_percent(status)
                            if item == "FAN"
                            else get_speed_percent(status)
                        )
                        page = "control_edit"

                elif page == "control_edit":
                    try:
                        if control_kind == "FAN":
                            value = max(0.0, min(100.0, float(control_value)))
                            script = f"M106 S{value * 2.55:.0f}"
                        else:
                            value = max(SPEED_MIN, min(SPEED_MAX, float(control_value)))
                            script = f"M220 S{value:g}"
                        control_action_queue.put((control_kind, value, script))
                        set_feedback(f"{control_kind} set to {value:g}%", "ok")
                        print(f"Control queued: {control_kind} = {value:g} %")
                    except Exception as exc:
                        set_feedback(f"{control_kind} set failed", "error")
                        print(f"{control_kind} set failed: {exc}")
                    page = "control"
                    control_selected = CONTROL_ITEMS.index(control_kind)

                elif page == "leveling":
                    item = LEVEL_ITEMS[leveling_selected]
                    if item == "BACK":
                        page = "main"
                        selected = MAIN_ITEMS.index("LEVELING")
                    elif item == "MANUAL BED LEVEL":
                        level_confirm_action = "START BED LEVEL"
                        confirm_yes = True
                        page = "level_confirm"
                    elif item == "Z ENDSTOP (CONSOLE)":
                        set_feedback("Use console: Z_ENDSTOP_CALIBRATE", "warn")
                        print("Z endstop calibration left to Klipper's supported console workflow.", flush=True)
                    elif item == "PROBE SETUP":
                        probe_is_detected, bed_mesh_is_configured = probe_capabilities()
                        page = "probe_setup"
                    elif item == "BED MESH":
                        probe_is_detected, bed_mesh_is_configured = probe_capabilities()
                        if not probe_is_detected:
                            set_feedback("No probe: mesh unavailable", "warn")
                            print("Bed mesh blocked: no probe configured; manual movement remains available.", flush=True)
                            page = "bed_mesh"
                        elif not bed_mesh_is_configured:
                            set_feedback("bed_mesh not configured", "warn")
                            print("Bed mesh blocked: bed_mesh is not configured.", flush=True)
                            page = "bed_mesh"
                        else:
                            level_confirm_action = "START BED MESH"
                            confirm_yes = True
                            page = "level_confirm"

                elif page == "level_confirm":
                    if confirm_yes:
                        try:
                            level_state, level_error = get_leveling_state()
                            if level_state in ("starting", "moving", "finishing"):
                                set_feedback(f"Leveling already {level_state}", "warn")
                                print(f"Leveling start blocked: already {level_state}.", flush=True)
                                page = "leveling"
                                raise RuntimeError("LEVELING ALREADY RUNNING")
                            # The leveling worker performs G28 itself. Manual
                            # HOME ALL is therefore not required beforehand.
                            if level_confirm_action == "START BED LEVEL":
                                set_leveling_state("starting")
                                bed_level_point, bed_level_z = start_manual_bed_level()
                                bed_level_action_selected = 0
                                bed_level_step_index = 1
                                page = "bed_level"
                            elif level_confirm_action == "START Z LEVEL":
                                set_leveling_state("starting")
                                z_level_z = start_z_level()
                                z_level_action_selected = 0
                                z_level_step_index = 1
                                page = "z_level"
                            elif level_confirm_action == "START BED MESH":
                                set_leveling_state("starting")
                                queue_bed_mesh()
                                page = "bed_mesh"
                        except Exception as exc:
                            set_leveling_state("error", str(exc))
                            set_feedback("Leveling start failed", "error")
                            print(f"Leveling start failed: {exc}", flush=True)
                            page = "leveling"
                    else:
                        page = "leveling"

                elif page == "bed_level":
                    action = BED_LEVEL_ACTIONS[bed_level_action_selected]
                    level_state, _ = get_leveling_state()

                    if level_state in ("starting", "moving", "finishing"):
                        # Physical motion is in progress. Keep the UI responsive,
                        # but do not allow another command into the motion queue.
                        set_feedback(f"Leveling busy: {level_state}", "warn")
                        print(f"Bed-level action ignored while {level_state}.", flush=True)
                    elif action == "Z -":
                        bed_level_z = bed_level_z_move(
                            -LEVEL_STEPS[bed_level_step_index], bed_level_z
                        )
                    elif action == "Z +":
                        bed_level_z = bed_level_z_move(
                            LEVEL_STEPS[bed_level_step_index], bed_level_z
                        )
                    elif action == "STEP":
                        bed_level_step_index = (bed_level_step_index + 1) % len(LEVEL_STEPS)
                    elif action == "NEXT":
                        if bed_level_point < LEVEL_POINT_COUNT - 1:
                            bed_level_point += 1
                            queue_bed_level_point(bed_level_point, bed_level_z)
                        else:
                            # The fourth corner is the end of the four-screw
                            # wizard. Do not silently wrap back to corner one.
                            queue_finish_bed_level()
                    elif action == "DONE":
                        queue_finish_bed_level()

                elif page == "z_level":
                    action = Z_LEVEL_ACTIONS[z_level_action_selected]
                    level_state, _ = get_leveling_state()

                    if level_state in ("starting", "moving", "finishing"):
                        set_feedback(f"Z-level busy: {level_state}", "warn")
                        print(f"Z-level action ignored while {level_state}.", flush=True)
                    elif action == "Z -":
                        delta = -LEVEL_STEPS[z_level_step_index]
                        run_gcode(f"SET_GCODE_OFFSET Z_ADJUST={delta:g} MOVE=1", timeout=5.0)
                        z_level_z += delta
                    elif action == "Z +":
                        delta = LEVEL_STEPS[z_level_step_index]
                        run_gcode(f"SET_GCODE_OFFSET Z_ADJUST={delta:g} MOVE=1", timeout=5.0)
                        z_level_z += delta
                    elif action == "STEP":
                        z_level_step_index = (z_level_step_index + 1) % len(LEVEL_STEPS)
                    elif action == "SAVE":
                        queue_finish_z_level()
                    elif action == "BACK":
                        # Always provide an escape route, including after a
                        # failed safety check such as "HOME ALL AXES FIRST".
                        page = "leveling"
                        leveling_selected = LEVEL_ITEMS.index("Z ENDSTOP (CONSOLE)")

                elif page == "bed_mesh":
                    page = "leveling"
                    leveling_selected = LEVEL_ITEMS.index("BED MESH")

                elif page == "probe_setup":
                    page = "leveling"
                    leveling_selected = LEVEL_ITEMS.index("PROBE SETUP")

                elif page == "settings":
                    item = SETTINGS_ITEMS[settings_selected]
                    if item == "BACK":
                        page = "main"
                        selected = MAIN_ITEMS.index("SETTINGS")
                    elif item == "WI-FI":
                        wifi_selected = 0
                        page = "wifi"
                    elif item == "LOGGING":
                        logging_enabled = not logging_enabled
                        settings["logging"] = logging_enabled
                        save_settings(settings)
                        diag_logging_enabled = logging_enabled
                        diag_log_path = "/tmp/aquila_dwin_diag.log" if logging_enabled else None
                        diag_last_report = time.monotonic()
                        diag_events = 0
                        diag_latency_total = 0.0
                        diag_latency_max = 0.0
                        diag_raw_changes = 0
                        diag_invalid_transitions = 0
                        diag_detents = 0
                        diag_pending_events.clear()
                        diag_draws = 0
                        diag_draw_total = 0.0
                        diag_draw_max = 0.0
                        diag_status_total = 0.0
                        diag_status_max = 0.0
                        diag_status_calls = 0
                        diag_loop_max = 0.0
                        diag_draw_by_page.clear()
                        diag_partial_draws = 0
                        diag_full_draws = 0
                        diag_full_reasons = {}
                        if logging_enabled:
                            try:
                                with open(diag_log_path, "a", encoding="utf-8") as f:
                                    f.write(f"\n--- V2.0 diagnostics logging enabled {time.strftime('%Y-%m-%d %H:%M:%S')} ---\n")
                            except OSError:
                                diag_log_path = None
                            print("Diagnostics logging ON", flush=True)
                        else:
                            print("Diagnostics logging OFF", flush=True)
                        # J4: the logging switch is an isolated UI change.
                        # Redraw only the Settings card so the toggle flips
                        # immediately without repainting the whole page.
                        partial_redraw = ("settings", settings_selected, settings_selected)
                    else:
                        settings_kind = item
                        if item == "BRIGHTNESS":
                            settings_value = int(brightness)
                        else:
                            settings_value = int(idle_timeout)
                        page = "settings_edit"

                elif page == "settings_edit":
                    if settings_kind == "BRIGHTNESS":
                        brightness = int(settings_value)
                        settings["brightness"] = brightness
                        screen_wake(lcd, brightness)

                    else:
                        idle_timeout = int(settings_value)
                        settings["timeout"] = idle_timeout

                    save_settings(settings)
                    page = "settings"
                    settings_selected = SETTINGS_ITEMS.index(settings_kind)

                    # J5: changing a setting only repaints its own card.
                    if settings_kind == "BRIGHTNESS":
                        draw_settings_brightness_card(lcd, brightness)
                    else:
                        draw_settings_timeout_card(lcd, idle_timeout)

                    lcd.UpdateLCD()


                elif page == "wifi":
                    action = WIFI_ACTIONS[wifi_selected]

                    if action == "BACK":
                        page = "settings"
                        settings_selected = SETTINGS_ITEMS.index("WI-FI")
                    elif action == "REFRESH":
                        wifi_refresh_requested = True

                elif page == "move_axes":
                    item = MOVE_AXES[selected]
                    if item == "BACK":
                        page = "main"
                        selected = 0
                    elif item in ("HOME ALL", "MOTORS OFF"):
                        confirm_action = item
                        confirm_yes = True
                        main._action_confirm_origin = "move"
                        page = "action_confirm"
                    else:
                        move_axis = item
                        jog_selected = 0
                        jog_encoder_mode = True
                        page = "jog"

                elif page == "jog":
                    if jog_encoder_mode:
                        # Push switches to the conventional menu layer.  This
                        # preserves access to STEP and BACK without sacrificing
                        # the natural direct-jog behaviour.
                        jog_encoder_mode = False
                        jog_selected = 2  # STEP is the first useful menu target
                        print("Jog encoder mode -> MENU", flush=True)
                    else:
                        action = ["- MOVE", "+ MOVE", "STEP", "HOME", "BACK"][jog_selected]
                        if action == "BACK":
                            page = "move_axes"
                            selected = MOVE_AXES.index(move_axis)
                        elif action == "STEP":
                            move_step_index = (move_step_index + 1) % len(MOVE_STEPS)
                            jog_encoder_mode = True
                            print(
                                f"Jog step -> {MOVE_STEPS[move_step_index]:g} mm; "
                                "encoder mode restored",
                                flush=True,
                            )
                        elif action == "HOME":
                            locked, reason = motion_action_locked(status)
                            if locked:
                                print(f"HOME {move_axis} blocked: printer busy ({reason}).", flush=True)
                            elif home_axis(move_axis, status):
                                jog_encoder_mode = True
                                status_refresh_requested.set()
                                last_refresh = 0.0
                        elif action in ("- MOVE", "+ MOVE"):
                            locked, reason = motion_action_locked(status)
                            if locked:
                                print(f"Jog {move_axis} blocked: printer busy ({reason}).", flush=True)
                            elif not axis_is_homed(status, move_axis):
                                print(
                                    f"Jog blocked: {move_axis} axis is not homed.",
                                    flush=True,
                                )
                            else:
                                distance = (
                                    -MOVE_STEPS[move_step_index]
                                    if action == "- MOVE"
                                    else MOVE_STEPS[move_step_index]
                                )
                                try:
                                    actual_distance = jog_axis(
                                        move_axis, distance, status
                                    )
                                    if actual_distance:
                                        print(
                                            f"Jog {move_axis} "
                                            f"{actual_distance:+g} mm",
                                            flush=True,
                                        )
                                        status_refresh_requested.set()
                                        last_refresh = 0.0
                                    else:
                                        print(
                                            f"Jog {move_axis} ignored at travel limit.",
                                            flush=True,
                                        )
                                except Exception as exc:
                                    set_feedback(f"Jog {move_axis} failed", "error")
                                    print(f"Jog failed: {exc}", flush=True)

                elif page == "info":
                    page = "main"
                    selected = 0

                elif page == "print_doctor":
                    page = "main"
                    selected = MAIN_ITEMS.index("PRINT DOCTOR")

                redraw = True
                redraw_reason = "button"
            last_ent = ent

            # Backlight sleep is based on user inactivity, not sensor changes.
            timeout = (
                DISPLAY_TIMEOUT_PRINTING
                if action_state(status) in ("printing", "paused")
                else idle_timeout
            )
            if timeout > 0 and (now - last_activity) >= timeout:
                screen_sleep(lcd)
                sleeping = True
                print(f"DWIN sleep after {timeout:.0f}s idle")
                last_refresh = now
                time.sleep(0.02)
                continue

            if now - last_refresh >= 1.0:
                # A transient footer message needs one redraw when it expires.
                feedback_before = bool(get_feedback()[0])
                try:
                    # V1.8j: never wait for Moonraker here.  Take the latest
                    # snapshot produced by the background worker.
                    new_status = get_cached_status()

                    ext_t = float(new_status.get("extruder", {}).get("temperature", 0))
                    ext_s = float(new_status.get("extruder", {}).get("target", 0))
                    bed_t = float(new_status.get("heater_bed", {}).get("temperature", 0))
                    bed_s = float(new_status.get("heater_bed", {}).get("target", 0))
                    pos = new_status.get("toolhead", {}).get("position", [0, 0, 0, 0])
                    p = [float(v) for v in pos[:3]]
                    state_now = str(new_status.get("print_stats", {}).get("state", "standby")).lower()
                    name_now = str(new_status.get("print_stats", {}).get("filename", ""))
                    progress_now = float(new_status.get("virtual_sdcard", {}).get("progress", 0)) * 100
                    elapsed_now = float(new_status.get("print_stats", {}).get("print_duration", 0))
                    fan_now = get_fan_percent(new_status)
                    speed_now = get_speed_percent(new_status)

                    sig = (
                        f"{ext_t:.1f}",
                        f"{ext_s:.0f}",
                        f"{bed_t:.1f}",
                        f"{bed_s:.0f}",
                        tuple(f"{v:.1f}" for v in p),
                        state_now,
                        name_now,
                        f"{progress_now:.1f}",
                        f"{elapsed_now:.0f}" if page == "print_status" else "",
                        f"{fan_now:.0f}" if page == "control" else "",
                        f"{speed_now:.0f}" if page == "control" else "",
                    )

                    previous_state = action_state(status)
                    status = new_status

                    # Automatically enter print status when a job starts or becomes paused.
                    if page in ("main", "files") and state_now in ("printing", "paused"):
                        if previous_state not in ("printing", "paused"):
                            print_status_selected = 0
                            page = "print_status"
                            redraw = True
                            redraw_reason = "print_started"

                    if sig != last_status_signature:
                        last_status_signature = sig
                        if page == "main":
                            # V1.8j2: only use the partial status redraw when
                            # the screen is already fully rendered.  On startup
                            # and after wake, redraw=True must be allowed to
                            # perform the complete home-screen draw first.
                            if not redraw:
                                if not partial_redraw:
                                    partial_redraw = ("main_status", 0, 0)
                            redraw = True
                            redraw_reason = "status"
                        elif page in ("print_status", "bed_level", "z_level", "bed_mesh", "print_doctor"):
                            redraw = True
                            redraw_reason = "status"
                        elif page == "jog":
                            if not redraw:
                                partial_redraw = ("jog_status", 0, 0)
                            redraw = True
                            redraw_reason = "status"

                except Exception as exc:
                    print(f"Status cache failed: {exc}")

                feedback_after = bool(get_feedback()[0])
                if feedback_before != feedback_after:
                    redraw = True
                    redraw_reason = "feedback"

                # Leveling worker completion/error is reflected here, keeping
                # all page changes inside the main UI thread.
                level_state, level_error = get_leveling_state()
                if level_state == "complete":
                    set_leveling_state("ready")
                    if page == "bed_level":
                        page = "leveling"
                        leveling_selected = 1
                        redraw = True
                        redraw_reason = "level_done"
                    elif page == "z_level" and z_level_action_selected == Z_LEVEL_ACTIONS.index("SAVE"):
                        page = "leveling"
                        leveling_selected = LEVEL_ITEMS.index("Z ENDSTOP (CONSOLE)")
                        redraw = True
                        redraw_reason = "z_level_done"
                    elif page == "bed_mesh":
                        page = "leveling"
                        leveling_selected = LEVEL_ITEMS.index("BED MESH")
                        redraw = True
                        redraw_reason = "bed_mesh_done"

                if level_state == "error" and page in ("bed_level", "z_level", "bed_mesh"):
                    if page == "z_level":
                        # If the paper-test safety gate rejects the start,
                        # land the encoder on BACK so the user always has a
                        # visible, actionable escape route.
                        z_level_action_selected = Z_LEVEL_ACTIONS.index("BACK")
                    redraw = True
                    redraw_reason = "level_error"

                last_refresh = now

            if redraw and not sleeping:
                draw_started = time.monotonic()
                draw_page = page
                draw_mode = "full"
                if page == "main":
                    if partial_redraw and partial_redraw[0] == "main":
                        draw_mode = "partial"
                        draw_main_selection(lcd, partial_redraw[1], partial_redraw[2])
                    elif partial_redraw and partial_redraw[0] == "main_status":
                        draw_mode = "partial"
                        draw_main_status(lcd, status)
                    else:
                        draw_main(lcd, status, selected)
                elif page == "files":
                    draw_files(lcd, files, selected)
                elif page == "confirm":
                    draw_confirm(lcd, confirm_file, confirm_yes)
                elif page == "prepare":
                    if partial_redraw and partial_redraw[0] == "prepare":
                        draw_prepare_selection(lcd, partial_redraw[1], partial_redraw[2])
                    else:
                        draw_prepare(lcd, selected)
                elif page == "temperature":
                    if partial_redraw and partial_redraw[0] == "temperature":
                        draw_temperature_selection(lcd, partial_redraw[1], partial_redraw[2])
                    else:
                        draw_temperature(lcd, status, temp_selected)
                elif page == "filament_presets":
                    draw_filament_presets(lcd, settings, filament_selected)
                elif page == "filament_edit":
                    draw_filament_edit(lcd, FILAMENT_PRESET_NAMES[filament_edit_selected], filament_edit_kind, filament_edit_value)
                elif page == "temperature_edit":
                    draw_temperature_edit(lcd, temp_kind, temp_value, status)
                elif page == "control":
                    if partial_redraw and partial_redraw[0] == "control":
                        draw_mode = "partial"
                        draw_control_selection(lcd, partial_redraw[1], partial_redraw[2])
                    else:
                        draw_control(lcd, status, control_selected)
                elif page == "control_edit":
                    draw_control_edit(lcd, control_kind, control_value, status)
                elif page == "leveling":
                    if partial_redraw and partial_redraw[0] == "leveling":
                        draw_leveling_selection(lcd, partial_redraw[1], partial_redraw[2])
                    else:
                        draw_leveling(lcd, leveling_selected, probe_is_detected, bed_mesh_is_configured)
                elif page == "level_confirm":
                    draw_action_confirm(lcd, level_confirm_action, confirm_yes)
                elif page == "bed_level":
                    level_state_draw, level_error_draw = get_leveling_state()
                    draw_bed_level(
                        lcd, bed_level_point, bed_level_action_selected,
                        bed_level_z, bed_level_step_index,
                        level_state_draw, level_error_draw,
                    )
                elif page == "z_level":
                    level_state_draw, level_error_draw = get_leveling_state()
                    draw_z_level(
                        lcd, z_level_action_selected, z_level_z,
                        z_level_step_index,
                        level_state_draw, level_error_draw,
                    )
                elif page == "bed_mesh":
                    mesh_state_draw, mesh_error_draw = get_leveling_state()
                    draw_bed_mesh(lcd, mesh_state_draw, mesh_error_draw,
                                  probe_is_detected, bed_mesh_is_configured)
                elif page == "probe_setup":
                    draw_probe_setup(lcd, probe_is_detected, bed_mesh_is_configured)
                elif page == "settings":
                    if partial_redraw and partial_redraw[0] == "settings":
                        draw_settings_selection(lcd, partial_redraw[1], partial_redraw[2], logging_enabled)
                    else:
                        draw_settings(lcd, brightness, idle_timeout, settings_selected, logging_enabled)
                elif page == "wifi":
                    draw_wifi_page(lcd, get_cached_wifi(), wifi_selected)
                elif page == "settings_edit":
                    draw_settings_edit(lcd, settings_kind, settings_value)
                elif page == "move_axes":
                    draw_move_axes(lcd, selected)
                elif page == "jog":
                    if partial_redraw and partial_redraw[0] == "jog_status":
                        draw_mode = "partial"
                        draw_jog_live_overlay(lcd, move_axis, move_step_index, status)
                    else:
                        draw_jog(lcd, move_axis, move_step_index, jog_selected, status, jog_encoder_mode)
                elif page == "action_confirm":
                    draw_action_confirm(lcd, confirm_action, confirm_yes)
                elif page == "print_status":
                    draw_print_status(lcd, status, print_status_selected)
                elif page == "job_confirm":
                    lcd.Frame_Clear(BLACK)
                    header(lcd, "PRINT")
                    panel(lcd, 8, 42, 471, 122)
                    send_text(lcd, 20, 51, "CONFIRM ACTION", 1, TEXT_DIM, PANEL)
                    action_fg = DANGER if confirm_job_action == "CANCEL" else WARN
                    send_text(lcd, 20, 73, confirm_job_action, 2, action_fg, PANEL)
                    prompt = (
                        "STOP THIS PRINT?" if confirm_job_action == "CANCEL"
                        else "Pause the current print?" if confirm_job_action == "PAUSE"
                        else "Resume the current print?"
                    )
                    send_text(lcd, 20, 101, prompt, 1, WHITE, PANEL)
                    for i, label in enumerate(("YES", "NO")):
                        sel = (i == 0 and confirm_yes) or (i == 1 and not confirm_yes)
                        x = 55 + i * 220
                        bg = PANEL2 if sel else BLACK
                        border = OK if sel else GREY
                        lcd.Draw_Rectangle(1, bg, x, 140, x + 165, 198)
                        lcd.Draw_Rectangle(0, border, x, 140, x + 165, 198)
                        send_text(lcd, x + 58, 160, label, 2, WHITE, bg)
                    footer(lcd, "TURN = SELECT   PRESS = CONFIRM")
                    lcd.UpdateLCD()
                elif page == "info":
                    draw_info(lcd, info)
                elif page == "print_doctor":
                    draw_print_doctor(
                        lcd,
                        status,
                        probe_is_detected,
                        bed_mesh_is_configured,
                        not klipper_was_unavailable,
                    )

                redraw = True
                redraw_reason = "button"

                draw_elapsed = time.monotonic() - draw_started
                diag_draws += 1
                diag_draw_total += draw_elapsed
                diag_draw_max = max(diag_draw_max, draw_elapsed)
                page_stats = diag_draw_by_page.setdefault(draw_page, [])
                page_stats.append(draw_elapsed)

                if draw_mode == "partial":
                    diag_partial_draws += 1
                else:
                    diag_full_reasons[redraw_reason] = diag_full_reasons.get(redraw_reason, 0) + 1
                    diag_full_draws += 1

                redraw_finished = time.monotonic()
                while diag_pending_events:
                    event_at = diag_pending_events.popleft()
                    latency = redraw_finished - event_at
                    diag_events += 1
                    diag_latency_total += latency
                    diag_latency_max = max(diag_latency_max, latency)

                partial_redraw = None
                redraw = False
                redraw_reason = "unknown"

            last_ent = ent
            # Print a compact diagnostic report every 10 seconds. This makes
            # it possible to distinguish encoder polling lag from DWIN redraw
            # time and from Moonraker status-request stalls.
            now_diag = time.monotonic()
            if diag_logging_enabled and now_diag - diag_last_report >= 10.0:
                avg_latency = (diag_latency_total / diag_events * 1000) if diag_events else 0.0
                avg_draw = (diag_draw_total / diag_draws * 1000) if diag_draws else 0.0
                avg_status = (diag_status_total / diag_status_calls * 1000) if diag_status_calls else 0.0
                with status_lock:
                    worker_calls = status_worker_calls
                    worker_total = status_worker_total
                    worker_max = status_worker_max
                worker_avg = (worker_total / worker_calls * 1000) if worker_calls else 0.0
                page_bits = []
                for page_name, samples in sorted(diag_draw_by_page.items()):
                    page_avg = sum(samples) / len(samples) * 1000
                    page_max = max(samples) * 1000
                    page_bits.append(f"{page_name}={page_avg:.1f}/{page_max:.1f}")
                pages_text = ",".join(page_bits) if page_bits else "none"
                reasons_text = ",".join(f"{k}={v}" for k, v in sorted(diag_full_reasons.items())) if diag_full_reasons else "none"
                diag_line = (
                    f"{time.strftime('%H:%M:%S')} DIAG  detents={diag_detents} raw={diag_raw_changes} invalid={diag_invalid_transitions} "
                    f"lat={avg_latency:.1f}/{diag_latency_max*1000:.1f}ms "
                    f"draw={avg_draw:.1f}/{diag_draw_max*1000:.1f}ms "
                    f"status_ui={avg_status:.1f}/{diag_status_max*1000:.1f}ms calls={diag_status_calls} "
                    f"worker={worker_avg:.1f}/{worker_max*1000:.1f}ms calls={worker_calls} "
                    f"loopmax={diag_loop_max*1000:.1f}ms partial={diag_partial_draws} full={diag_full_draws} "
                    f"full_by_reason=[{reasons_text}] "
                    f"pages[{pages_text}]"
                )
                print(diag_line, flush=True)
                if diag_log_path:
                    try:
                        with open(diag_log_path, "a", encoding="utf-8") as f:
                            f.write(diag_line + "\n")
                    except OSError:
                        pass
                diag_events = 0
                diag_latency_total = 0.0
                diag_latency_max = 0.0
                diag_raw_changes = 0
                diag_invalid_transitions = 0
                diag_detents = 0
                diag_pending_events.clear()
                diag_draws = 0
                diag_draw_total = 0.0
                diag_draw_max = 0.0
                diag_status_total = 0.0
                diag_status_max = 0.0
                diag_status_calls = 0
                with status_lock:
                    status_worker_total = 0.0
                    status_worker_max = 0.0
                    status_worker_calls = 0
                diag_loop_max = 0.0
                diag_draw_by_page.clear()
                diag_partial_draws = 0
                diag_full_draws = 0
                diag_full_reasons = {}
                diag_last_report = now_diag

            time.sleep(0.005)

    except KeyboardInterrupt:
        virtual_stop.set()
        virtual_thread.join(timeout=0.5)
        status_stop.set()
        status_thread.join(timeout=1.5)
        try:
            run_gcode("M104 S0\nM140 S0")
            print("Ctrl+C: heaters turned off.")
        except Exception as exc:
            print(f"Cool-down on exit failed: {exc}")
    finally:
        virtual_stop.set()
        virtual_thread.join(timeout=0.5)
        GPIO.cleanup()
        print("Aquila DWIN control stopped.")


if __name__ == "__main__":
    main()
