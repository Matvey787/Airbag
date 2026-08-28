#!/usr/bin/env python3
"""ESP32 firmware emulator for testing the Airbag GUI without real hardware.

Speaks the same line-based TCP protocol FirmwareLink expects (see
firmwarelink.cpp):

    GUI -> ESP:  "HI_ESP", "GET_CONFIG", "SET <key> <val>"
    ESP -> GUI:  "HI_GUI", "START_CONFIG", "SET <key> <val>"..., "END_CONFIG"

Point the app's IP/port fields at this process (default 127.0.0.1:8080)
and hit "Reconnect" - the GUI can't tell it apart from a real ESP32.

Usage:
    python3 tools/esp32_emulator.py [--host 127.0.0.1] [--port 8080] [--rate 10]
"""
import argparse
import math
import random
import socket
import threading
import time
from datetime import datetime, timezone

# Sent once in response to GET_CONFIG, mirrors firmware_vars.json's key set.
INITIAL_PARAMS = {
    "PWM_ME": 1500.0, "PWM_LE": 1500.0, "PWM_RE": 1500.0,
    "ACCEL_X": 0.0, "ACCEL_Y": 0.0, "ACCEL_Z": 9.81,
    "ANG_VEL_X": 0.0, "ANG_VEL_Y": 0.0, "ANG_VEL_Z": 0.0,
    "PITCH": 0.0, "YAW": 0.0, "ROLL": 0.0,
    "TEMP_FRONT": 25.0, "TEMP_REAR": 26.0,
    "HUM_FRONT": 45.0, "HUM_REAR": 47.0,
    "GPS_LAT": 55.751244, "GPS_LON": 37.618423, "GPS_SPD": 0.0, "GPS_SAT": 9,
    "GPS_DAY": 1, "GPS_MONTH": 1, "GPS_YEAR": 2026,
    "GPS_HOUR": 0, "GPS_MIN": 0, "GPS_SEC": 0,
    "CH_MAIN_ENGINE_VAL": 1500, "CH_MAIN_ENGINE_MIN": 1000, "CH_MAIN_ENGINE_MAX": 2000,
    "CH_FORWARD_VAL": 1500, "CH_FORWARD_MIN": 1000, "CH_FORWARD_MAX": 2000,
    "CH_ROTATE_VAL": 1500, "CH_ROTATE_MIN": 1000, "CH_ROTATE_MAX": 2000,
    "CH_ARM_VAL": 1000, "CH_ARM_MIN": 1000, "CH_ARM_MAX": 2000,
    "CH_HH_VAL": 1000, "CH_HH_MIN": 1000, "CH_HH_MAX": 2000,
}


class ClientSession:
    def __init__(self, conn, addr, rate_hz):
        self.conn = conn
        self.addr = addr
        self.rate_hz = rate_hz
        self.verified = False
        self.running = True
        self.t0 = time.monotonic()

    def send_line(self, line: str):
        try:
            self.conn.sendall((line + "\n").encode("utf-8"))
        except OSError:
            self.running = False

    def send_set(self, key, val):
        text = f"{val:.3f}" if isinstance(val, float) else str(val)
        self.send_line(f"SET {key} {text}")

    def send_config(self):
        self.send_line("START_CONFIG")
        for key, val in INITIAL_PARAMS.items():
            self.send_set(key, val)
        self.send_line("END_CONFIG")
        self.verified = True

    def reader_loop(self):
        buf = b""
        while self.running:
            try:
                chunk = self.conn.recv(4096)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.handle_line(line.decode("utf-8", "ignore").strip())
        self.running = False

    def handle_line(self, line: str):
        if not line:
            return
        print(f"[{self.addr}] <- {line}")
        if line == "HI_ESP":
            self.send_line("HI_GUI")
        elif line == "GET_CONFIG":
            threading.Thread(target=self.send_config, daemon=True).start()
        # Params the GUI pushes back (slider edits, etc.) are just logged.

    def stream_loop(self):
        while not self.verified and self.running:
            time.sleep(0.05)
        period = 1.0 / self.rate_hz
        while self.running:
            self.send_streaming_values(time.monotonic() - self.t0)
            time.sleep(period)

    def send_streaming_values(self, t):
        # PWM_ME/LE/RE stay inside the chart's 990-2010 axis range:
        # a slow throttle sweep plus a faster left/right roll-correction bias.
        throttle = 1500 + 350 * math.sin(t * 0.3)
        roll_bias = 100 * math.sin(t * 0.9)
        self.send_set("PWM_ME", throttle + random.uniform(-5, 5))
        self.send_set("PWM_LE", throttle - roll_bias + random.uniform(-5, 5))
        self.send_set("PWM_RE", throttle + roll_bias + random.uniform(-5, 5))

        self.send_set("ACCEL_X", 0.3 * math.sin(t * 1.5) + random.uniform(-0.1, 0.1))
        self.send_set("ACCEL_Y", 0.3 * math.cos(t * 1.3) + random.uniform(-0.1, 0.1))
        self.send_set("ACCEL_Z", 9.81 + random.uniform(-0.05, 0.05))

        self.send_set("ANG_VEL_X", 15 * math.sin(t * 0.8))
        self.send_set("ANG_VEL_Y", 15 * math.cos(t * 0.6))
        self.send_set("ANG_VEL_Z", 10 * math.sin(t * 0.4))

        self.send_set("PITCH", 20 * math.sin(t * 0.5))
        self.send_set("ROLL", 15 * math.sin(t * 0.7 + 1.0))
        # No wraparound (no "% 360") on purpose - a smooth swing keeps the
        # test signal free of jumps, so any jump seen in the GUI is real.
        self.send_set("YAW", 180 + 150 * math.sin(t * 0.15))

        self.send_set("TEMP_FRONT", 25 + 2 * math.sin(t * 0.05))
        self.send_set("TEMP_REAR", 26 + 2 * math.cos(t * 0.05))
        self.send_set("HUM_FRONT", 45 + 5 * math.sin(t * 0.03))
        self.send_set("HUM_REAR", 47 + 5 * math.cos(t * 0.03))

        now = datetime.now(timezone.utc)
        # MapController plots raw GPS_LAT/GPS_LON degrees at a fixed 5 px/degree
        # (mapcontroller.cpp), so a realistic few-meter GPS drift (~0.0001 deg)
        # is sub-pixel and invisible. Trace a wide, slow circle instead - not
        # geographically real, but it visibly moves the dot/route on the map.
        gps_w = 0.05
        self.send_set("GPS_LAT", 55.751244 + 40 * math.sin(t * gps_w))
        self.send_set("GPS_LON", 37.618423 + 40 * math.cos(t * gps_w))
        self.send_set("GPS_SPD", 5 + 3 * math.sin(t * 0.2))
        self.send_set("GPS_SAT", 9)
        self.send_set("GPS_DAY", now.day)
        self.send_set("GPS_MONTH", now.month)
        self.send_set("GPS_YEAR", now.year)
        self.send_set("GPS_HOUR", now.hour)
        self.send_set("GPS_MIN", now.minute)
        self.send_set("GPS_SEC", now.second)

    def run(self):
        reader = threading.Thread(target=self.reader_loop, daemon=True)
        streamer = threading.Thread(target=self.stream_loop, daemon=True)
        reader.start()
        streamer.start()
        reader.join()
        self.running = False
        streamer.join(timeout=1)
        self.conn.close()
        print(f"[{self.addr}] disconnected")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--rate", type=float, default=10.0,
        help="Hz of streamed SET updates (default: 10, matches the GUI's chart timer)")
    args = parser.parse_args()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)
    print(f"ESP32 emulator listening on {args.host}:{args.port}")

    try:
        while True:
            conn, addr = server.accept()
            print(f"[{addr}] connected")
            session = ClientSession(conn, addr, args.rate)
            threading.Thread(target=session.run, daemon=True).start()
    except KeyboardInterrupt:
        print("\nShutting down")
    finally:
        server.close()


if __name__ == "__main__":
    main()
