# tools/

## esp32_emulator.py

Emulates the ESP32 firmware's TCP link so the Airbag GUI can be exercised
without real hardware. It speaks the same line-based protocol as
`firmwarelink.cpp` (see the docstring at the top of the script) and streams
synthetic sensor/GPS/PWM values at a configurable rate.

Pure standard library — no pip install needed, just Python 3.

## Running the app against the emulator

1. **Start the emulator** (defaults to `127.0.0.1:8080`):

   ```
   python3 tools/esp32_emulator.py
   ```

   Options: `--host`, `--port`, `--rate` (Hz of streamed updates, default 10).
   Leave it running in its own terminal — it logs each connection and
   incoming line.

2. **Build the GUI** (from the `app/` directory), if not already built:

   ```
   cmake -B build/Desktop-Debug -S .
   cmake --build build/Desktop-Debug
   ```

   Or open the project in Qt Creator and build the `airbag_gui` target
   (kit "Desktop-Debug" matches the existing `build/Desktop-Debug` dir).

3. **Run the GUI**:

   ```
   ./build/Desktop-Debug/airbag_gui
   ```

4. **Point it at the emulator.** The app auto-connects on startup using the
   "IP"/"Port" fields, which default to `192.168.4.1:8080` (the real ESP32's
   AP address) — not the emulator. Before or after launch, set the IP field
   to `127.0.0.1` (port `8080` already matches) and click the connect/
   reconnect button. The status bar should report a successful connection,
   and the charts, gizmo, and map should start updating with the emulator's
   synthetic data.

The emulator can't be told apart from a real ESP32 by the GUI, so this is
also the fastest way to check a GUI change (parsing, charts, gizmo, map)
without flashing firmware.
