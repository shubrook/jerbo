# drone-tracker

An acoustic direction-finder: a Raspberry Pi 4 with a microphone array on a
servo pan/tilt mount that turns to face the sound of a drone.

## How it works (current version)

- **`pi/tracker.py`** runs on the Raspberry Pi. It reads 4 analog electret
  mics (MAX4466 modules) through an MCP3008 ADC over SPI, batches
  256 samples per channel, and streams the raw `(timestamp, value)` pairs
  to the desktop over TCP (port 12345). Servo control code
  (pan on GPIO 18, tilt on GPIO 16) is present but currently commented out.
- **`pc/listener.py`** runs on the desktop. One thread receives the sample
  batches and bulk-inserts them into a SQL Server table (`telemetry`);
  a second thread polls a view (`vw_clumped_readings`) for computed
  pan/tilt angles and streams them back to the Pi (port 12346).

Update the IP addresses at the top of both scripts for your network, and the
connection string in `pc/listener.py` for your SQL Server instance.

## Hardware

- Raspberry Pi 4
- MCP3008 ADC (SPI)
- 4x MAX4466 electret mic amplifier modules (left/right/top/bottom on ADC channels 0-3)
- 2x hobby servos for pan/tilt

## Known issues / planned direction

The amplitude-difference approach works for close, loud sounds (claps) but
won't localize a distant drone: at range, the level difference across a
small mic array is far smaller than mic gain mismatch. Planned rework:

- Replace the MAX4466 + MCP3008 analog chain with INMP441 I2S MEMS mics
  (sample-synchronous stereo straight into the Pi, no ADC).
- Estimate direction with TDOA via GCC-PHAT cross-correlation instead of
  amplitude comparison.
- Compute angles in-process and drive the servos directly; keep SQL Server
  for logging only, out of the control path.
- The Pi script does not yet listen on port 12346 for the angle stream that
  `pc/listener.py` sends back.
