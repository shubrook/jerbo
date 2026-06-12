# drone-tracker

An acoustic direction-finder: a Raspberry Pi 4 with a microphone array on a
servo pan/tilt mount that turns to face the sound of a drone.

## How it works

The mics are captured as one sample-synchronous multichannel stream.
`pi/doa.py` cross-correlates every mic pair with **GCC-PHAT** to measure
the time difference of arrival (TDOA), then fuses all pairs into a 3D
source direction with a confidence-weighted least-squares solve.
`pi/tracker.py` runs that on ~85 ms blocks and steers the pan and tilt
servos directly — no network or database in the control loop.

A drone's buzz is ideal for this: its many prop harmonics and broadband
motor noise all vote for the same delay, and the correlation peak is only
searched within the physically possible delay range, so the periodic
signal can't alias to a wrong bearing.

`pc/listener.py` is optional: it receives `(timestamp, pan, tilt, rms,
confidence)` records and logs them to SQL Server for analysis. Set
`PC_ADDR = None` in the Pi script to run fully standalone.

## Two hardware configurations

### 4 mics — pan + tilt (`pi/tracker.py`)

The Pi's own I2S bus has one data-in line, and INMP441s support at most
two mics per data line, so **four synchronized channels need different
capture hardware**:

- **ReSpeaker 4-Mic Array HAT** (recommended): four mics in a 45.7 mm
  square, shows up as a 4-channel ALSA device after installing the
  seeed-voicecard driver. Lay it flat; a planar array can't tell above
  from below, so the code assumes the source is above — fine for drones.
  This geometry is the default `MIC_POSITIONS` preset.
- **4-input USB audio interface** with your own mics on a vertical cross
  (left/right/top/bottom, like the original design): use Preset B in
  `tracker.py`. Wider spacing gives finer angles — the 15 cm preset
  measured ~4x more accurate than the HAT in synthetic tests.

The servos must avoid GPIO18-21 (I2S): pan is on GPIO12, tilt on GPIO13.
On first run set `IDENTIFY = True`, tap each mic, and reorder
`MIC_POSITIONS` until the spiking channel matches the mic you tapped.

### 2x INMP441 — pan only (`pi/tracker_2mic.py`)

Two INMP441s on the Pi's I2S bus, 15 cm apart. Wiring (both mics):

| INMP441 pin | Connect to |
|---|---|
| VDD | 3.3V |
| GND | GND |
| SCK | GPIO18 (I2S BCLK) |
| WS  | GPIO19 (I2S LRCLK) |
| SD  | GPIO20 (I2S DIN) — both mics share this line |
| L/R | **GND on the left mic, 3.3V on the right mic** |

```bash
echo "dtoverlay=googlevoicehat-soundcard" | sudo tee -a /boot/firmware/config.txt
sudo reboot
arecord -D plughw:0 -c2 -r48000 -f S32_LE -d 3 test.wav   # should record stereo
```

(On older Raspberry Pi OS the config file is `/boot/config.txt`.)

## Running

```bash
pip install -r pi/requirements.txt
python -c "import sounddevice; print(sounddevice.query_devices())"  # set DEVICE
python pi/tracker.py          # or tracker_2mic.py
```

## Tuning

- **Rig turns away from the sound?** Flip `PAN_SIGN` / `TILT_SIGN`
  (`ANGLE_SIGN` in the 2-mic script).
- **Tracks silence / jitters?** Watch the printed `rms`/`conf` columns
  with the room quiet vs. a source playing, and set `RMS_GATE` /
  `CONFIDENCE_GATE` between the two. In synthetic tests real sources score
  confidence in the tens and uncorrelated noise stays under ~6.
- **Changed the geometry?** Update `MIC_POSITIONS` (meters; x = right,
  y = up, z = forward). Wider spacing gives finer angular resolution; keep
  pair distances under ~0.4 m to avoid correlation ambiguity at the prop
  fundamental.
- **Sluggish or twitchy?** Adjust `SMOOTHING` (higher = steadier) and
  `MAX_STEP_DEG` (servo speed per ~85 ms block).
