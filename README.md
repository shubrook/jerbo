# drone-tracker

An acoustic direction-finder: a Raspberry Pi 4 with two INMP441 I2S MEMS
microphones on a servo pan mount that turns to face the sound of a drone.

## How it works

Both INMP441 mics share the Pi's I2S bus as the left and right channels of
one stereo stream, so every sample pair is captured at the same instant.
`pi/tracker.py` cross-correlates the two channels with **GCC-PHAT** to find
the time difference of arrival (TDOA), converts it to a bearing
(`angle = asin(tdoa x speed_of_sound / mic_spacing)`), and steers the pan
servo — all on the Pi, with no network round trip in the control loop.

A drone's buzz is ideal for this: its many prop harmonics and broadband
motor noise all vote for the same delay, and with the mics ~15 cm apart the
largest physically possible delay (~0.44 ms) is far shorter than one period
of the prop fundamental, so the periodic signal can't alias to a wrong peak.

`pc/listener.py` is optional: it receives `(timestamp, angle, tdoa, rms,
confidence)` records and logs them to SQL Server for analysis. Set
`PC_ADDR = None` in the Pi script to run fully standalone.

## Hardware

- Raspberry Pi 4
- 2x INMP441 I2S MEMS microphone modules, mounted 15 cm apart
- 1x hobby servo for pan (GPIO12 — GPIO18 is taken by I2S)

### Wiring (both mics)

| INMP441 pin | Connect to |
|---|---|
| VDD | 3.3V |
| GND | GND |
| SCK | GPIO18 (I2S BCLK) |
| WS  | GPIO19 (I2S LRCLK) |
| SD  | GPIO20 (I2S DIN) — both mics share this line |
| L/R | **GND on the left mic, 3.3V on the right mic** |

## Pi setup

```bash
echo "dtoverlay=googlevoicehat-soundcard" | sudo tee -a /boot/firmware/config.txt
sudo reboot
arecord -l                                              # find the card number
arecord -D plughw:0 -c2 -r48000 -f S32_LE -d 3 test.wav # should record stereo
pip install -r pi/requirements.txt
python pi/tracker.py
```

(On older Raspberry Pi OS the config file is `/boot/config.txt`.)

## Tuning

- **Servo turns away from the sound?** Flip `ANGLE_SIGN` to `-1.0`.
- **Tracks silence / jitters?** Raise `RMS_GATE` and `CONFIDENCE_GATE`;
  watch the printed `rms`/`conf` columns with the room quiet vs. with a
  source playing to pick thresholds between the two.
- **Changed the mic spacing?** Update `MIC_SPACING_M`. Wider spacing gives
  finer angular resolution but keep it under ~0.4 m to avoid correlation
  ambiguity at the prop fundamental.
- **Sluggish or twitchy?** Adjust `SMOOTHING` (higher = steadier) and
  `MAX_STEP_DEG` (servo speed per ~85 ms block).

## Adding tilt later

The Pi has a single stereo I2S input, so two synchronized channels is the
ceiling for this wiring. For pan + tilt (4 mics), use a ReSpeaker 4-Mic
Array HAT, which provides four synchronized channels and works with the
ODAS sound-localization library; the GCC-PHAT math here extends directly
to the vertical pair.
