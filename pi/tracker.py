"""
Acoustic drone tracker — 4 synchronized mics + GCC-PHAT TDOA, pan & tilt.

Needs a capture device that delivers 4 SAMPLE-SYNCHRONOUS channels. The
Pi's own I2S bus tops out at 2 mics (one stereo frame, one data-in line),
so for 4 channels use a ReSpeaker 4-Mic Array HAT or a 4-input USB audio
interface. For two bare INMP441 mics, use tracker_2mic.py instead.

All 6 mic pairs are cross-correlated (GCC-PHAT) and fused into a 3D source
direction by least squares (see doa.py), which drives the pan and tilt
servos directly. The servos must NOT use GPIO18-21 (taken by I2S):
pan is on GPIO12, tilt on GPIO13.

First run: set IDENTIFY = True, tap each mic, and reorder MIC_POSITIONS
until the spiking channel matches the mic you tapped.
"""
import math
import socket
import time

import numpy as np
import sounddevice as sd
from gpiozero import AngularServo

from doa import DirectionFinder

# --- array geometry (x = right, y = up, z = forward; meters) ---
# Preset A: ReSpeaker 4-Mic Array HAT lying flat, mics at the corners of a
# 45.7 mm square (the values ODAS uses for this HAT). Tilt ambiguity is
# resolved upward — a flat array can't tell above from below, and drones
# are above. Verify the channel order with IDENTIFY.
MIC_POSITIONS = [
    (+0.02285, 0.0, +0.02285),  # ch 0
    (-0.02285, 0.0, +0.02285),  # ch 1
    (-0.02285, 0.0, -0.02285),  # ch 2
    (+0.02285, 0.0, -0.02285),  # ch 3
]
ARRAY_NORMAL = (0.0, 1.0, 0.0)  # assume the source is above the array

# Preset B: vertical cross on a 4-input USB interface — left/right/top/
# bottom mics 15 cm from center, facing forward; assumes the source is in
# front (the rig pans toward it anyway).
# MIC_POSITIONS = [
#     (-0.15, 0.0, 0.0),   # ch 0: left
#     (+0.15, 0.0, 0.0),   # ch 1: right
#     (0.0, +0.15, 0.0),   # ch 2: top
#     (0.0, -0.15, 0.0),   # ch 3: bottom
# ]
# ARRAY_NORMAL = (0.0, 0.0, 1.0)

PAN_SIGN = 1.0             # flip if the rig pans away from the sound
TILT_SIGN = 1.0            # flip if it tilts away

# --- capture ---
SAMPLE_RATE = 48000
BLOCK_SIZE = 4096          # ~85 ms per estimate
DEVICE = None              # None = default; else from sounddevice.query_devices()
IDENTIFY = False           # True: print per-channel levels, no tracking

# --- detection gates (tune by watching the printed values) ---
RMS_GATE = 1e-4            # ignore blocks quieter than this (full scale = 1.0)
CONFIDENCE_GATE = 10.0     # median pair confidence; uncorrelated noise sits
                           # below ~6, real sources in the tens
BAND_HZ = (100.0, 8000.0)  # prop fundamentals + harmonics

# --- track gating: rejects other sound sources once locked on ---
TRACK_GATE_S = 60e-6       # accept per-pair delays within this of the track's
                           # prediction (60 us ~ a 14 deg cone at 15 cm arms)
TRACK_HOLD_GATE = 5.0      # confidence for "weak evidence": keep the lock and
                           # hold position, but don't re-center the gate (a
                           # loud interferer's sidelobes can score this high)
TRACK_MISS_LIMIT = 120     # blocks (~10 s) without a strong hit before the
                           # track is dropped and the search goes wide again
TRACK_SMOOTHING = 0.25     # the gate center must follow raw estimates much
                           # faster than the servo, or it lags the target out
                           # of its own gate
WEAK_SMOOTHING = 0.85      # gate drift rate on weak evidence; needed to follow
                           # a moving target through a louder interferer, but
                           # past ~+12 dB it can slide onto the louder source

# --- servos ---
PAN_PIN, TILT_PIN = 12, 13
PAN_RANGE = (-90, 90)
TILT_RANGE = (-10, 45)
MAX_STEP_DEG = 6.0         # max servo travel per block (slew limit)
SMOOTHING = 0.6            # EMA weight on the previous estimate

# --- optional telemetry to the PC; set to None to run standalone ---
PC_ADDR = ('192.168.0.243', 12345)

NUM_MICS = len(MIC_POSITIONS)
finder = DirectionFinder(MIC_POSITIONS, ARRAY_NORMAL, SAMPLE_RATE,
                         band_hz=BAND_HZ)


def connect_pc():
    if PC_ADDR is None:
        return None
    try:
        sock = socket.create_connection(PC_ADDR, timeout=2)
        print(f"Telemetry connected to {PC_ADDR}")
        return sock
    except OSError as e:
        print(f"Telemetry unavailable ({e}); tracking continues without it")
        return None


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def unit_vector(pan_deg, tilt_deg):
    p, t = math.radians(pan_deg), math.radians(tilt_deg)
    return np.array([math.sin(p) * math.cos(t), math.sin(t),
                     math.cos(p) * math.cos(t)])


servo_pan = AngularServo(PAN_PIN, min_angle=PAN_RANGE[0], max_angle=PAN_RANGE[1],
                         min_pulse_width=0.0005, max_pulse_width=0.0025)
servo_tilt = AngularServo(TILT_PIN, min_angle=TILT_RANGE[0], max_angle=TILT_RANGE[1],
                          min_pulse_width=0.0005, max_pulse_width=0.0025)
servo_pan.angle = 0
servo_tilt.angle = 0
sock = connect_pc()

pan_est = tilt_est = 0.0
pan_servo = tilt_servo = 0.0
track_pan = track_tilt = 0.0  # gate center, array frame (no sign flips)
track_u = None  # array-frame unit vector of the current lock, None = searching
misses = 0

print("Tracking. Columns: pan / tilt / rms / confidence")
try:
    with sd.InputStream(samplerate=SAMPLE_RATE, channels=NUM_MICS,
                        dtype='int32', blocksize=BLOCK_SIZE,
                        device=DEVICE) as stream:
        while True:
            block, overflowed = stream.read(BLOCK_SIZE)
            if overflowed:
                print("warning: input overflow, samples dropped")

            x = block.astype(np.float64) / 2 ** 31
            channels = (x - x.mean(axis=0)).T  # (NUM_MICS, BLOCK_SIZE), DC removed

            if IDENTIFY:
                levels = np.sqrt((channels ** 2).mean(axis=1))
                print('  '.join(f"ch{i} {'#' * int(lvl * 400):<20s}"
                                for i, lvl in enumerate(levels)))
                continue

            rms = float(np.sqrt((channels ** 2).mean()))
            pan_raw, tilt_raw, confidence = finder.estimate(
                channels, expected_u=track_u, gate_s=TRACK_GATE_S)

            strong = rms > RMS_GATE and confidence > CONFIDENCE_GATE
            holding = (not strong and track_u is not None
                       and rms > RMS_GATE and confidence > TRACK_HOLD_GATE)
            if strong:
                if track_u is None:  # acquisition: snap to the detection
                    track_pan, track_tilt = pan_raw, tilt_raw
                    pan_est = PAN_SIGN * pan_raw
                    tilt_est = TILT_SIGN * tilt_raw
                else:
                    track_pan = (TRACK_SMOOTHING * track_pan
                                 + (1 - TRACK_SMOOTHING) * pan_raw)
                    track_tilt = (TRACK_SMOOTHING * track_tilt
                                  + (1 - TRACK_SMOOTHING) * tilt_raw)
                    pan_est = SMOOTHING * pan_est + (1 - SMOOTHING) * PAN_SIGN * pan_raw
                    tilt_est = SMOOTHING * tilt_est + (1 - SMOOTHING) * TILT_SIGN * tilt_raw
                track_u = unit_vector(track_pan, track_tilt)
                misses = 0

                # slew-limit so one bad estimate can't slam the servos
                pan_servo += clamp(pan_est - pan_servo, -MAX_STEP_DEG, MAX_STEP_DEG)
                tilt_servo += clamp(tilt_est - tilt_servo, -MAX_STEP_DEG, MAX_STEP_DEG)
                servo_pan.angle = clamp(pan_servo, *PAN_RANGE)
                servo_tilt.angle = clamp(tilt_servo, *TILT_RANGE)
            elif track_u is not None:
                if holding:  # let the gate creep after a moving target
                    track_pan = (WEAK_SMOOTHING * track_pan
                                 + (1 - WEAK_SMOOTHING) * pan_raw)
                    track_tilt = (WEAK_SMOOTHING * track_tilt
                                  + (1 - WEAK_SMOOTHING) * tilt_raw)
                    track_u = unit_vector(track_pan, track_tilt)
                # servo holds position on anything short of a strong hit
                misses += 1
                if misses > TRACK_MISS_LIMIT:
                    track_u = None
                    print("track lost, searching wide")

            status = 'TRACK' if strong else ('hold ' if holding else 'idle ')
            print(f"{status} "
                  f"pan={pan_est:6.1f}  tilt={tilt_est:6.1f}  "
                  f"rms={rms:.5f}  conf={confidence:5.1f}")

            if sock is not None:
                record = np.array([time.time(), pan_est, tilt_est,
                                   rms, confidence], dtype=np.float64)
                try:
                    sock.sendall(record.tobytes())
                except OSError:
                    sock.close()
                    sock = None
                    print("Telemetry link lost; tracking continues")
except KeyboardInterrupt:
    print("\nStopped")
finally:
    servo_pan.angle = 0
    servo_tilt.angle = 0
    if sock is not None:
        sock.close()
