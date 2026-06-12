"""
Acoustic drone tracker — 2x INMP441 I2S MEMS mics + GCC-PHAT TDOA.

Both mics share the Pi's I2S bus (one with L/R tied to GND = left channel,
one tied to 3.3V = right channel), so every stereo frame is
sample-synchronous. Direction is estimated from the time difference of
arrival (TDOA) between the channels via GCC-PHAT cross-correlation, and a
pan servo is steered toward the source.

Wiring (both mics):
    VDD -> 3.3V        GND -> GND
    SCK -> GPIO18 (I2S BCLK)
    WS  -> GPIO19 (I2S LRCLK)
    SD  -> GPIO20 (I2S DIN)    <- both mics' SD pins share this one line
    L/R -> GND on the LEFT mic, 3.3V on the RIGHT mic

GPIO18 belongs to I2S now, so the pan servo moved to GPIO12.

One-time setup on the Pi:
    echo "dtoverlay=googlevoicehat-soundcard" | sudo tee -a /boot/firmware/config.txt
    sudo reboot
    arecord -l                                                    # find the card
    arecord -D plughw:0 -c2 -r48000 -f S32_LE -d 3 test.wav       # sanity check
    pip install numpy sounddevice gpiozero
    python -c "import sounddevice; print(sounddevice.query_devices())"
"""
import math
import socket
import time

import numpy as np
import sounddevice as sd
from gpiozero import AngularServo

# --- geometry / physics ---
MIC_SPACING_M = 0.15       # center-to-center mic distance; keep under ~0.4 m
SPEED_OF_SOUND = 343.0     # m/s at ~20 C
ANGLE_SIGN = 1.0           # positive angle = toward the RIGHT mic; flip to
                           # -1.0 if the servo turns away from the sound

# --- capture ---
SAMPLE_RATE = 48000
BLOCK_SIZE = 4096          # ~85 ms per angle estimate
DEVICE = None              # None = default input; else e.g. 'plughw:0' or an
                           # index from sounddevice.query_devices()

# --- detection gates (tune by watching the printed values) ---
RMS_GATE = 1e-4            # ignore blocks quieter than this (full scale = 1.0)
CONFIDENCE_GATE = 3.5      # min peak-to-mean ratio of the correlation;
                           # uncorrelated noise sits ~2.2, real sources ~4-7
BAND_HZ = (100.0, 8000.0)  # prop fundamentals + harmonics; rejects DC and rumble

# --- servo ---
SERVO_PIN = 12
MAX_STEP_DEG = 6.0         # max servo travel per block (slew limit)
SMOOTHING = 0.6            # EMA weight on the previous angle estimate

# --- optional telemetry to the PC; set to None to run standalone ---
PC_ADDR = ('192.168.0.243', 12345)

MAX_TAU = MIC_SPACING_M / SPEED_OF_SOUND  # largest physically possible delay


def gcc_phat(left, right, fs, max_tau, interp=16):
    """Delay of `left` relative to `right` in seconds, plus a confidence score.

    PHAT weighting whitens the spectrum so every frequency bin votes on the
    delay with equal strength — exactly what you want for a buzzy, tonal
    source like a drone. The correlation is only searched within +/-max_tau,
    so periodic signals can't alias to an impossible delay.
    """
    n = 2 * len(left)
    nfft = 1 << (n - 1).bit_length()
    spec = np.fft.rfft(left, n=nfft) * np.conj(np.fft.rfft(right, n=nfft))
    spec /= np.abs(spec) + 1e-15  # PHAT: keep phase, discard magnitude
    freqs = np.fft.rfftfreq(nfft, 1.0 / fs)
    spec[(freqs < BAND_HZ[0]) | (freqs > BAND_HZ[1])] = 0
    cc = np.fft.irfft(spec, n=nfft * interp)  # interp gives sub-sample delays
    max_shift = int(interp * fs * max_tau)
    cc = np.concatenate((cc[-max_shift:], cc[:max_shift + 1]))
    peak = int(np.argmax(np.abs(cc)))
    confidence = float(np.abs(cc[peak]) / (np.abs(cc).mean() + 1e-15))
    tau = (peak - max_shift) / float(interp * fs)
    return tau, confidence


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


servo = AngularServo(SERVO_PIN, min_angle=-90, max_angle=90,
                     min_pulse_width=0.0005, max_pulse_width=0.0025)
servo.angle = 0
sock = connect_pc()

angle_est = 0.0
servo_angle = 0.0

print("Tracking. Printed columns: angle / tdoa(us) / rms / confidence")
try:
    with sd.InputStream(samplerate=SAMPLE_RATE, channels=2, dtype='int32',
                        blocksize=BLOCK_SIZE, device=DEVICE) as stream:
        while True:
            block, overflowed = stream.read(BLOCK_SIZE)
            if overflowed:
                print("warning: input overflow, samples dropped")

            # INMP441 delivers 24-bit samples left-justified in 32-bit words
            x = block.astype(np.float64) / 2 ** 31
            left = x[:, 0] - x[:, 0].mean()
            right = x[:, 1] - x[:, 1].mean()

            rms = math.sqrt(float(np.mean(left ** 2 + right ** 2)) / 2)
            tau, confidence = gcc_phat(left, right, SAMPLE_RATE, MAX_TAU)

            heard = rms > RMS_GATE and confidence > CONFIDENCE_GATE
            if heard:
                raw_angle = ANGLE_SIGN * math.degrees(
                    math.asin(max(-1.0, min(1.0, tau / MAX_TAU))))
                angle_est = SMOOTHING * angle_est + (1 - SMOOTHING) * raw_angle

                # slew-limit so one bad estimate can't slam the servo
                step = max(-MAX_STEP_DEG, min(MAX_STEP_DEG, angle_est - servo_angle))
                servo_angle = max(-90.0, min(90.0, servo_angle + step))
                servo.angle = servo_angle

            print(f"{'TRACK' if heard else 'idle '} "
                  f"angle={angle_est:6.1f}  tdoa={tau * 1e6:7.1f}us  "
                  f"rms={rms:.5f}  conf={confidence:5.1f}")

            if sock is not None:
                record = np.array([time.time(), angle_est, tau * 1e6,
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
    servo.angle = 0
    if sock is not None:
        sock.close()
