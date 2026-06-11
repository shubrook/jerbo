import spidev
import time
from gpiozero import AngularServo
import socket
import numpy as np

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000
HOST = '192.168.0.243'  # Your desktop PC IP - change this
PORT = 12345  # Same as listener

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

SAMPLE_WINDOW = 256  # Number of samples per batch

# servo_pan = AngularServo(18, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)
# servo_tilt = AngularServo(16, min_angle=-10, max_angle=45, min_pulse_width=0.0005, max_pulse_width=0.0025)

# Weights for calibration (adjust these if mics have different sensitivities)
left_weight = 1.0  # Multiplier for left mic amplitude
right_weight = 1.0  # Multiplier for right mic amplitude
top_weight = 1.0  # Multiplier for top mic amplitude
bottom_weight = 10  # Multiplier for bottom mic amplitude (if you have a bottom mic on channel 3, else set to 0)

def read_adc(channel):
    if channel < 0 or channel > 7:
        return -1
    r = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((r[1] & 3) << 8) + r[2]
    return data

print("Calculating angles from mics with weights. Clap left/right/up/down to see movement.")
try:
    while True:

        left_val = read_adc(0)
        right_val = read_adc(1)
        top_val = read_adc(2)
        bottom_val = read_adc(3)  # Assume bottom mic on channel 3

        batch_data = []
        for ch in range(4):  # Left (0), right (1), top (2), bottom (3)
            for _ in range(SAMPLE_WINDOW):
                timestamp = time.time()
                sample = read_adc(ch)
                batch_data.extend([timestamp, sample])
                time.sleep(0.0002)  # Small delay for rate control (adjust for ~5kHz sampling)
        s.sendall(np.array(batch_data, dtype=np.float64).tobytes())
        # Amplitude (abs from bias) with weights
        # left_amp = abs(left_vol - 1.65) * left_weight
        # right_amp = abs(right_vol - 1.65) * right_weight
        # top_amp = abs(top_vol - 1.65) * top_weight
        # bottom_amp = abs(bottom_vol - 1.65) * bottom_weight

        # Pan angle from left/right diff
        # total_lr = left_amp + right_amp
        # if total_lr > 0.2:  # Threshold
        #     diff_lr = (left_amp - right_amp) / total_lr
        #     pan_angle = diff_lr * 90  # -90 left to 90 right
        #     servo_pan.angle = pan_angle
        #     print(f'Pan diff: {diff_lr:.2f}, Pan angle: {pan_angle:.2f} degrees')
        # else:
        #     servo_pan.angle = 0
        #     print('Pan angle: 0 degrees (no sound)')

        # Tilt angle from top/bottom diff (or top vs average LR if no bottom)
        # total_tb = top_amp + bottom_amp
        # if total_tb > 0.2:
        #     diff_tb = (top_amp - .5) / total_tb
        #     tilt_angle = diff_tb * 90 # -30 down to 30 up
        #     servo_tilt.angle = tilt_angle
        #     print(f'Tilt diff: {diff_tb:.2f}, Tilt angle: {tilt_angle:.2f} degrees')
        # else:
        #     servo_tilt.angle = 0
        #     print('Tilt angle: 0 degrees (no sound)')

        time.sleep(.1)
except KeyboardInterrupt:
    print("\nStopped")
finally:
    spi.close()
    s.close()
    # Servos are commented out above; re-enable these resets along with them
    # servo_pan.angle = 0
    # servo_tilt.angle = 0
