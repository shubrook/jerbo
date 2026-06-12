"""Direction-of-arrival math: pairwise GCC-PHAT + least-squares fusion.

Pure numpy, no hardware — importable for testing.

Coordinate system: x = right, y = up, z = forward (the direction the rig
faces at servo zero). Pan is atan2(x, z), tilt is asin(y), both in degrees.
"""
import itertools
import math

import numpy as np


def gcc_phat(a, b, fs, max_tau, band_hz, interp=16, tau_window=None):
    """Delay of `a` relative to `b` in seconds (t_a - t_b), plus confidence.

    PHAT weighting whitens the spectrum so every frequency bin votes on the
    delay with equal strength — ideal for a buzzy, tonal source like a
    drone. The peak is only searched within +/-max_tau, so a periodic
    signal cannot alias to a physically impossible delay.

    `tau_window` = (center_s, half_width_s) narrows the search further,
    e.g. around the delay an existing track predicts. A second source at a
    different bearing cannot produce a peak inside that window, which is
    what rejects other propellers once a track is established.
    """
    n = 2 * len(a)
    nfft = 1 << (n - 1).bit_length()
    spec = np.fft.rfft(a, n=nfft) * np.conj(np.fft.rfft(b, n=nfft))
    spec /= np.abs(spec) + 1e-15  # PHAT: keep phase, discard magnitude
    freqs = np.fft.rfftfreq(nfft, 1.0 / fs)
    spec[(freqs < band_hz[0]) | (freqs > band_hz[1])] = 0
    cc = np.fft.irfft(spec, n=nfft * interp)  # interp gives sub-sample delays
    # Confidence compares the in-window peak against the mean over the WHOLE
    # correlation circle, so it is independent of the array's aperture
    global_mean = float(np.abs(cc).mean())
    max_shift = int(interp * fs * max_tau)
    lo, hi = -max_shift, max_shift
    if tau_window is not None:
        center, half = tau_window
        lo = max(lo, int((center - half) * fs * interp))
        hi = min(hi, int((center + half) * fs * interp))
        if lo > hi:  # prediction outside the physical range; fall back
            lo, hi = -max_shift, max_shift
    idx = np.arange(lo, hi + 1)
    window = cc[idx % len(cc)]
    peak = int(np.argmax(np.abs(window)))
    confidence = float(np.abs(window[peak])) / (global_mean + 1e-15)
    tau = idx[peak] / float(interp * fs)
    return tau, confidence


class DirectionFinder:
    """Fuses TDOAs from every mic pair into a source direction.

    For a far-field source with unit direction u, the measured delay for
    pair (i, j) is tau_ij = (r_j - r_i) . u / c. Stacking all pairs gives a
    linear system solved for u by confidence-weighted least squares.

    A planar array only observes the two in-plane components of u; the
    out-of-plane component is reconstructed as sqrt(1 - |u|^2) along
    `array_normal` (the side the source is assumed to be on — straight up
    for a flat HAT, straight ahead for a vertical cross). Pass
    array_normal=None for a non-planar array.
    """

    def __init__(self, mic_positions, array_normal, fs,
                 band_hz=(100.0, 8000.0), speed_of_sound=343.0, interp=16):
        self.positions = np.asarray(mic_positions, dtype=float)
        self.array_normal = (None if array_normal is None
                             else np.asarray(array_normal, dtype=float))
        self.fs = fs
        self.band_hz = band_hz
        self.interp = interp
        self.pairs = list(itertools.combinations(range(len(self.positions)), 2))
        baselines = np.array([self.positions[j] - self.positions[i]
                              for i, j in self.pairs])
        self.A = baselines / speed_of_sound
        self.pair_max_tau = np.linalg.norm(baselines, axis=1) / speed_of_sound

    def estimate(self, channels, expected_u=None, gate_s=60e-6):
        """channels: (num_mics, block_size) array -> (pan_deg, tilt_deg, conf).

        With `expected_u` (unit vector toward an established track), each
        pair's peak search is confined to +/-gate_s around the delay that
        direction predicts, rejecting louder sources elsewhere.
        """
        taus = np.empty(len(self.pairs))
        confs = np.empty(len(self.pairs))
        for k, (i, j) in enumerate(self.pairs):
            window = (None if expected_u is None
                      else (float(self.A[k] @ expected_u), gate_s))
            taus[k], confs[k] = gcc_phat(channels[i], channels[j], self.fs,
                                         self.pair_max_tau[k], self.band_hz,
                                         self.interp, tau_window=window)

        w = confs / (confs.sum() + 1e-15)
        u, *_ = np.linalg.lstsq(self.A * w[:, None], taus * w, rcond=None)

        norm = float(np.linalg.norm(u))
        if norm > 1.0:
            u = u / norm
        elif self.array_normal is not None:
            u = u + math.sqrt(1.0 - norm * norm) * self.array_normal
        elif norm > 0:
            u = u / norm

        pan = math.degrees(math.atan2(u[0], u[2]))
        tilt = math.degrees(math.asin(max(-1.0, min(1.0, u[1]))))
        return pan, tilt, float(np.median(confs))
