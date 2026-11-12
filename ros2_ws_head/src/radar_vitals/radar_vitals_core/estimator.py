"""Breathing and heart rate estimation from radar phase, with no dependency on ROS.

Each second, the estimator finds the range bin with the most motion, turns its phase into chest
displacement, computes spectra of that motion, and updates a belief over possible rates.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from scipy import signal
from scipy.ndimage import gaussian_filter1d

WAVELENGTH_MM = 3e8 / 62.3e9 * 1e3  # Centre of the sampled 60.7 to 63.9 GHz sweep (sampling starts 7 us into the ramp)
BREATH_BAND_HZ = (0.1, 0.6)
HEART_BAND_HZ = (0.8, 2.5)

# IWR6843AOP virtual antenna positions in half wavelengths, in the order the firmware sends them
# (TX1 RX1-4, TX2 RX1-4, TX3 RX1-4), from TI's antGeometry0 and antGeometry1 for this board.
# The demo's compRangeBiasAndRxChanPhase already applies the board's phase rotation.
ANTENNA_AZIMUTH = np.array([-1, -1, 0, 0, -3, -3, -2, -2, -1, -1, 0, 0])
ANTENNA_ELEVATION = np.array([-1, 0, -1, 0, -3, -2, -3, -2, -3, -2, -3, -2])


def displacement_mm(x):
    """Detrended chest displacement from one bin and antenna.

    Static reflections in the same bin offset the circle that the signal traces, which distorts
    the phase, so a circle fit finds that offset and removes it first.
    """
    fit = np.c_[x.real, x.imag, np.ones(len(x))]
    c = np.linalg.lstsq(fit, x.real ** 2 + x.imag ** 2, rcond=None)[0]
    centred = x - (c[0] + 1j * c[1]) / 2
    return signal.detrend(np.unwrap(np.angle(centred))) * WAVELENGTH_MM / (4 * np.pi)


class RateBelief:
    """Discrete Bayes filter: a probability for every candidate rate.

    A Kalman filter assumes one Gaussian bump, but radar spectra often show several competing
    peaks. Keeping a whole histogram lets the belief hold those options until evidence settles them.
    """

    def __init__(self, low, high, step, drift_per_sqrt_s, evidence_weight, confidence_width):
        self.grid = np.arange(low, high + step / 2, step)
        self.step = step
        self.drift = drift_per_sqrt_s
        self.evidence_weight = evidence_weight
        self.confidence_width = confidence_width
        self.updates = 0
        self.reset()

    def reset(self):
        self.belief = np.full(len(self.grid), 1 / len(self.grid))
        self.updates = 0

    def predict(self, dt):
        """Rates drift over time, so the belief spreads out; a little leaks everywhere to allow recovery."""
        sigma = self.drift * math.sqrt(dt) / self.step
        spread = gaussian_filter1d(self.belief, sigma, mode='nearest')
        spread = 0.998 * spread + 0.002 / len(self.grid)
        self.belief = spread / spread.sum()

    def update(self, rates, power):
        """Multiply the belief by the evidence, so rates with more spectral power gain probability."""
        evidence = np.interp(self.grid, rates, power / power.mean())
        # Windows overlap heavily, so each update counts as only a small piece of new evidence
        posterior = self.belief * (evidence + 1e-3) ** self.evidence_weight
        self.belief = posterior / posterior.sum()
        self.updates += 1

    def estimate(self):
        """Return (rate, standard deviation, confidence) from the most likely region of the belief."""
        if not self.updates:
            return math.nan, math.nan, 0.0
        centre = self.grid[np.argmax(self.belief)]
        near = np.abs(self.grid - centre) <= self.confidence_width
        mass = self.belief[near].sum()
        rate = float((self.grid[near] * self.belief[near]).sum() / mass)
        std = float(math.sqrt((((self.grid[near] - rate) ** 2) * self.belief[near]).sum() / mass))
        return rate, std, float(mass)


@dataclass
class VitalsEstimate:
    warming_up: bool = True
    person_detected: bool = False
    motion_detected: bool = False
    range_m: float = math.nan
    heart_rate_bpm: float = math.nan
    heart_rate_std_bpm: float = math.nan
    heart_confidence: float = 0.0
    breathing_rate_bpm: float = math.nan
    breathing_rate_std_bpm: float = math.nan
    breathing_confidence: float = 0.0
    breathing_amplitude_mm: float = math.nan
    heart_amplitude_mm: float = math.nan


class VitalsEstimator:
    """Add one radar frame at a time with add_frame(), and call update() about once per second."""

    def __init__(self, frame_rate_hz, bin_m, window_s=30.0, min_window_s=15.0, min_range_m=0.2,
                 max_range_m=2.5, presence_ratio=8.0, bin_switch_ratio=1.3, motion_limit_mm=0.6,
                 motion_step_limit_mm=0.45, min_breath_amplitude_mm=0.3, absent_reset_s=10.0,
                 max_frame_gap=20, rate_change_s=10.0, rate_change_bpm=3.0, rate_change_power_ratio=0.5):
        self.fs = frame_rate_hz
        self.bin_m = bin_m
        self.min_frames = int(min_window_s * frame_rate_hz)
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.presence_ratio = presence_ratio
        self.bin_switch_ratio = bin_switch_ratio
        self.motion_limit_mm = motion_limit_mm
        self.motion_step_limit_mm = motion_step_limit_mm
        self.min_breath_amplitude_mm = min_breath_amplitude_mm
        self.absent_reset_s = absent_reset_s
        self.max_frame_gap = max_frame_gap
        self.rate_change_s = rate_change_s
        self.rate_change_bpm = rate_change_bpm
        self.rate_change_power_ratio = rate_change_power_ratio

        self.frames = deque(maxlen=int(window_s * frame_rate_hz))
        self.last_number = None
        self.missed_frames = 0
        self.selected_bin = None
        self.absent_s = 0.0
        self.breath_disagree_s = 0.0
        self.breathing_motion_mm = None  # Latest window of chest movement in the breathing band
        self.chest_direction = None  # (azimuth, elevation) in radians of the strongest breathing motion
        azimuth, elevation = (a.ravel() for a in np.meshgrid(
            np.radians(np.arange(-60, 61, 2)), np.radians(np.arange(-40, 41, 2)), indexing='ij'))
        self._directions = np.c_[azimuth, elevation]
        # Conjugate steering vectors; positive azimuth is the radar's right and positive elevation is up
        self._steering = np.exp(1j * np.pi * (np.outer(np.cos(elevation) * np.sin(azimuth), ANTENNA_AZIMUTH)
                                              + np.outer(np.sin(elevation), ANTENNA_ELEVATION)))
        self._bands = {band: signal.butter(4, band, btype='band', fs=frame_rate_hz, output='sos')
                       for band in (BREATH_BAND_HZ, HEART_BAND_HZ)}
        self.heart = RateBelief(48, 150, 0.5, drift_per_sqrt_s=0.5, evidence_weight=0.07,
                                confidence_width=3.0)
        self.breath = RateBelief(6, 36, 0.25, drift_per_sqrt_s=0.3, evidence_weight=0.07,
                                 confidence_width=1.5)

    def add_frame(self, number, iq):
        if self.last_number is not None:
            gap = number - self.last_number - 1
            if gap > 0:
                self.missed_frames += gap
                if gap > self.max_frame_gap:
                    self.frames.clear()
                else:
                    # A short gap is bridged by repeating the last frame, which keeps timing correct
                    self.frames.extend([self.frames[-1]] * gap if self.frames else [])
        self.last_number = number
        self.frames.append(iq)

    def _filtered(self, x, band):
        return signal.sosfiltfilt(self._bands[band], x)

    def _spectrum(self, x, band):
        """Return (rates per minute, normalised power) of a displacement signal within a band."""
        y = self._filtered(x, band)
        n = 8192
        power = np.abs(np.fft.rfft(y * np.hanning(len(y)), n)) ** 2
        rates = np.fft.rfftfreq(n, 1 / self.fs) * 60
        keep = (rates >= band[0] * 60) & (rates <= band[1] * 60)
        return rates[keep], power[keep] / power[keep].sum()

    def _select_bin(self, iq):
        """Return the range bin with the most motion, or None if no bin stands out from the scene."""
        low = max(int(self.min_range_m / self.bin_m), 0)
        high = min(int(self.max_range_m / self.bin_m) + 1, iq.shape[1])
        motion = (np.abs(iq - iq.mean(axis=0)) ** 2).mean(axis=(0, 2))[low:high]
        best = int(np.argmax(motion))
        if motion[best] < self.presence_ratio * np.median(motion):
            return None
        best += low
        # Stay on the current bin unless another one is clearly better
        current = self.selected_bin
        if current is not None and low <= current < high:
            if motion[best - low] < self.bin_switch_ratio * motion[current - low]:
                return current
        return best

    def update(self, dt=1.0):
        result = VitalsEstimate(warming_up=len(self.frames) < self.min_frames)
        self.breathing_motion_mm = None
        self.chest_direction = None
        self.heart.predict(dt)
        self.breath.predict(dt)
        if result.warming_up:
            return result

        iq = np.stack(self.frames)
        chosen = self._select_bin(iq)
        if chosen is None:
            self.absent_s += dt
            if self.absent_s >= self.absent_reset_s:
                self.heart.reset()
                self.breath.reset()
                self.selected_bin = None
            self._fill_rates(result)
            return result

        self.absent_s = 0.0
        if self.selected_bin is not None and abs(chosen - self.selected_bin) * self.bin_m > 0.3:
            # Probably a different person or place, so earlier beliefs no longer apply
            self.heart.reset()
            self.breath.reset()
        self.selected_bin = chosen
        result.person_detected = True
        result.range_m = chosen * self.bin_m

        strongest = int(np.argmax((np.abs(iq[:, chosen] - iq[:, chosen].mean(axis=0)) ** 2).mean(axis=0)))
        chest = displacement_mm(iq[:, chosen, strongest])
        recent = int(10 * self.fs)
        self.breathing_motion_mm = self._filtered(chest, BREATH_BAND_HZ)
        self.chest_direction = self._breathing_direction(iq[:, chosen])
        result.breathing_amplitude_mm = float(self.breathing_motion_mm[-recent:].std())
        heart_band = self._filtered(chest, HEART_BAND_HZ)
        result.heart_amplitude_mm = float(heart_band[-recent:].std())

        # Body movement swamps the heart band, so hold the beliefs rather than learn from it.
        # Very fast movement scrambles the phase instead, which shows up as large jumps between frames.
        latest = int(3 * self.fs)
        result.motion_detected = bool(heart_band[-latest:].std() > self.motion_limit_mm
                                      or np.diff(chest)[-latest:].std() > self.motion_step_limit_mm)
        if not result.motion_detected:
            self._update_beliefs(iq, chosen, result, dt)
        self._fill_rates(result)
        return result

    def _breathing_direction(self, x):
        """Return (azimuth, elevation) of the strongest breathing-band motion in one range bin.

        The array is small, so the answer is only good to about 15 degrees either way.
        """
        moving = signal.sosfiltfilt(self._bands[BREATH_BAND_HZ], x - x.mean(axis=0), axis=0)
        covariance = moving.T @ moving.conj() / len(moving)
        power = np.einsum('da,ac,dc->d', self._steering, covariance, self._steering.conj()).real
        azimuth, elevation = self._directions[np.argmax(power)]
        return float(azimuth), float(elevation)

    def _reset_breath_if_rate_changed(self, rates, power, dt):
        """A belief locked on an old rate takes too long to move, so restart it once the spectrum clearly disagrees."""
        centre = self.breath.grid[np.argmax(self.breath.belief)]
        disagrees = (abs(rates[np.argmax(power)] - centre) > self.rate_change_bpm
                     and np.interp(centre, rates, power) < self.rate_change_power_ratio * power.max())
        self.breath_disagree_s = self.breath_disagree_s + dt if disagrees and self.breath.updates else 0.0
        if self.breath_disagree_s >= self.rate_change_s:
            self.breath.reset()
            self.breath_disagree_s = 0.0

    def _update_beliefs(self, iq, chosen, result, dt):
        bins = range(max(chosen - 1, 0), min(chosen + 2, iq.shape[1]))
        signals = [displacement_mm(iq[:, b, a]) for b in bins for a in range(iq.shape[2])]

        if result.breathing_amplitude_mm >= self.min_breath_amplitude_mm:
            rates, power = self._mean_spectrum(signals, BREATH_BAND_HZ)
            self._reset_breath_if_rate_changed(rates, power, dt)
            self.breath.update(rates, power)

        rates, power = self._mean_spectrum(signals, HEART_BAND_HZ)
        breath_rate, _, breath_confidence = self.breath.estimate()
        if breath_confidence > 0.5:
            power = power * self._harmonic_mask(rates, breath_rate)
        self.heart.update(rates, power)

    def _mean_spectrum(self, signals, band):
        spectra = [self._spectrum(x, band) for x in signals]
        return spectra[0][0], np.mean([power for _, power in spectra], axis=0)

    @staticmethod
    def _harmonic_mask(rates, breath_rate, width=1.0, depth=0.4):
        """Weaken rates at multiples of the breathing rate, since breathing is not a pure sine."""
        mask = np.ones(len(rates))
        for k in range(2, int(rates[-1] / breath_rate) + 1):
            mask -= depth * np.exp(-0.5 * ((rates - k * breath_rate) / width) ** 2)
        return np.clip(mask, 1 - depth, 1)

    def _fill_rates(self, result):
        result.heart_rate_bpm, result.heart_rate_std_bpm, result.heart_confidence = self.heart.estimate()
        result.breathing_rate_bpm, result.breathing_rate_std_bpm, result.breathing_confidence = self.breath.estimate()
