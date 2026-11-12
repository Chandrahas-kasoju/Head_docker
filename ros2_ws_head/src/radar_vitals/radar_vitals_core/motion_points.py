"""Fine motion points, similar to the minor motion layer of TI's 3D People Tracking demo. No dependency on ROS.

Over the last 2 s, each range bin's static reflection (its mean) is removed, so whatever changed in that
time is kept: breathing, heartbeat, small shifts, and also anything else that moves. Points therefore
appear about 2 s after the start, but they do not show that the motion is breathing. Range bins that move clearly more than the rest of
the scene are scanned in azimuth and elevation with Capon beamforming, and every local peak above the
noise becomes a point. A still, breathing person therefore gives a small cluster of points, which shifts a
little from update to update. The 12 antenna array still limits the true resolution to about 30 degrees.
"""
import numpy as np
from scipy.ndimage import maximum_filter

from .estimator import ANTENNA_AZIMUTH, ANTENNA_ELEVATION

POINT_DTYPE = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('power_db', 'f4')])


class MotionPoints:

    def __init__(self, frame_rate_hz, bin_m, window_s=2.0, min_range_m=0.2,
                 max_range_m=2.5, step_deg=2.0, azimuth_deg=60.0, elevation_deg=40.0,
                 bin_ratio=4.0, peak_ratio=4.0, max_below_strongest_db=15.0, loading=0.01, max_bins=12):
        self.bin_m = bin_m
        self.frames = int(window_s * frame_rate_hz)
        self.low = max(int(min_range_m / bin_m), 0)
        self.high = int(max_range_m / bin_m) + 1
        self.bin_ratio = bin_ratio
        self.peak_ratio = peak_ratio
        self.max_below = max_below_strongest_db
        self.loading = loading
        self.max_bins = max_bins
        self.azimuth = np.radians(np.arange(-azimuth_deg, azimuth_deg + step_deg / 2, step_deg))
        self.elevation = np.radians(np.arange(-elevation_deg, elevation_deg + step_deg / 2, step_deg))
        az, el = np.meshgrid(self.azimuth, self.elevation, indexing='ij')
        # Unit directions in radar axes: x right, y forward, z up
        self.directions = np.stack([np.cos(el) * np.sin(az), np.cos(el) * np.cos(az), np.sin(el)], axis=-1)
        phase = np.pi * (self.directions[..., 0, None] * ANTENNA_AZIMUTH + self.directions[..., 2, None] * ANTENNA_ELEVATION)
        self.steering = np.exp(-1j * phase).reshape(-1, len(ANTENNA_AZIMUTH))  # Arrival phase across the antennas

    def compute(self, iq):
        """Return POINT_DTYPE points from iq of shape (frames, range bins, antennas); uses the last window_s."""
        x = iq[-self.frames:, self.low:self.high].astype(np.complex64)
        if len(x) < self.frames:
            return np.zeros(0, dtype=POINT_DTYPE)
        moving = x - x.mean(axis=0)
        covariance = np.einsum('tba,tbc->bac', moving, moving.conj()) / len(moving)
        bin_power = np.einsum('baa->b', covariance).real / covariance.shape[1]
        noise = np.median(bin_power)
        found = []
        moving_bins = np.nonzero(bin_power > self.bin_ratio * noise)[0]
        # Scanning every bin of a busy scene takes too long, so keep only the strongest
        moving_bins = moving_bins[np.argsort(bin_power[moving_bins])[::-1][:self.max_bins]]
        for b in moving_bins:
            r = covariance[b]
            r = r + self.loading * np.trace(r).real / len(r) * np.eye(len(r))
            inverse = np.linalg.inv(r)
            capon = 1 / np.einsum('da,ac,dc->d', self.steering.conj(), inverse, self.steering).real
            capon = capon.reshape(len(self.azimuth), len(self.elevation))
            peaks = (capon == maximum_filter(capon, size=3, mode='nearest')) & (capon > self.peak_ratio * noise)
            for i, j in zip(*np.nonzero(peaks)):
                found.append(((self.low + b) * self.bin_m * self.directions[i, j], capon[i, j]))
        if not found:
            return np.zeros(0, dtype=POINT_DTYPE)
        power = np.array([p for _, p in found])
        keep = 10 * np.log10(power / power.max()) >= -self.max_below
        points = np.zeros(int(keep.sum()), dtype=POINT_DTYPE)
        xyz = np.array([v for v, _ in found])[keep]
        points['x'], points['y'], points['z'] = xyz.T
        points['power_db'] = 10 * np.log10(power[keep] / noise)
        return points
