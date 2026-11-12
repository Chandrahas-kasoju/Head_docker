import numpy as np
from radar_vitals_core.estimator import RateBelief, VitalsEstimator, WAVELENGTH_MM

FS = 20.0
BIN_M = 0.046875
CHEST_BIN = 21


def scene(motion_mm, seed=0):
    """Simulated frames: a moving chest in one bin, a still wall in another, and noise everywhere."""
    rng = np.random.default_rng(seed)
    shape = (len(motion_mm), 64, 12)
    iq = (rng.normal(size=shape) + 1j * rng.normal(size=shape)) * 20
    iq[:, CHEST_BIN] += 3000 * np.exp(1j * 4 * np.pi * motion_mm[:, None] / WAVELENGTH_MM)
    iq[:, 37] += 5000
    return iq.astype(np.complex64)


def run(iq):
    estimator = VitalsEstimator(FS, BIN_M)
    estimates = []
    for i, frame in enumerate(iq):
        estimator.add_frame(i, frame)
        if (i + 1) % int(FS) == 0:
            estimates.append(estimator.update(dt=1.0))
    return estimator, estimates


def chest(seconds, breath_hz=0.25, heart_hz=1.2):
    t = np.arange(0, seconds, 1 / FS)
    return 4.0 * np.sin(2 * np.pi * breath_hz * t) + 0.15 * np.sin(2 * np.pi * heart_hz * t)


def test_recovers_known_rates_at_the_right_range():
    _, estimates = run(scene(chest(60)))
    last = estimates[-1]
    assert last.person_detected
    assert abs(last.range_m - CHEST_BIN * BIN_M) < 0.05
    assert abs(last.breathing_rate_bpm - 15.0) < 1.0
    assert abs(last.heart_rate_bpm - 72.0) < 2.0
    assert last.heart_confidence > 0.5


def test_tracks_a_changing_heart_rate():
    t = np.arange(0, 120, 1 / FS)
    heart_hz = np.where(t < 60, 1.2, 1.0)  # 72 then 60 per minute
    phase = 2 * np.pi * np.cumsum(heart_hz) / FS
    motion = 4.0 * np.sin(2 * np.pi * 0.25 * t) + 0.15 * np.sin(phase)
    _, estimates = run(scene(motion))
    assert abs(estimates[58].heart_rate_bpm - 72.0) < 3.0
    assert abs(estimates[-1].heart_rate_bpm - 60.0) < 3.0


def test_follows_a_large_change_in_breathing_rate():
    t = np.arange(0, 120, 1 / FS)
    breath_hz = np.where(t < 40, 10 / 60, 20 / 60)
    motion = 4.0 * np.sin(2 * np.pi * np.cumsum(breath_hz) / FS) + 0.15 * np.sin(2 * np.pi * 1.2 * t)
    _, estimates = run(scene(motion))
    assert abs(estimates[38].breathing_rate_bpm - 10.0) < 2.0
    assert abs(estimates[-1].breathing_rate_bpm - 20.0) < 2.0


def test_steady_breathing_keeps_its_belief():
    estimator, estimates = run(scene(chest(120)))
    assert estimator.breath_disagree_s == 0
    assert min(e.breathing_confidence for e in estimates[60:]) > 0.5


def test_exposes_breathing_motion_for_plotting():
    estimator, _ = run(scene(chest(40)))
    assert len(estimator.breathing_motion_mm) == 30 * FS
    assert 1.0 < estimator.breathing_motion_mm.std() < 5.0  # The simulated 4 mm sine has a 2.8 mm std


def test_warms_up_before_reporting():
    _, estimates = run(scene(chest(20)))
    assert estimates[0].warming_up
    assert np.isnan(estimates[0].heart_rate_bpm)
    assert not estimates[-1].warming_up


def test_empty_scene_reports_no_person():
    _, estimates = run(scene(np.zeros(int(30 * FS))))
    assert not estimates[-1].person_detected
    assert np.isnan(estimates[-1].heart_rate_bpm)


def test_large_movement_holds_the_estimate():
    t = np.arange(0, 60, 1 / FS)
    burst = (t > 50) & (t < 56)
    motion = chest(60) + burst * 2.0 * np.sin(2 * np.pi * 1.5 * t)  # A 2 mm sway, not a heartbeat
    _, estimates = run(scene(motion))
    assert not any(e.motion_detected for e in estimates[:50])
    assert any(e.motion_detected for e in estimates[50:])
    # The sway must not drag the estimate to its own 90 per minute
    assert abs(estimates[-1].heart_rate_bpm - 72.0) < 3.0


def test_fast_movement_is_flagged():
    t = np.arange(0, 60, 1 / FS)
    burst = (t > 50) & (t < 56)
    # Faster than the phase can follow between frames, so it arrives scrambled
    motion = chest(60) + burst * np.random.default_rng(1).normal(0, 3.0, len(t))
    _, estimates = run(scene(motion))
    assert any(e.motion_detected for e in estimates[50:])


def test_missed_frames_are_counted_and_bridged():
    estimator = VitalsEstimator(FS, BIN_M)
    frame = np.ones((64, 12), dtype=np.complex64)
    estimator.add_frame(0, frame)
    estimator.add_frame(4, frame)
    assert estimator.missed_frames == 3
    assert len(estimator.frames) == 5


def test_belief_holds_competing_peaks_then_settles():
    belief = RateBelief(48, 150, 0.5, drift_per_sqrt_s=0.5, evidence_weight=0.07, confidence_width=3.0)
    rates = np.arange(48, 150.5, 0.5)
    power = np.ones(len(rates))
    power[np.abs(rates - 70) < 1] = 20
    power[np.abs(rates - 100) < 1] = 20
    belief.update(rates, power)
    assert belief.estimate()[2] < 0.5  # One update is not enough to be sure

    power[np.abs(rates - 100) < 1] = 1
    for _ in range(60):
        belief.predict(1.0)
        belief.update(rates, power)
    rate, _, confidence = belief.estimate()
    assert abs(rate - 70) < 2
    assert confidence > 0.8


def test_finds_the_direction_of_the_breathing():
    from radar_vitals_core.estimator import ANTENNA_AZIMUTH, ANTENNA_ELEVATION
    azimuth, elevation = np.radians(20), np.radians(-10)
    right, up = np.cos(elevation) * np.sin(azimuth), np.sin(elevation)
    iq = scene(chest(30))
    # The chest echo now arrives from one direction, as a wave across the antennas
    iq[:, CHEST_BIN] = iq[:, CHEST_BIN, :1] * np.exp(-1j * np.pi * (ANTENNA_AZIMUTH * right + ANTENNA_ELEVATION * up))
    estimator, _ = run(iq)
    found_azimuth, found_elevation = np.degrees(estimator.chest_direction)
    assert abs(found_azimuth - 20) < 5
    assert abs(found_elevation + 10) < 5


def test_motion_points_cluster_at_the_moving_chest():
    from radar_vitals_core.estimator import ANTENNA_AZIMUTH
    from radar_vitals_core.motion_points import MotionPoints
    azimuth = np.radians(-20)
    iq = scene(chest(12))
    iq[:, CHEST_BIN] = iq[:, CHEST_BIN, :1] * np.exp(-1j * np.pi * ANTENNA_AZIMUTH * np.sin(azimuth))
    points = MotionPoints(FS, BIN_M).compute(iq)
    assert len(points) > 0
    distance = np.sqrt(points['x'] ** 2 + points['y'] ** 2 + points['z'] ** 2)
    assert np.all(np.abs(distance - CHEST_BIN * BIN_M) < 0.1)  # Nothing at the still wall in bin 37
    strongest = points[np.argmax(points['power_db'])]
    assert abs(np.degrees(np.arctan2(strongest['x'], strongest['y'])) + 20) < 5
