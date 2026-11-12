# radar_vitals

ROS2 Humble package that publishes a point cloud, breathing rate and heart rate from a TI IWR6843AOP.
It runs on the out-of-box demo firmware (`xwr64xxAOP_(6843AOP)_mmw_demo.bin`), with no closed vital signs binary.

This is a research prototype, not a medical device.

**Known issue:** the heart rate is not validated and has failed once, for an unknown reason. One live seated run read about 57 per minute against a measured pulse of 77 to 80, with `heart_confidence` never above 0.49. Three later runs matched: 73.5 against 72 to 75, 74.6 against 70 to 84, and 69.6 live against mostly 68 to 72. Those ended with confidence of 0.8 to 0.9, but too few runs exist to set a cut-off.

## Run

Close any other driver using the radar ports first.

```bash
cd ~/Desktop/Phd_thesis/Phd_research/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select radar_vitals
source install/setup.bash
ros2 launch radar_vitals vitals.launch.py
```

Launch arguments: `command_port`, `data_port`, `frame_id`, `rviz:=true` and `plot:=true`.

Vital signs need about 15 s to start, and the person should sit still, facing the radar.

With `rviz:=true`, RViz opens looking out from the radar. Grey spheres are the radar's detected points. One larger sphere marks where the breathing is: at the chest distance the estimator chose, in the direction where the breathing motion is strongest. It turns from blue (chest in) to orange (chest out) with each breath, about 1 s late, and disappears when no person is detected. The view assumes the default `frame_id`.

The direction comes from the 12 antennas and is only good to about 15 degrees either way. Left and right matched a live test with a person off to each side; up and down has not been checked. The radar's own detected points almost never fall on a still, breathing chest (0 to 7% of frames in two counts), because one velocity step is about 0.6 m/s, so the marker does not use them.

Yellow and purple spheres are fine motion points, similar to the minor motion points of TI's 3D People Tracking demo. Every 0.5 s, whatever changed over the last 2 s is scanned in distance and direction, and each local peak well above the room's noise becomes a point. They appear about 2 s after the start. RViz keeps them for 2 s, so a still, breathing person shows as a small cluster. They show any small movement, not only breathing, and slow breathing can leave short gaps. They are coloured by strength, and switched off with `-p motion_points:=false` on the node. They take about 20 ms per update and do not change the vital signs. Like the breathing marker, their direction is only good to about 15 degrees.

To watch breathing live, add `plot:=true`. The top graph shows chest movement over the last 30 s, and the bottom graph shows the estimated rate. Closing the window stops the radar.

```bash
ros2 launch radar_vitals vitals.launch.py plot:=true
```

## Breathing results

These runs compare the 2-minute average with a manual count: one seated person at about 1.2 m. They are not a validation, and moment-to-moment accuracy has not been measured.

| Manual rate (per minute) | Radar (per minute) | Difference |
| --- | --- | --- |
| 9.5 | 9.61 | +0.11 |
| 20.5 | 20.82 | +0.32 |
| 32.5 | 32.36 | -0.14 |

Two other runs were excluded because of movement during settling and a missed count: 10.5 against 10.24, and 12.0 against 10.92. These rates are from the estimator before the rate-change reset was added; replaying the recordings with it gives the same values.

## Topics

| Topic | Type | Rate |
| --- | --- | --- |
| `/radar_vitals/points` | `sensor_msgs/PointCloud2` | 20 Hz |
| `/radar_vitals/vital_signs` | `radar_vitals/VitalSigns` | 1 Hz |
| `/radar_vitals/breathing_points` | `sensor_msgs/PointCloud2` | 20 Hz; one point where the breathing is, with field `breathing_mm`, or none when no person is detected |
| `/radar_vitals/motion_points` | `sensor_msgs/PointCloud2` | 2 Hz; fine motion points, with field `power_db` (above the scene's noise) |
| `/radar_vitals/breathing_motion_mm` | `std_msgs/Float32MultiArray` | 1 Hz; latest window of chest movement in the breathing band, at 20 samples per second |

Point fields: `x`, `y`, `z` (m; x forward, y left, z up), `velocity` (m/s), `intensity` (SNR in dB), `noise_db`.

Use `heart_confidence`, `breathing_confidence` and `motion_detected` before trusting a rate. See [msg/VitalSigns.msg](msg/VitalSigns.msg).

## Limits

- Maximum range is about 3 m, because the config trades range for enough serial bandwidth to send phase data.
- Clutter removal is off, because it would remove the chest signal. Static objects therefore appear in the point cloud.
- Only one person is tracked: the range bin with the most motion, between 0.2 m and 2.5 m.
- Range bin selection uses the last 30 s of motion, so after someone moves into place it can stay on the wrong bin for up to about 30 s. Breathing readings during that time can be wrong, and `breathing_confidence` rises more slowly than the reading improves.
- After a large change in breathing rate, the estimate takes about 25 to 45 s to follow in simulation. In one live run it reached the new rate about 30 s after the change, and can restart again later if the rate keeps changing, which briefly lowers confidence.
- Breathing rates can only be reported from 6 to 36 per minute.

## Test

```bash
colcon test --packages-select radar_vitals
colcon test-result --test-result-base build/radar_vitals
ros2 run radar_vitals replay_recording ../vitals/data/sitting_1m.npz
```

`replay_recording` runs the estimator on a recording from `vitals/iq_capture.py`, with no radar attached.

## Layout

- `radar_vitals_core/tlv.py`: decodes the radar's UART packets.
- `radar_vitals_core/estimator.py`: selects the range bin, measures chest motion and runs the belief filters. It has no ROS dependency.
- `radar_vitals_core/motion_points.py`: finds fine motion points with Capon beamforming.
- `scripts/radar_vitals_node`: configures the radar and publishes the topics.
- `scripts/breathing_plot`: shows the live breathing graphs.
- `rviz/vitals.rviz`: RViz view of the radar and breathing point clouds.
- `cfg/vitals_iwr6843aop.cfg`: radar configuration.
