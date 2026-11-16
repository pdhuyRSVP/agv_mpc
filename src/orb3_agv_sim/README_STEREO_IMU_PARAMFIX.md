# Stereo IMU viewer / IMU sanity parameter fix

This patch fixes `orb3_stereo_imu_stable_launch.py` so command-line overrides such as `use_viewer:=true` are actually forwarded to `orb3_stereo_imu_launch.py`.

It also adds these parameters to `orb3_stereo_imu_node`:

- `enable_accel_sanity_check` default `false`
- `drop_zero_accel_samples` default `false`
- `min_acc_norm` default `0.0`
- `max_acc_norm` default `30.0`

Reason: Gazebo IMU acceleration conventions can differ; dropping frames at the wrapper level can starve ORB-SLAM3 and trigger repeated `not IMU meas` / active map resets.

Recommended first run:

```bash
ros2 launch orb3_agv_sim orb3_stereo_imu_stable_launch.py \
  use_viewer:=true \
  enable_accel_sanity_check:=false \
  drop_zero_accel_samples:=false
```
