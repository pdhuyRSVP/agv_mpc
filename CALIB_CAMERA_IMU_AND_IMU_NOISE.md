# Camera-IMU Tbc and HWT901B IMU Noise Calibration

## Current geometry-derived Tbc

For the current AGV geometry:

- `Base_footprint -> imu_link`: `[0.0175, 0.014007, 0.1425]`
- `Base_footprint -> color_optical_frame`: `[0.3875, -0.0030, 0.1425]`
- `color_optical_frame` uses ROS optical axes: x right, y down, z forward.

So `imu_link -> color_optical_frame` translation is:

```text
[0.370000, -0.017007, 0.000000]
```

The matching ORB-SLAM3 block is:

```yaml
IMU.T_b_c1: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [0.0,  0.0,  1.0,  0.370000,
        -1.0,  0.0,  0.0, -0.017007,
         0.0, -1.0,  0.0,  0.000000,
         0.0,  0.0,  0.0,  1.0]
```

Check/regenerate it:

```bash
cd ~/agv_mpc
python3 scripts/check_tbc_camera_imu.py
```

This is a geometry-derived transform, not a dynamic camera-IMU calibration. For best RGB-D-Inertial quality, run Kalibr with a calibration target.

## Quick IMU noise check

Robot must be completely still.

```bash
cd ~/agv_mpc
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/calibrate_imu_noise.py \
  --topic /imu/data_raw \
  --duration 120 \
  --output-dir ~/agv_mpc/calib/imu_noise_$(date +%Y%m%d_%H%M%S)
```

This writes:

- `imu_noise_metrics.json`
- `orb_imu_noise_preliminary.yaml`

Use the preliminary `IMU.NoiseGyro` and `IMU.NoiseAcc` only as a sanity check. Do not blindly reduce ORB-SLAM3 noise from a short static run.

## Proper Allan variance data

For `IMU.GyroWalk` and `IMU.AccWalk`, record a long static bag:

```bash
mkdir -p ~/agv_mpc/calib
ros2 bag record /imu/data_raw \
  -o ~/agv_mpc/calib/imu_static_$(date +%Y%m%d_%H%M%S)
```

Recommended duration: 30-60 minutes, robot still, no vibration.

Then process with Allan variance tools. Until that is done, keep the current conservative walk values in the ORB YAML.

## Live status found during setup

At the time of this check, the running IMU node still published `/imu` at about 50 Hz and published AHRS orientation. Restart the sensor stack with the updated launch to get `/imu/data_raw` at 100 Hz and `orientation_covariance[0] = -1`.
