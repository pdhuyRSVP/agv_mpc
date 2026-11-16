# Stereo-IMU OpenCV memory fix

Symptom:

```text
OpenCV(4.6.0) ... alloc.cpp:73: error: (-4:Insufficient memory) Failed to allocate ... bytes
ORB3 stereo callback exception
```

Cause in this package:

The first stereo package converted ROS Image messages through `cv_bridge::toCvCopy(..., MONO8)` directly. If Gazebo/cv_bridge sees inconsistent image metadata or an unexpected encoding while converting, OpenCV may try to allocate a matrix with a nonsensical size and the node dies.

Fixes applied:

1. `orb3_stereo_imu_node.cpp`
   - Added `imageMsgToMono8()`.
   - Validates `width`, `height`, `step`, `data.size()` before any OpenCV conversion.
   - Supports `mono8`, `rgb8`, `bgr8`, `rgba8`, `bgra8`, `8UC1`, `8UC3`, `8UC4`.
   - Drops invalid frames instead of letting OpenCV allocate huge memory.
   - Catches `cv::Exception` explicitly.

2. `urdf/agv.urdf`
   - Stereo left/right cameras now publish grayscale `L8` images.
   - Stereo camera update rate lowered to 10 Hz for weak laptops.

3. `config/orb3_stereo_imu_gz.template.yaml`
   - `Camera.fps: 10`
   - `ORBextractor.nFeatures: 800`
   - `Camera.RGB: 0`

4. `launch/orb3_stereo_imu_launch.py` and stable launch
   - Default `use_viewer:=false` to reduce memory while testing.

Recommended first run:

```bash
ros2 launch orb3_agv_sim orb3_stereo_imu_stable_launch.py use_viewer:=false
```

Then if stable:

```bash
ros2 launch orb3_agv_sim orb3_stereo_imu_stable_launch.py use_viewer:=true
```
