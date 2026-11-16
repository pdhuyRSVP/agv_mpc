# orb3_agv_sim

AGV simulation in Gazebo Classic with ORB-SLAM3 RGB-D visual SLAM on ROS 2 Humble.

## Contents

```
orb3_agv_sim/
├── CMakeLists.txt
├── package.xml
├── README.md
├── src/
│   └── orb3_rgbd_node.cpp        # ROS 2 node wrapping ORB_SLAM3::System (RGB-D)
├── include/                      # (empty, required by CMake)
├── launch/
│   ├── sim_launch.py             # Gazebo Classic + robot + sensors
│   └── orb3_launch.py            # ORB-SLAM3 RGB-D node
├── config/
│   └── orb3_rgbd.yaml            # ORB-SLAM3 camera / ORB settings
├── urdf/
│   └── agv.urdf                  # Robot model (RGB-D camera, IMU, Lidar, diff drive)
├── worlds/
│   └── corridor.world            # Textured corridor with coloured walls + landmarks
├── meshes/                       # <-- COPY your .stl files here!
└── rviz/
    └── orb3.rviz                 # RViz layout (path, pose, camera, scan, tf)
```

## Prerequisites

- ROS 2 Humble
- Gazebo Classic 11 (`sudo apt install ros-humble-gazebo-ros-pkgs`)
- Common ROS packages (one-liner):
  ```bash
  sudo apt install ros-humble-cv-bridge ros-humble-vision-opencv \
                   ros-humble-tf2-geometry-msgs ros-humble-message-filters \
                   ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
                   ros-humble-teleop-twist-keyboard ros-humble-rviz2 \
                   libopencv-dev libeigen3-dev libglew-dev
  ```
- **ORB_SLAM3** built from source (default path: `~/Dev/ORB_SLAM3`). The library
  must exist at `~/Dev/ORB_SLAM3/lib/libORB_SLAM3.so`.
- **Pangolin** built and installed (`/usr/local/lib/libpangolin.so`).

## 1. Initial setup

```bash
# Assuming workspace is ~/agv_ws1/src
cd ~/agv_ws1/src
# Place this entire folder here as orb3_agv_sim/

# Copy your STL meshes (Base_footprint.stl, Base_Link.stl, Left_Wheel.stl,
# Right_Wheel.stl, Lidar.stl, Mam_xoay.stl) into:
cp /path/to/your/meshes/*.stl ~/agv_ws1/src/orb3_agv_sim/meshes/

# Verify mesh names match URDF expectations
ls ~/agv_ws1/src/orb3_agv_sim/meshes/
# Expected: Base_footprint.stl  Base_Link.stl  Left_Wheel.stl
#           Right_Wheel.stl     Lidar.stl      Mam_xoay.stl
```

## 2. Build

```bash
cd ~/agv_ws1
colcon build --packages-select orb3_agv_sim
source install/setup.bash
```

If ORB_SLAM3 is not at `~/Dev/ORB_SLAM3`, override the path:

```bash
colcon build --packages-select orb3_agv_sim \
    --cmake-args -DORB_SLAM3_ROOT=/your/actual/path/to/ORB_SLAM3
```

## 3. Run

Open four terminals. In each, first:

```bash
cd ~/agv_ws1 && source install/setup.bash
```

Then:

| Terminal | Command |
|----------|---------|
| T1 | `ros2 launch orb3_agv_sim sim_launch.py` |
| T2 | `ros2 launch orb3_agv_sim orb3_launch.py` |
| T3 | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=false` |
| T4 | `rviz2 -d $(ros2 pkg prefix orb3_agv_sim)/share/orb3_agv_sim/rviz/orb3.rviz` |

## 4. Verify each stage

```bash
# Camera topics publishing?
ros2 topic hz /camera/image_raw          # ~10 Hz
ros2 topic hz /camera/depth/image_raw    # ~10 Hz
ros2 topic hz /imu                       # ~200 Hz
ros2 topic hz /scan                      # ~10 Hz
ros2 topic hz /odom                      # ~50 Hz

# ORB-SLAM3 publishing pose?
ros2 topic hz /orbslam3/pose             # ~10 Hz (only when tracking!)
ros2 topic echo /orbslam3/pose --once
```

## 5. Teleop tips for ORB-SLAM3 RGB-D

The RGB-D mode needs **parallax** (different camera viewpoints of the same
features) to triangulate 3D points. After launching T3 (teleop):

1. Focus the teleop window.
2. Reduce speed: press **`x`** 6-7 times -> `speed 0.1 m/s`.
3. Reduce turn: press **`c`** 5 times -> `turn 0.3 rad/s`.
4. Drive: tap **`i`** (forward) for 2-3 s, pause, repeat.

In T2 look for:
```
[orb3_rgbd_node]: ORB-SLAM3 produced first non-identity pose
```
Once you see that message, tracking has started and `/orbslam3/path` will grow
as you drive.

## 6. View the sparse map

Three options:

### A. Pangolin viewer (built into ORB-SLAM3)
```bash
ros2 launch orb3_agv_sim orb3_launch.py use_viewer:=true
```
Shows keyframes, map points, covisibility graph live. **Note**: on some
systems the Pangolin viewer crashes on start due to GL/ROS thread conflict;
if that happens, use option B or C instead.

### B. RViz + path + pose (works always)
The provided `orb3.rviz` already has:
- `/orbslam3/path` (green line)
- `/orbslam3/pose` (red arrow)
- `/camera/image_raw` (image panel)
- `/scan`, `/tf`

You won't see the sparse ORB features, but you'll see the trajectory.

### C. Save the ORB-SLAM3 Atlas to disk
Edit `config/orb3_rgbd.yaml`:
```yaml
System.SaveAtlasToFile: "my_map"
```
After you Ctrl-C the node, `my_map.osa` is saved. To reload:
```yaml
System.LoadAtlasFromFile: "my_map"
```

## Troubleshooting

| Symptom | Cause / Fix |
|---------|-------------|
| `ORB_SLAM3 library not found` during build | Wrong path. Override: `colcon build --cmake-args -DORB_SLAM3_ROOT=/your/path` |
| `Pangolin library not found` | Build & install Pangolin from source |
| `Failed to load plugin libgazebo_ros_openni_kinect.so` | URDF uses `libgazebo_ros_camera.so` (already correct) — if you see this, you modified the URDF incorrectly |
| Robot not visible in Gazebo | `package://` not rewritten. Launch file does this automatically — check log for "package-> file://" line |
| No `/camera/*` topics | Check URDF camera plugin block; `ros2 topic list \| grep camera` |
| `ORB-SLAM3 still returns identity pose` | Robot is stationary. Drive with teleop (see section 5) |
| Scene too uniform, can't track | Increase `ORBextractor.nFeatures` in `config/orb3_rgbd.yaml`; or drive closer to coloured walls / landmarks |
| OpenCV YAML parse error | Make sure the YAML has NO comment lines containing `:`, `->`, `=` outside of values |

## Published topics by orb3_rgbd_node

| Topic | Type | Description |
|-------|------|-------------|
| `/orbslam3/camera_pose` | `geometry_msgs/PoseStamped` | Camera pose in `orb_map` frame |
| `/orbslam3/pose` | `geometry_msgs/PoseStamped` | Robot base pose in `orb_map` frame (uses TF to convert from camera pose) |
| `/orbslam3/path` | `nav_msgs/Path` | Accumulated trajectory of robot base |

## License

BSD
