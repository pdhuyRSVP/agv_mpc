Copy your STL mesh files into this directory.

The URDF (urdf/agv.urdf) expects exactly these filenames:
    Base_footprint.stl
    Base_Link.stl
    Left_Wheel.stl
    Right_Wheel.stl
    Lidar.stl
    Mam_xoay.stl

Without these files, the robot will spawn in Gazebo but appear as a
collection of collision boxes (no visuals).
