Graph SLAM Repo
=============================
Graph-based SLAM using NDT-OMP and GICP algorithms.
- [Overview](#Overview)
    - [Scheme](#Scheme)
    - [Description](#Description)
- [Build and Run](#Build-and-Run)

## Overview
### Scheme
```
    /cloud
       ⇩
----------------
| prefiltering |
----------------
       ⇩
/filtered_points
       ⇩
----------------
|              | ⇨ /local_map /scan_match_path /scan_match_pose /scan_match_odom
| scan_matcher | ⇦ Odometry
|              | ⇦ IMU
----------------
       ⇩
  /lidar_frame
       ⇩
----------------
|   backend    |
----------------
       ⇩
/modified_map
/modified_path
/modified_lidar_frame
/candidate_lidar_frame
```

### Description
- graphslam - basic package that runs all nodes of the GraphSLAM.
- graphslam_backend - all manipulations with graphs (loops closing, optimization, etc.).
- graphslam_prefiltering - filtering of points received from LiDARs  (distance, downsample, outlier filter).
- graphslam_scan_matcher - data matching using NDT-OMP or GICP or FAST-GICP.
- graphslam_msgs - messages and services for GraphSLAM packages.
- graphslam_utils - utility functions for the GraphSLAM.
- thirdparty - third-party packages.

The path to the robot's base frame `base_frame_id` is `<your_ws_path>/slam/graphslam_scan_matcher/config/graphslam_scan_matcher.param.yaml`.

## Build and Run
Install dependencies:
```
cd ~/ws
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

Build:
```
cd ~/ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Source the environment:
```
source install/setup.bash
```

Launching GraphSLAM with Rviz:
```
ros2 launch graphslam graphslam.launch.xml
```
