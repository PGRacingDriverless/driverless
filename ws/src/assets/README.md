Assets Repo
=============================
This repository contains the robot, its model, worlds, launch script for simulations in Gazebo.
- [Robot](#Robot)
    - [Build and Run](#Build-and-Run)
- [List of worlds](#List-of-worlds)

# Robot
The robot is based on TurtleBot3 with the added Intel RealSense depth camera and Velodyne Lidars.

## Build and Run
Build:
```
cd ~/ws
colcon build --symlink-install
```

Source the environment:
```
source install/setup.bash
```

Export the waffle model with the RealSense camera:
```
export TURTLEBOT3_MODEL=waffle
```

Export the desired world for the Gazebo simulation:
```
export GAZEBO_WORLD=turtlebot3_world
```

Launching the simulation in Gazebo:  
```
ros2 launch turtlebot3_gazebo run.launch.py
```
> **Important**  
> Gazebo can take a long time to load worlds because it loads models from internet resources.

To control the robot with the keyboard:  
```
ros2 run turtlebot3_teleop teleop_keyboard
```

# List of worlds
- empty_world
- turtlebot3_world
- turtlebot3_house
- turtlebot3_dqn_stage4
- jackal_race
