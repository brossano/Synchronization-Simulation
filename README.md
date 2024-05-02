# ROB 450 Final Project

## Ford AMR/AGV Synchronization
This repository hold the code for the ROB 450 synchronization final project. This branch is for running the sychronization stack on Michigan Robotics MBots. You can check out the simulation based code in the other branch.

NOTE: I apologize for the package naming. All of the code falls under a package called *image_proc*, which started as just part of the sychronization but became the entire thing.

NOTE: Some of the ROS2 nodes take in params files that must be adjusted in the code for different machines
#### Running the stack (once inside ROS workspace)
```
source install/setup.bash
ros2 launch image_proc mbot_slam_nav.launch.py
(New Terminal)
source install/setup.bash
ros2 run image_proc amr_nav
```

