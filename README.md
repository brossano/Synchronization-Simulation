# ROB 450 Final Project

## Ford AMR/AGV Synchronization
This repository hold the code for the ROB 450 synchronization final project. This branch is for running the sychronization stack in simulation using Gazebo. You can check out the real-time code in the other branch.

NOTE: I apologize for the package naming. A lot of the code falls under a package called *image_proc*, which started as just part of the sychronization but became the entire thing. I haven't gotten around to cleaning that up yet. 

NOTE: Some of the ROS2 nodes take in params files that must be adjusted in the code for different machines. Please take a look at the main launch file that's being run below.
#### Running the stack (once inside ROS workspace)
```
colcon build
source install/setup.bash
ros2 launch turtlebot3_gazebo multi_turtlebot3_velodyne_world.launch.py
```

