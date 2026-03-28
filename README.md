

##
ROS2 - Humble
## Requirements

- [UR_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [UR_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [UR_Client_Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library)


## How to run the simulation
To run the simulation, first souce your workspace's setup.bash. Then run,
```
ros2 launch ur5_weaver bringup.launch.py
```
That launch contains the simulation, controllers, moveit, and additional nodes, such as the marker_detector and the weaver_trajectory_generator server.

Finally, to generate the trajectory from the image and make the robot move, simply run
```
ros2 launch ur5_weaver motion_planning.launch.py
```

## Future Works
### Display trajectory
By using the MoveitTaskConstructor (MTC) to manage the trajectories, we're not able to publish the trajectories segments and visualize in Rviz.
One solution is to rewrite the code to work with PlanningComponents.
## MoveitCpp

### RobotModel
contains the relationship between all linkg and joints inlcuding their joint limit properties loaded from the URDF. It should be loaded by the RobotModelLoader

### RobotState
contains information about the robot at a certain point in time, storing joint positions, velocities and accelerations.

### JointModelGroup
represents the robot model for a particular group, such as the robot's arm.

### Planning Scene
