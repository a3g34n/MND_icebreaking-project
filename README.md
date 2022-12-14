# MND_icebreaking-project

Given the tricycle mobile robot configuration The front wheel is used for both steering and traction. The unactuated rear wheels are for stabilization only. The steering angle is constrained between -45 and +45 degrees. The maximum linear and angular velocities should be bounded and the initial posture of the robot Pi = (x, y, theta) and given the waypoints Wp = (x, y) the objective is;

  - calculating global path which is passing through the way-points
  
  - simulating path tracking of the robot and plot the linear and angular velocities (v, w) of the robot

## How to launch model:

### To start gazebo with tricycle robot use:

```
roslaunch tricycle_gazebo tricycle_empty_world.launch
```

### To start teleoperation with keyboard:

```
roslaunch tricycle_control teleop.launch
```

### To start autonomous movement to waypoints:

```
rosrun tricycle_navigation go_to_point.py
```

## How to get this project up and running

  - [x] Steering joint controller must be replaced from Velocity_controller to Position_controller
  - [x] Calculations in odometry.py should be fixed
  - [ ] Multiple waypoints should be chosen and the global path must be calculated in a script
  - [ ] A script must be written for the robot to follow the path