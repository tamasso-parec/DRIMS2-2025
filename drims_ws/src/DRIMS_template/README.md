# DRIMS2 Simulation Environment

## Gazebo Simulation

In order to run the simulation:

1. Load Gazebo environment and robot simulation:
```bash
roslaunch drims_dice_demo arm_gazebo.launch robot:=yumi # or robot:=gofa
```

2. Launch low level ROS Services Server for robot planning and control:
```bash
roslaunch abb_wrapper_control launchControlServer.launch robot:=yumi # or robot:=gofa
```

3. Launch high level ROS Services Server for robot planning and control:
```bash
roslaunch abb_wrapper_control launchTaskServer.launch
```

4. Spawn the dice in simulation:
```bash
roslaunch drims_dice_demo spawn_dice.launch
```

**NOTE:** Each command should run in its own terminal

## Example nodes

Two example nodes are provided in both Python and C++ to perform a simple grasp of the dice:

For the **Python** version, launch the following ROS node:

```bash
rosrun drims_template take_the_dice_example.py
```

Instead, for the **C++** version:

```bash
roslaunch drims_template test_example.launch
```
This launcher runs the script `test_example.cpp` located in the `DRIMS_template` ROS package.

ATTENTION: remember to press 'next' in the RvizVisualToolsGui window to execute the motion on the robot.
(Check the terminal where you launched the `launchControlServer.launch` to visualize the previous blue message)

Both of the example programs present a basic use of the services to move the robot:
- `/plan_and_execute_pose`: plan a collision-free joint trajectory to a given goal Cartesian pose and execute it.
- `/plan_and_execute_slerp`: plan a collision-free joint trajectory by using SLERP interpolation to a given a goal Cartesian pose and execute it.
- `/plan_and_execute_joint`: plan a collision-free joint trajectory to a given a Joint goal and execute it.
- `/open_gripper`: open the gripper.
- `/close_gripper`: close the gripper.

In the following description are reported the ROS service definition, located in the `srv` folder of `abb_wrapper_msgs` ROS package:

**Service structure `abb_wrapper_msgs/plan_and_execute_pose`**
```
# == Request ==
geometry_msgs/Pose goal_pose
bool is_relative
---
# == Response ==
bool success
string message
```

**Service structure `abb_wrapper_msgs/plan_and_execute_joint`**
```
# == Request ==
float64[] joint_goal
---
# == Response ==
bool success
string message
```

**Service structure `abb_wrapper_msgs/plan_and_execute_slerp`**
```
# == Request ==
geometry_msgs/Pose goal_pose
bool is_relative
---
# == Response ==
bool success
string message
```

**Service structure `abb_wrapper_msgs/open_gripper` and `abb_wrapper_msgs/open_gripper`**
```
bool in_flag
---
bool out_flag
string message
```
## Dice manipulation task

Now, you can start to create your own task for dice manipulation.

For the **C++** version, you can modify this ROS node (`task_dice.cpp`) between lines 154-164 to create your sequence of actions to perform the dice manipulation.
Remember that you can launch the previous ROS node by using the following command:
```bash
roslaunch drims_template task_dice.launch
```




