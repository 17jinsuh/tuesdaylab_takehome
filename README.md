## About
This project implements a simple motion planning and control system for a simulated 2-D turtle robot (`turtlesim`) using ROS2. The robot navigates toward target poses issued by either a user or an autonomous clock-based system.

## Prerequisites
- **Ubuntu 22.04**
- **ROS2 Humble**
- **Bash**

## Features
- **Target-based motion control** (2-phases)
  - Phase 1: Rotate in place to align with target orientation.
  - Phase 2: Drive forward until Target position is reached.
- **Pose input from two sources:**
  - **GUI/CLI user input** (via terminal)
  - **Clock-based autonomous pose generator**
- **Pose priority management**
  - User poses take priority.
  - Automatically switches back to clock pose after a timeout or a spacebar press.
- **Clean Pub-Sub and Service-Client architecture** (follow image below)
![alt text](https://github.com/17jinsuh/turtlesim_guiclockpose/blob/main/images/architecture.png "Control Loop Architecture")

## Nodes
### `clock_pose_issuer`
  - Publishes: 
    - `/clock_pose`: Clock-based target pose (based on world frame, NOT turtle frame)

### `guicli_pose_issuer`
  - Publishes: 
    - `/guicli_pose`: GUI/CLI-based target pose (based on world frame, NOT turtle frame)

### `pose_manager`
- Subscribes to:
  - `/clock_pose`
  - `/guicli_pose`
- Publishes:
  - `/target_pose`: Target pose to reach.
- Service:
  - `/get_target_pose`:
    - Receives request: `need_new_target` (bool)
    - Responses `updated_target` (bool) and publishes `/target_pose`.

### `motion_controller`
- Subscribes to:
  - `/turtle1/pose`: Current robot pose.
  - `/target_pose`
- Publishes:
  - `/turtle1/cmd_vel`: Velocity commands to robot.
- Service:
  - `/get_target_pose`:
    - Requests `need_new_target` (bool)
    - Receives response `updated_target` (bool) and updates `target_pose_` (class variable).

### Robot movement direction based on pose input
| Minute hand Input  | (x, y) in world frame | Head direction in world frame |
| ------------- | ------------- | ------------- |
| 0 or 60  | (1, 0)  | +x-axis |
| 30       | (-1, 0) | -x-axis |
| 45       | (0, 1)  | +y-axis |
| 15       | (0, -1) | -y-axis |

**Note:** Turtle will spawn at x=[5.544445], y=[5.544445], theta=[0.0] (where theta = 0 is the +x-axis). Inputs in between the above minute inputs will go to their respective quadrants.

![alt text](https://github.com/17jinsuh/turtlesim_guiclockpose/blob/main/images/turtle_in_world_frame.png "Turtle in World Frame")

## Installation
- In the project directory, run
```
cd ~/${PROJECT_DIR}
source setup.sh
```
- **What's happening inside `setup.sh`?**
  - Check if **Ubuntu 22.04** is installed.
  - Check if **ROS2 Humble** is installed.
  - Build repo using `colcon build`
  - Source `install/setup.bash` to prepare for runs.

## How to Use
### Run full simulation run
1. Run `guicli_pose_issuer`.
```
ros2 run algorithms guicli_pose_issuer
```
2. Open another new terminal (preferably, new window) and source the most updated `install/setup.bash`. Launch `turtlesim` simulation, `clock_pose_issuer`, `pose_manager`, `motion_controller`. (Prefer to keep the turtlesim simulation window on "Always on Top"). 
```
cd ~/${PROJECT_DIR}/
source install/setup.bash
ros2 launch bringup fullrun.launch.xml
```

3. In the terminal where `guicli_pose_issuer` is running, add minute hand inputs from 1 to 60 (or 0)
```
Enter minute hand from [1, 60] (or press <space> and Enter to use clock pose): 30
```
or press `space` and Enter to switch to clock-based poses.

4. **Expected Behavior**:
- `pose_manager` receives `/clock_pose` topic and publishes it as an active `target_pose` topic.
- Once a `/gui_cli_pose` is received, `pose_manager` prioritizes the `/gui_cli_pose` as the active `target_pose` and publishes it.
- Robot moves to desired target pose, and will stop motion once target pose is reached. 
- If no user gui/cli input is given for at least the `gui_timeout` (default: 30 seconds), then `pose_manager` continues to publish the latest `/clock_pose` topic.

### Run turtlesim with GUI/CLI commands only
1. Launch `turtlesim` simulation, `pose_manager`, `motion_controller`. 
(Prefer to keep the turtlesim simulation window on "Always on Top").
```
ros2 launch bringup guicli_turtlesim_test.launch.xml
```
2. Open another new terminal (preferably, new window) and source the most updated `install/setup.bash`. 
Run `guicli_pose_issuer`.
```
cd ~/${PROJECT_DIR}/
source install/setup.bash
ros2 run algorithms guicli_pose_issuer
```
3. In the terminal where `guicli_pose_issuer` is running, add minute hand inputs from 1 to 60 (or 0)
```
Enter minute hand from [1, 60] (or press <space> and Enter to use clock pose): 30
```
or press `space` and Enter to switch to clock-based poses.

3. **Expected Behavior**:

- `pose_manager` receives `/gui_cli_pose` topic and publishes it as an active `target_pose` topic.
- Robot moves to desired target pose, and will stop motion once target pose is reached. 

### Test `get_target_pose` Service-Client setup.
1. Launch `pose_manager` and `clock_pose_issuer`. 
```
ros2 launch bringup posemanager_test.launch.xml
```
2. Open another new terminal (preferably, new window) and source the most updated `install/setup.bash`. 
Run `get_target_pose_srv_test`.
```
cd ~/${PROJECT_DIR}/
source install/setup.bash
ros2 run algorithms get_target_pose_srv_test
```
3. **Expected Behavior**:
- `pose_manager` will continuously subscribe to the `/clock_pose` topic and publish it as `/target_pose` topic.
- Once `get_target_pose_srv_test` is called, `pose_manager` receives a request for a new target pose.
- `pose_manager` sends response that target pose is updated, and `get_target_pose_srv_test` receives the updated message.


## How to tune parameters
1. Modify parameters in `~/${PROJECT_DIR}/src/bringup/config/robot_config.yaml`
2. Rebuild `bringup` package
```
cd ~/${PROJECT_DIR}
colcon build --packages-select bringup
source install/setup.bash
```
3. Run full simulation and see changed behaviors from modified parameters.

## Future Works
- Currently, Robot takes the target pose orientation based on the world frame. Rotating based on its most updated pose orientation is ideal.
- Adding GUI implementation as user input. More options for user to command target pose input.
- `spacebar` is recognized as a clock pose input instead of replacing the previous gui input. Stack (LIFO) or Queue (FIFO) data structures can be used for such feature.

## Dependencies
- `rclcpp`
- `geometry_msgs`, `turtlesim`, `tf2`

## Misc.
### Install ROS2 Humble

To install ROS2 follow the official instructions provided here.

Make sure to install the version that corresponds to the branch you require or slight modifications may be required to compile the code.

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

### Building Individual Packages
  `colcon build --packages-select <package_name>`
  example:
  `colcon build --packages-select algorithms`

### Using ros2 run
- You can run each node individually using ros2 run using
  `ros2 run <package_name> <node_executable_name>`

### Using ros2 launch
- Launch files are wrtitten using python scripts
- To invoke a node you can run the python script using ros2 launch like below
  `ros2 launch <package_name> <launch_file_name>`
