# my_rb1_description

I created a URDF version of an RB1 robot that can be manipulated with joint_state_publisher_gui in rviz. It is from Phase 2, Part 1: Gazebo, Step 1: Build RB1 URDF Replica of [The Construct's Robot Developer Master Class](https://www.theconstructsim.com/robotics-developer/).

![Moving the joints using joint_state_publisher_gui](my_rb1_description.png)

Clone this project to your `catkin_ws/src`.

Open 1 shell

1. rviz
```
source ~/simulation_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch my_rb1_description rviz.launch
```

## ROS2

`build` shell: launch robot state publisher, joint state publisher and rviz2
```
cd ~/ros2_ws
rm -rf build install log
source /home/simulations/ros2_sims_ws/install/setup.bash
colcon build --packages-select phase2_custom_interfaces
colcon build
. install/setup.bash
ros2 launch my_rb1_description display.launch.py
```

Get error
unless comment out in  `my_rb1_description/CMakeLists.txt`
```
# find_package(joint_state_publisher REQUIRED)

# find_package(joint_state_publisher_gui REQUIRED)
```

```
...
--- stderr: my_rb1_description
CMake Error at CMakeLists.txt:20 (find_package):
  By not providing "Findjoint_state_publisher.cmake" in CMAKE_MODULE_PATH
  this project has asked CMake to find a package configuration file provided
  by "joint_state_publisher", but CMake did not find one.

  Could not find a package configuration file provided by
  "joint_state_publisher" with any of the following names:

    joint_state_publisherConfig.cmake
    joint_state_publisher-config.cmake

  Add the installation prefix of "joint_state_publisher" to CMAKE_PREFIX_PATH
  or set "joint_state_publisher_DIR" to a directory containing one of the
  above files.  If "joint_state_publisher" provides a separate development
  package or SDK, be sure it has been installed.


---
Failed   <<< my_rb1_description [6.72s, exited with code 1]
Aborted  <<< phase2_custom_interfaces [6.85s]
Aborted  <<< project_part2 [8.94s]

Summary: 7 packages finished [10.6s]
  1 package failed: my_rb1_description
...
```
Note that **ROS2** still has some issues, so please check that there is no node running when you stop your launch files; otherwise, it could generate conflicts and glitches.
in ros2 info
```
ros2 node list
```

If needed, manual joint state publisher
```
cd ~/ros2_ws
source install/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
