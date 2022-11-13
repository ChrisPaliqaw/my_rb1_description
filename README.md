# my_rb1_description

I created a URDF version of an RB1 robot that can be manipulated with joint_state_publisher_gui in rviz. It is from Phase 2, Part 1: Gazebo, Step 1: Build RB1 URDF Replica of [The Construct's Robot Developer Master Class](https://www.theconstructsim.com/robotics-developer/).

![Moving the joints using joint_state_publisher_gui](my_rb1_description.png)

Clone this project to your `catkin_ws/src`.

Open 3 shells

1. roscore

```
source ~/catkin_ws/devel/setup.bash
roscore
```

2. gazebo

```
source ~/simulation_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch my_rb1_description my_rb1_model.launch
```

3. rviz
```
source ~/simulation_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch my_rb1_description rviz.launch
```
