# my_rb1_description

Clone this project and `my_rb1_gazebo` to your `catkin_ws`.

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
roslaunch my_rb1_gazebo my_rb1_robot_warehouse.launch
```

3. Run rviz, then load my_rb1_description/rviz_config/my_rb1.rviz
