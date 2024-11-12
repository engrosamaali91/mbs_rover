# MBS ROVER

- [MBS ROVER](#mbs-rover)
- [Simulation](#simulation)
  - [Install Gazebo Fortress](#install-gazebo-fortress)
  - [Operation](#operation)
    - [RViz2](#rviz2)
    - [Teleop](#teleop)
    - [Fortress Simulation](#fortress-simulation)


![Robot design](assets/robot_design.gif)

# Simulation

## Install Gazebo Fortress 
Install ROS2 Humble -> Gazebo Fortress via:

```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```


Convert xacro to urdf 
```bash
xacro robot.xacro > rover.urdf 
```

## Operation

### RViz2

```bash
ros2 launch rover_viz view_robot.launch.py
```


### Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

### Fortress Simulation


```bash
ros2 launch rover_gazebo fortress_simulation.launch.py
```


### Nav2