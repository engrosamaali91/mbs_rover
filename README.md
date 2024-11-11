# MBS ROVER

- [MBS ROVER](#mbs-rover)
- [Simulation](#simulation)
  - [Install Gazebo Fortress](#install-gazebo-fortress)
  - [Operation](#operation)


![Robot design](assets/robot_design.gif)

# Simulation

## Install Gazebo Fortress 
Install ROS2 Humble -> Gazebo Fortress via:

```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```


Convert xacro to urdf 
```bash
xacro robot.xacro > argo.urdf 
```

## Operation


```bash
ros2 launch rover_gazebo fortress_simulation.launch.py
```
