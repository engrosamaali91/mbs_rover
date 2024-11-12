# Quick Usage Instructions

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Odom Navigation

- Navigate in global frame of `map`

```bash
ros2 launch rover_nav2 odom_navi.launch.py
```

## Mapping

```bash
ros2 launch rover_nav2 slam.launch.py
```

- Mapping can begin using the `joystick controller`. Once you are satisfied with your map you can export it by running the following command in one of the husky terminals:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/administrator/ros2_ws/src/mybotshop/rover_nav2/maps/custom_map
```

- Rebuild so that it can be found

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Map Navigation

### Localization Node

```bash
ros2 launch rover_nav2 localization.launch.py
```

### Autonomous Navigation

```bash
ros2 launch rover_nav2 map_navi.launch.py
```

## GPS Navigation

Ensure that the Emlid GPS is activated and the `/emlid/fix` topic is being published and then launch:

```bash
ros2 launch hsky_waypoint system.launch.py
```

- Record GPS points via:
  
```bash
ros2 service call /a200_1045/record_gps_waypoints rover_msgs/srv/String "data: 'record'"
```

> Steamdeck **X + Up Arrow**


- Save GPS points and Odom points

```bash
ros2 service call /a200_1045/record_gps_waypoints rover_msgs/srv/String "data: 'save'"
```

> Steamdeck **X + Down Arrow**

- Run GPS navigation

```bash
ros2 service call /a200_1045/record_gps_waypoints rover_msgs/srv/String "data: 'activate_gps_waypoints'"
```

> Steamdeck **X + Right Arrow**

# Dependencies

```bash
sudo apt-get install ros-$ROS_DISTRO-twist-mux\
                     ros-$ROS_DISTRO-navigation2\
                     ros-$ROS_DISTRO-nav2-bringup
```