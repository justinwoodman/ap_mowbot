# Mowbot with ArduPilot Flight Controller & Raspberry Pi 5 Companion Computer
## Bringup Instructions
## Robot Terminal 1
### for DDS (DDS must be enabled for FCU serial port)
```
ssh mowbot-pi5
cd ros2_mowbot_ws/
source install/setup.bash

# To use MicroXRCEAgent
sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 2000000

# To use micro_ros_agent (preferred)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 2000000
```

### for MAVROS (MAVLINK 2 must be enabled for FCU serial port)
```
ssh mowbot-pi5
cd ros2_mowbot_ws/
source install/setup.bash

# for FCU serial connector <-> PI5 header serial
ros2 launch mavros apm.launch fcu_url:=/dev/ttyAMA0:2000000

# for FCU USB <-> PI5 USB
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:2000000
```

## Robot Terminal 2
### Launches sensors and AP to ROS2 bridges
```
ssh mowbot-pi5
cd ros2_mowbot_ws/
source install/setup.bash
ros2 launch robot_bringup robot_bringup.launch.py
```

## Host Terminal 1
### Start RViz2
```
rviz2
```

## Host Terminal 2
### Launch Navigation
```
ros2 launch nav2_bringup navigation_launch.py params_file:="/home/ros/Desktop/navigation.config.yaml"
```

## Host Terminal 3
### Stamp the cmd_vel messages from Navigation
```
ros2 run twist_stamper twist_stamper --ros-args -r cmd_vel_in:=cmd_vel -r cmd_vel_out:=cmd_vel_stamped
```

## Service Calls
```
# in DDS mode
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 15}"

# in MAVROS mode
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: MANUAL}"
```

## Todo
- need to install and test realsense node on robot
- need to install and test ros2 nav2 on robot
- sync ROS2 to flight controller time?
- rename "ardupilot_tf_publisher" something like "ardupilot_odometry_bridge"
- maybe split out navsat stuff and call it "ardupilot_navsat_bridge"?
- both could belong to a node called "ardupilot_bridges"?

## Done
- test publishing /joy from host with xbox controller