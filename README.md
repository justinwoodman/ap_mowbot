# Mowbot with ArduPilot Flight Controller & Raspberry Pi 5 Companion Computer
## Bringup Instructions

## Mission Planner (to be replaced with service calls...)
### Start RTK/GPS Inject
### Must be in "Guided" mode
### Must be armed

## rpanion-server
### http://mowbot-pi5:3001/

## Robot Terminal 1
### for DDS (DDS must be enabled for FCU serial port)
```
ssh mowbot-pi5
cd ros2_mowbot_ws/
source install/setup.bash

# To use micro_ros_agent (preferred)
# for FCU serial connector <-> PI5 header serial
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 2000000
# for FCU serial connector <-> FTDI <-> PI5 USB
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 2000000
# for FCU USB <-> PI5 USB
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1 -b 2000000

# To use MicroXRCEAgent
# for FCU serial connector <-> PI5 header serial
sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 2000000
# for FCU serial connector <-> FTDI <-> PI5 USB
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 2000000
# for FCU USB <-> PI5 USB
sudo MicroXRCEAgent serial --dev /dev/ttyACM1 -b 2000000
```

## Robot Terminal 2
### Launches sensors and AP to ROS2 bridges
```
ssh mowbot-pi5
cd ros2_mowbot_ws/
source install/setup.bash
ros2 launch robot_bringup robot_bringup.launch.py
```

## Robot or Host Terminal (possibly better on faster host?)
### Launch Navigation
```
ros2 launch nav2_bringup navigation_launch.py params_file:="/home/ros/ros2_mowbot_ws/src/robot_bringup/config/navigation.config.yaml"
```

## Host Terminal 1
### Start RViz2
```
rviz2
```

## For Xbox Controller Teleop Bringup
## Host Terminal 1
### Start joy node
```
ros2 run joy joy_node
```

## Host Terminal 2
### Start teleop_twist_joy
```
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' publish_stamped_twist=True
```

## Service Calls
```
# in Ardupilot DDS mode
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 15}"
```

## Todo
- Robot bringup should start nav2 instead of manually on host
- create host_bringup package that starts rviz2, joy, and teleop_twist_joy
- sync ROS2 to flight controller time?

## Done
- test publishing /joy from host with xbox controller
- need to install and test realsense node on robot
- need to install and test ros2 nav2 on host
- nav2 config file should be in bringup package
- Robot bringup should start twist stamper instead of manually on host