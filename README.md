# Mowbot with ArduPilot Flight Controller & Raspberry Pi 5 Companion Computer
## Bringup Instructions
## Robot Terminal 1
### for DDS (DDS must be enabled for FCU serial port)
```
ssh mowbot-pi5
cd ros2_mowbot_ws/
source install/setup.bash
sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 2000000
```
### or (preferred)
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 2000000
```

### for MAVROS (MAVLINK 2 must be enabled for FCU serial port)
```
ssh mowbot-pi5
cd ros2_mowbot_ws/
source install/setup.bash
```
### for FCU serial connector <-> PI5 header serial
```
ros2 launch mavros apm.launch fcu_url:=/dev/ttyAMA0:2000000
```
### for FCU USB <-> PI5 USB
```
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:2000000
```

## Robot Terminal 2
```
ssh mowbot-pi5
cd ros2_mowbot_ws/
source install/setup.bash
ros2 launch robot_bringup robot_bringup.launch.py
```

### Host Terminal 1
```
rviz2
```

## Service Calls
```
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 15}"
```

## Todo
- test publishing /joy from host with xbox controller
- need to install and test realsense node on robot
- need to install and test ros2 nav2 on robot
- sync ROS2 to flight controller time?