# Mowbot with ArduPilot Flight Controller & Raspberry Pi 5 Companion Computer
## Bringup Instructions
### Terminal 1 (robot)
```
ssh mowbot-pi5
sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 2000000
```

### Terminal 2 (robot)
```
ssh mowbot-pi5
cd ros2_mowbot_ws/
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_s3_launch.py
```

### Terminal 3 (host, eventually robot, rolling in lidar above plus realsense)
```
cd ros2_mowbot_ws/
source install/setup.bash
ros2 launch robot_bringup robot_bringup.launch.py
```

### Terminal 4 (host)
```
rviz2
```

## Todo
- test publishing /joy from host with xbox controller
- need to install and test realsense node on robot
- need to install and test ros2 nav2 on robot
- sync ROS2 to flight controller time?