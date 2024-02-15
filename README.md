# PADAWAN: Pixel Aware Differential-drive Autonomous robot With Artificial Neural networks

Raspberry Pi code for PADAWAN robot

The robot structure aims to be simple as possible. Using ROS and basic hardware, one should be able to power up the robot and start testing and probing. The objective is to collect data easy and quick to test computer vision algorithms. Some things the Pi3B will not be able to handle, naturally a more powerfull computer in the same network can process and send the results via ROS

## Hardware
 - Raspberry Pi 3 B
 - 2x DC motors
 - L298N motor driver board
 - Picam V2.1
 - MPU9250 9-axis IMU
 - 12 V battery
## Software
 - Ubuntu 20.04
 - ROS Noetic

Some useful details for understanding the robot.

### Libraries or packages installed
 - [imusensor](https://github.com/niru-5/imusensor) (using the setup.py script instead of "pip install")

### ROS nodes
- raspicam_node
- motor_driver
- imu_driver

![L298N board schematic](https://newscrewdriver.files.wordpress.com/2021/01/l298n-module-schematic-16x9-1.jpg?w=772)

![Raspberry Pi 3B pinout usage](/assets/padawan_pinout.png)

For quick reference when creating executable python scripts:
```
git update-index --chmod=+x file.sh
chmod +x file.sh

git add .
git commit -m 'commit message'
git push origin main
```

### TODO list
- [X] drive motors
- [X] read camera in decent fps
- [X] read IMU in decent Hz
- [X] create a node for the motors
- [X] create a node for the camera
- [X] create a node for the IMU
- [ ] bringup at boot
- [ ] (basic) process camera image and republish
- [ ] navigate based on the image processed
- [ ] estimate using IMU only
- [ ] navigate based on IMU estimation and processed image
- [ ] (not so basic) process camera image
- [ ] create a node for calibrating IMU or perform calibration at start
- [ ] create rviz models and tfs
