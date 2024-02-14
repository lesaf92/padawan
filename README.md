# PADAWAN: Pixel Aware Differential-drive Autonomous robot With Artificial Neural networks

Raspberry Pi code for PADAWAN robot

The robot structure aims to be simple as possible. Using ROS and basic hardware, one should be able to power up the robot and start testing and probing. The objective is to collect data easy and quick to test computer vision algorithms. Some things the Pi3B will not be able to handle, naturally a more powerfull computer in the same network can process and send the results via ROS

## Hardware
 - Raspberry Pi 3 B
 - 2x DC motors
 - L298N motor driver board
 - Picam V2.1
 - MPU6050 IMU
 - 12 V battery
## Software
 - Ubuntu 20.04
 - ROS Noetic

Some useful details for understanding the robot.

### ROS nodes
- raspicam_node
- robot_upstart
- motor_driver

![L298N board schematic](https://newscrewdriver.files.wordpress.com/2021/01/l298n-module-schematic-16x9-1.jpg?w=772)
L298N board schematic
![Raspberry Pi 3B pinout usage](/assets/padawan_pinout.png)
Raspberry Pi pin usage

### TODO list
- [X] drive motors
- [X] read camera in decent fps
- [ ] read IMU in decent Hz
- [X] create a node for the motors
- [X] create a node for the camera
- [ ] create a node for the IMU
- [ ] bringup at boot
- [ ] (basic) process camera image and republish
- [ ] navigate based on the image processed
- [ ] estimate using IMU only
- [ ] navigate based on IMU estimation and processed image
- [ ] (not so basic) process camera image
