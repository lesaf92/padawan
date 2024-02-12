# PADAWAN: Pixel Aware Differential-drive Autonomous robot With Artificial Neural networks

Raspberry Pi code for PADAWAN robot

The robot structure aims to be simple as possible. The objective is to collect data fast and easy to test computer vision algorithms. 

## Hardware
 - Raspberry Pi
 - 2x DC motors
 - L298N motor driver board
 - Picam V2.1
 - ...
## Software
 - ROS Noetic
 - Python
 - C++

TODO list
- [X] drive motors
- [X] read camera in decent fps
- [ ] read IMU in decent Hz
- [X] create a node for the motors
- [X] create a node for the camera
- [ ] create a node for the IMU
- [ ] bringup at boot
- [ ] process camera image and republish
- [ ] navigate based on the image processed
- [ ] estimate using IMU only
- [ ] navigate based on IMU estimation and processed image
