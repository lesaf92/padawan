#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray

import sys
import time
import smbus

from imusensor.MPU9250 import MPU9250

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

print('Calibrating gyro...')
imu.caliberateGyro()
print('Done.')
print('Calibrating acc...')
imu.caliberateAccelerometer()
print('Done.')

# or load your own caliberation file
#imu.loadCalibDataFromFile("/home/pi/calib_real_bolder.json")

while True:
	imu.readSensor()
	imu.computeOrientation()
	print(imu.AccelVals)
	print(imu.GyroVals)
	print(imu.MagVals)
	print("roll: {0} ; pitch : {1} ; yaw : {2}".format(imu.roll, imu.pitch, imu.yaw))