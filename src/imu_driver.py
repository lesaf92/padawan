#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray

import numpy as np
import time
import os
import smbus

from imusensor.MPU9250 import MPU9250
'''
IMU driver node. Publishes data in a 12 position array [accXYZ, gyrXYZ, magXYZ, RPY] (temporarily)
Loads a previously saved calibration file to begin publishing.
'''
###############################################################################
############################# Main starts here ################################
###############################################################################
def main():
    rospy.init_node('imu_driver', anonymous=False)
    rate = rospy.Rate(100)
    imu_pub = rospy.Publisher('imu_data', Float64MultiArray, queue_size=10)
    # Initializing the imu class using i2c at address 0x68
    imu = MPU9250.MPU9250(smbus.SMBus(1), 0x68)
    imu.begin()
    filepath = os.path.dirname(os.path.abspath(__file__)) + '/imu_calibration_data.json'
    imu.loadCalibDataFromFile(filepath)
    print('Loaded imu calibration file!')
    while not rospy.is_shutdown():
        imu.readSensor()
        imu.computeOrientation()
        full_info = Float64MultiArray(data=np.concatenate((imu.AccelVals, imu.GyroVals, imu.MagVals, np.array([imu.roll, imu.pitch, imu.yaw]))))
        imu_pub.publish(full_info)
        rate.sleep()
        
###############################################################################
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass