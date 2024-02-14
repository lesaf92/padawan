#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray

import sys
import time
import smbus

from imusensor.MPU9250 import MPU9250
'''
print('Calibrating gyro...')
imu.caliberateGyro()
print('Done.')
print('Calibrating acc...')
imu.caliberateAccelerometer()
print('Done.')
'''

# or load your own caliberation file
#imu.loadCalibDataFromFile("/home/pi/calib_real_bolder.json")
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
    while not rospy.is_shutdown():
        imu.readSensor()
        imu.computeOrientation()
        full_info = Float64MultiArray(data=float([imu.AccelVals, imu.GyroVals, imu.MagVals, imu.roll, imu.pitch, imu.yaw]))
        imu_pub.publish(full_info)
        rate.sleep()
        
###############################################################################
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass