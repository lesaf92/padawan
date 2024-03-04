#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu, MagneticField
import tf

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
    # imu_pub = rospy.Publisher('imu_data', Float64MultiArray, queue_size=10)
    imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
    mag_pub = rospy.Publisher('mag', MagneticField, queue_size=10)

    # Initializing the imu class using i2c at address 0x68
    imu = MPU9250.MPU9250(smbus.SMBus(1), 0x68)
    imu.begin()
    
    filepath = os.path.dirname(os.path.abspath(__file__)) + '/imu_calibration_data.json'
    imu.loadCalibDataFromFile(filepath)
    print('Loaded imu calibration file!')

    while not rospy.is_shutdown():
        imu.readSensor()
        imu.computeOrientation()
        
        t = rospy.Time.now()
        q = tf.transformations.quaternion_from_euler(imu.roll, imu.pitch, imu.yaw)
        imu_msg = Imu()
        imu_msg.header.frame_id = 'imu_frame_id'
        imu_msg.header.stamp = t
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        imu_msg.linear_acceleration.x = imu.AccelVals[0]
        imu_msg.linear_acceleration.y = imu.AccelVals[1]
        imu_msg.linear_acceleration.z = imu.AccelVals[2]
        imu_msg.angular_velocity.x = imu.GyroVals[0]
        imu_msg.angular_velocity.y = imu.GyroVals[1]
        imu_msg.angular_velocity.z = imu.GyroVals[2]

        mag_msg = MagneticField()
        mag_msg.header.frame_id = 'imu_frame_id'
        mag_msg.header.stamp = t
        mag_msg.magnetic_field.x = imu.MagVals[0]
        mag_msg.magnetic_field.y = imu.MagVals[1]
        mag_msg.magnetic_field.z = imu.MagVals[2]

        # full_info = Float64MultiArray(data=np.concatenate((imu.AccelVals, imu.GyroVals, imu.MagVals, np.array([imu.roll, imu.pitch, imu.yaw]))))
        
        imu_pub.publish(imu_msg)
        mag_pub.publish(mag_msg)
        rate.sleep()
        
###############################################################################
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass