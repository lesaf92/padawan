#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray

import numpy as np
import os
import smbus
import time

from imusensor.MPU9250 import MPU9250
'''
Performs calibration of accelerometer, gyrometer and magnetometer.
Saving the data to file.
'''

###############################################################################
############################# Main starts here ################################
###############################################################################
def main():
    # Initializing the imu class using i2c at address 0x68
    imu = MPU9250.MPU9250(smbus.SMBus(1), 0x68)
    imu.begin()

    # GYRO ---------------------------------
    print('Calibrating gyro...')
    time.sleep(2)
    imu.caliberateGyro()
    print('Done.')
    print('GyroBias')
    print(imu.GyroBias)
    # ACCEL --------------------------------
    print('Calibrating acc...')
    time.sleep(2)
    imu.caliberateAccelerometer()
    print('Done.')
    print('AccelBias')
    print(imu.AccelBias)
    print('Accels')
    print(imu.Accels)
    # MAG ----------------------------------
    print('Calibrating mag...')
    time.sleep(2)
    # imu.caliberateMagApprox()
    imu.caliberateMagPrecise()
    print('Done.')
    print('MagBias')
    print(imu.MagBias)
    print('Magtransform')
    print(imu.Magtransform)
    print('Mags')
    print(imu.Mags)
    filepath = os.path.dirname(os.path.abspath(__file__)) + '/imu_calibration_data.json'
    imu.saveCalibDataToFile(filepath)
    
        
###############################################################################
if __name__ == "__main__":
    main()