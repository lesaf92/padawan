#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO
from time import sleep

GPIO.cleanup()
# Left wheel
in1 = 17
in2 = 27
# Right wheel
in3 = 23
in4 = 24

def config():
    # Config
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.setup(in3, GPIO.OUT)
    GPIO.setup(in4, GPIO.OUT)

def left_forward():
    global in1, in2, in3, in4
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
def left_backward():
    global in1, in2, in3, in4
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
def right_forward():
    global in1, in2, in3, in4
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
def right_backward():
    global in1, in2, in3, in4
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
def left_stop():
    global in1, in2, in3, in4
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
def right_stop():
    global in1, in2, in3, in4
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)

def cmd_vel_cb(msg):
    if msg.linear.x > 0:
        left_forward()
        right_forward()
    elif msg.linear.x < 0:
        left_backward()
        right_backward()
    else:
        left_stop()
        right_stop()
###############################################################################
############################# Main starts here ################################
###############################################################################
def main():
    rospy.init_node('motor_driver', anonymous=False)
    rate = rospy.Rate(10)
	config()

    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_cb)
    rospy.spin()
    GPIO.cleanup()

######################################################################################################
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        GPIO.cleanup()

