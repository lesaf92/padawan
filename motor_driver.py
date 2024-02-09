#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO
from time import sleep

# Left wheel
in1 = 17
in2 = 27
en1 = 22
pwm1 = []
# Right wheel
in3 = 23
in4 = 24
en2 = 18
pwm2 = []

dc_global = 30

def config():
    global in1, in2, in3, in4, en1, en2, pwm1, pwm2
    # Config
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.setup(in3, GPIO.OUT)
    GPIO.setup(in4, GPIO.OUT)
    GPIO.setup(en1, GPIO.OUT)
    GPIO.setup(en2, GPIO.OUT)
    pwm1 = GPIO.PWM(en1, 50)
    pwm2 = GPIO.PWM(en2, 50)
    



def left_forward(dc):
    global in1, in2, in3, in4, en1, en2, pwm1, pwm2
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    pwm1.start(dc)
def left_backward(dc):
    global in1, in2, in3, in4, en1, en2, pwm1, pwm2
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    pwm1.start(dc)
def right_forward(dc):
    global in1, in2, in3, in4, en1, en2, pwm1, pwm2
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    pwm2.start(dc)
def right_backward(dc):
    global in1, in2, in3, in4, en1, en2, pwm1, pwm2
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    pwm2.start(dc)
def left_stop():
    global in1, in2, in3, in4, en1, en2, pwm1, pwm2
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    pwm1.stop()
def right_stop():
    global in1, in2, in3, in4, en1, en2, pwm1, pwm2
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    pwm2.stop()
def vel_left_wheel(vel, weak):
    global dc_global
    dc = dc_global
    if weak:
        dc = dc - 10
    if vel > 0:
        left_forward(dc)
    elif vel < 0:
        left_backward(dc)
    else:
        left_stop()
def vel_right_wheel(vel, weak):
    global dc_global
    dc = dc_global
    if weak:
        dc = dc - 10
    if vel > 0:
        right_forward(dc)
    elif vel < 0:
        right_backward(dc)
    else:
        right_stop()
# Default values for padawan geometry
def inverse_kinematics_diff_drive(vx, wz, L=0.26, r=0.047):
    v_left = (v_x - (L/2)*omega)/r
    v_right = (v_x + (L/2)*omega)/r
    return v_left, v_right

def cmd_vel_cb(msg):
    global dc_global
    vx = msg.linear.x
    wz = msg.linear.z
    v_left, v_right = inverse_kinematics_diff_drive(vx, wz)
    vel_left_wheel(v_left, False)
    vel_right_wheel(v_right, False)
###############################################################################
############################# Main starts here ################################
###############################################################################
def main():
    config()
    rospy.init_node('motor_driver', anonymous=False)
    rate = rospy.Rate(10)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_cb)
    rospy.spin()
    GPIO.cleanup()

######################################################################################################
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        GPIO.cleanup()

