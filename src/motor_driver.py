#!/usr/bin/env python3
'''
ROS node for controlling the motors
This code provides the hardware-level control of the wheels for the PADAWAN robot.

- input: command of body velocities (linear.x and angular.z)
-- topic: /cmd_vel [geometry_msgs/Twist]
- output: motor drive
--
Since the motors do not have any way of measuring rotation, we used fixed PWM based on the signal of the control input.

'''
import rospy
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO

###############################################################################
############################# Global variables ################################
###############################################################################
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

# Global default PWM duty cycle
dc_global = 30
###############################################################################
############################### Aux Functions #################################
###############################################################################
# Initial configuration of GPIO pins
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
    

# Primitive functions to drive motors, separated in left and right
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
# The weak parameter is boolean and should be used if we have composite motion
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
# Inverse kinematics. Used to compute wheel velocity based on body velocity commands
# The default arguments are the values specific for padawan geometry
def inverse_kinematics_diff_drive(vx, wz, L=0.26, r=0.047):
    v_left = (vx - (L/2)*wz)/r
    v_right = (vx + (L/2)*wz)/r
    return v_left, v_right

# Based on the wheel velocities, apply the fixed PWM accordingly
def apply_fixed_pwm(v_left, v_right):
    if (v_left > v_right) and (v_left*v_right > 0):
        vel_left_wheel(v_left, False)
        vel_right_wheel(v_right, True)
    elif (v_left < v_right) and (v_left*v_right > 0):
        vel_left_wheel(v_left, True)
        vel_right_wheel(v_right, False)
    else:
        vel_left_wheel(v_left, False)
        vel_right_wheel(v_right, False)

# Final function to call. It uses all the functions above to move the robot
def move_robot(vx, wz):
    v_left, v_right = inverse_kinematics_diff_drive(vx, wz)
    apply_fixed_pwm(v_left, v_right)

###############################################################################
############################## ROS callbacks ##################################
###############################################################################
def cmd_vel_cb(msg):
    vx = msg.linear.x
    wz = msg.angular.z
    move_robot(vx, wz)
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

###############################################################################
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        GPIO.cleanup()

