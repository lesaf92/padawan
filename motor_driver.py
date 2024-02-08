import RPi.GPIO as GPIO
from time import sleep

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
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
def left_backward():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
def right_forward():
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
def right_backward():
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
def left_stop():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
def right_stop():
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)

config()
left_backward()
left_forward()
left_stop()

GPIO.cleanup()