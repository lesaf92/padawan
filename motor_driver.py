import RPi.GPIO as GPIO
from time import sleep

# Left wheel
in1 = 17
in2 = 27
# Right wheel
in3 = 23
in4 = 24
# Config
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)

sleep(3)

GPIO.cleanup()