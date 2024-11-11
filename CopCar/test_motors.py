import serial
import time
# import struct
import RPi.GPIO as GPIO
import cv2
from matplotlib import pyplot as plt
# from timer import Timer
import numpy as np
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import CircularOutput
import argparse

def pid_controller():
    right_duty_cycle = 80
    left_duty_cycle = 20
    left_motor.ChangeDutyCycle(left_duty_cycle)
    right_motor.ChangeDutyCycle(right_duty_cycle)
    time.sleep(3)
    right_duty_cycle = 20
    left_duty_cycle = 80
    left_motor.ChangeDutyCycle(left_duty_cycle)
    right_motor.ChangeDutyCycle(right_duty_cycle)
    time.sleep(3)


GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
left_motor = GPIO.PWM(12, 50)
right_motor = GPIO.PWM(13, 50)
left_motor.start(0)
right_motor.start(0)

while True:

    pid_controller()

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break
    if key == ord("c"):
        plt.show()