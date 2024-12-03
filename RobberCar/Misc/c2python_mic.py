import serial
import time
from time import sleep
# import struct
import RPi.GPIO as GPIO
import cv2
from matplotlib import pyplot as plt
# from timer import Timer
import numpy as np
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import CircularOutput
from collections import deque
import argparse

sampleWindow = 50  #Sample window width in mS (50 mS = 20Hz)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN)
AMP_PIN = GPIO.input(17)       # Preamp output pin connected to A0
# unsigned int sample;

# void setup()
# {
#   Serial.begin(9600);
# }

while True:

  start = time.time() # Start of sample window
  peakToPeak = 0  # peak-to-peak level

  signalMax = 0
  signalMin = 1024


  # collect data for 50 mS and then plot data
  while (time.time() - start)*1000 < sampleWindow:
    sample = AMP_PIN
    if (sample < 1024):  # toss out spurious readings
      if (sample > signalMax):
        signalMax = sample  # save just the max levels
      elif (sample < signalMin):
        signalMin = sample  # save just the min levels

  peakToPeak = signalMax - signalMin  # max - min = peak-peak amplitude
  print(peakToPeak)
  #double volts = (peakToPeak * 5.0) / 1024;  # convert to volts
  #Serial.println(volts);
