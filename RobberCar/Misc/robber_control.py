#!/usr/bin/env python3
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
from scipy.signal import butter, sosfilt


Kp = 1
Kd = 0
Ki = 0
prev_error = 0
error_sum = 0

prev_left = 0
prev_right = 0

H_val = 15
thold_val = 4

lowest_duty_cycle = 20
highest_duty_cycle = 80

fs = 44100  # Sampling frequency - change
target_freq = 24000

# Function for performing color subtraction
# Inputs:
#   img     Image to have a color subtracted. shape: (N, M, C)
#   color   Color to be subtracted.  Integer ranging from 0-255
# Output:
#   Grayscale image of the size as img with higher intensity denoting greater color difference. shape: (N, M)
def color_subtract(img, color):
    img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    h = img_hsv[:,:,0].astype(np.int16)
    color = np.int16(color)
    diff = np.abs(h - color).astype(np.int16)
    final_img = diff.astype(np.uint8)
    return final_img



# Function to find the centroid and radius of a detected region
# Inputs:
#   img         Binary image
#   USE_IDX_IMG Binary flag. If true, use the index image in calculations; if
#               false, use a double nested for loop.
# Outputs:
#   center      2-tuple denoting centroid
#   radius      Radius of found circle
def identify_ball(img):
    # Find centroid and number of pixels considered valid
    h, w = img.shape

    k = np.sum(img)
    # print(k)
        # print("K is: ", k)
    if k == 0:
    # No orange pixels.  Return some default value
        return (0,0), 0

    # Index image vectors
    x_idx = np.expand_dims(np.arange(w),0)
    y_idx = np.expand_dims(np.arange(h),1)

    # TODO: Calculate the center and radius using the index image vectors
    #       and numpy commands
    x_center = np.int16(np.sum(x_idx * img) / k)
    y_center = np.int16(np.sum(y_idx * img) / k)
    center = (x_center, y_center)
    radius = np.int16(np.sqrt(k / np.pi))
    return center, radius



# Function to find the centroid and radius of a detected region using OpenCV's
# built-in functions.
# Inputs:
#   img         Binary image
# Outputs:
#   center      2-tuple denoting centroid
#   radius      Radius of found circle
def contours_localization(img):

    # TODO: Use OpenCV's findContours function to identify contours in img.
    #       Assume the biggest contour is the ball and determine its center and
    #       radius. Do not forget to deal with the boundary case where no
    #       contours are found.
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return (0,0), 0
    
    largest_contour = max(contours, key=cv2.contourArea)

    (x,y), radius = cv2.minEnclosingCircle(largest_contour)
    center = (np.int16(x), np.int16(y))
    # print(radius)
    if radius < 50:
        return (0,0), 0
    else:
        return center, np.int16(radius)


# proportional and derivative controller limited to 20 to 80% duty cycle
# can change this later
def pid_controller(input, prev_left, prev_right, pwm_array):
    # print(input)
    if input!= 0:
        normalized_input = input / 640
        setpoint = 0.5
        error = setpoint - normalized_input
        P = Kp * error
        # D = Kd * (error - prev_error)
        # prev_error = error
        # error_sum += error
        # I = Ki * error_sum

        right_duty_cycle = ((P * 60) + 50)
        left_duty_cycle = ((-P * 60) + 50)

        left_duty_cycle = max(20, min(80, left_duty_cycle))
        right_duty_cycle = max(20, min(80,right_duty_cycle))

        prev_left = left_duty_cycle
        prev_right = right_duty_cycle

        pwm_array.appendleft(left_duty_cycle)

    else:
        pwm_array.appendleft(0)
        if sum(pwm_array) == 0:
            left_duty_cycle = 0
            right_duty_cycle = 0
        else:
             left_duty_cycle = prev_left
             right_duty_cycle = prev_right
        
    # else:
        # left_motor.ChangeDutyCycle(1)
        # right_motor.ChangeDutyCycle(1)
    return left_duty_cycle, right_duty_cycle, prev_left, prev_right

# Samples the mic and returns the sampled signal
def sampleMic():
    #TODO
    return completeSignal

def create_bandpass_filter(lowcut, highcut, fs, order=4):
    return butter(order, [lowcut, highcut], btype='band', fs=fs, output='sos')

sos = create_bandpass_filter(target_freq - 50, target_freq + 50, fs) # make filter before while loop

def apply_filter(data, sos):

    return sosfilt(sos, data)
    

def audio_scale(left, right, audio_signal):
    # Apply bandpass filter to the audio signal
    filtered_signal = apply_filter(audio_signal)

    # Calculate peak-to-peak amplitude
    signal_max = np.max(filtered_signal)
    signal_min = np.min(filtered_signal)
    peak_to_peak = signal_max - signal_min

    max_amplitude = 1024

    scale_factor = np.clip(peak_to_peak / max_amplitude, 0, 1) # normalize amplitude

    # Scale left and right while maintaining their ratio
    total = left + right
    left_ratio = left / total if total > 0 else 0.5  # Prevent division by zero
    right_ratio = right / total if total > 0 else 0.5

    if(scale_factor > 0):
        new_total = scale_factor * total
        left_out = left_ratio * new_total
        right_out = right_ratio * new_total

    return left_out, right_out

# # serial communication between arduino and raspberry pi
# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
# time.sleep(3)
# ser.reset_input_buffer()
# print("Serial OK")



# initialize gpio pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
left_motor = GPIO.PWM(12, 1)
right_motor = GPIO.PWM(13, 1)
left_motor.start(0)
right_motor.start(0)


# initialize the camera and grab a reference to the raw camera capture

# try:
#     camera = Picamera2()
#     camera.start_preview()
#     sleep(5)
#     camera.stop_preview()
# except Exception as e:
#     print(f"Error: {e}")
# finally:
#     camera.close()

picam2 = Picamera2()
sleep(5)
video_config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(video_config)
encoder = H264Encoder(1000000, repeat=True)
encoder.output = CircularOutput()
picam2.start()
picam2.start_encoder(encoder)
time.sleep(0.1)

pwm_array = deque(maxlen=5)

while True:
    frame_start = time.time() # Start timer for whole frame

    image = picam2.capture_buffer("main").reshape((480,640,3)).astype(np.uint8)

    difference_img = color_subtract(image,H_val)

    filtered_image = cv2.boxFilter(difference_img,ddepth=-1,ksize = (5,5))

    _, binary_img = cv2.threshold(filtered_image,thold_val,255,cv2.THRESH_BINARY_INV)

    # center, radius = identify_ball(binary_img)
    center, radius = contours_localization(binary_img)
    x_coordinate, y_coordinate = center
    cv2.circle(image,center, radius,(0,255,0),2)

    cv2.imshow("Image", image)

    cv2.imshow("Binary Image", binary_img)
    # print(x_coordinate

    left, right, prev_left, prev_right = pid_controller(x_coordinate, prev_left, prev_right, pwm_array)

    audio_signal = sampleMic()

    left, right = audio_scale(left, right, audio_signal)

    left_motor.ChangeDutyCycle(left)
    right_motor.ChangeDutyCycle(right)

    print("Left Duty Cycle:")
    print(left)
    print("Right Duty Cycle:")
    print(right)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break
    if key == ord("c"):
        plt.show()


cv2.destroyAllWindows()
picam2.stop_encoder()


# @reboot /usr/bin/python3 /home/pi/Labs/AutonomousRobbery/first_communication.py > /home/pi/program.log 2>&1