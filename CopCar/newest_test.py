#!/usr/bin/env python3
import serial
import time
import RPi.GPIO as GPIO
import cv2
from matplotlib import pyplot as plt
import numpy as np
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import CircularOutput
import argparse
from picamera import PiCamera

Kp = 1
Kd = 0
Ki = 0
prev_error = 0
error_sum = 0

H_val = 15
thold_val = 4

# Add the PiCamera initialization test function
def test_picamera():
    try:
        camera = PiCamera()
        camera.start_preview()
        time.sleep(5)  # Allow the camera to initialize and preview
        camera.stop_preview()
        print("PiCamera initialized successfully.")
    except Exception as e:
        print(f"PiCamera initialization failed: {e}")
        exit(1)  # Exit if the camera initialization fails
    finally:
        camera.close()

# Function definitions
def color_subtract(img, color):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h = img_hsv[:, :, 0].astype(np.int16)
    color = np.int16(color)
    diff = np.abs(h - color).astype(np.int16)
    final_img = diff.astype(np.uint8)
    return final_img

def identify_ball(img):
    h, w = img.shape
    k = np.sum(img)
    if k == 0:
        return (0, 0), 0
    x_idx = np.expand_dims(np.arange(w), 0)
    y_idx = np.expand_dims(np.arange(h), 1)
    x_center = np.int16(np.sum(x_idx * img) / k)
    y_center = np.int16(np.sum(y_idx * img) / k)
    center = (x_center, y_center)
    radius = np.int16(np.sqrt(k / np.pi))
    return center, radius

def contours_localization(img):
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return (0, 0), 0
    largest_contour = max(contours, key=cv2.contourArea)
    (x, y), radius = cv2.minEnclosingCircle(largest_contour)
    center = (np.int16(x), np.int16(y))
    if radius < 50:
        return (0, 0), 0
    else:
        return center, np.int16(radius)

def pid_controller(input):
    if input != 0:
        normalized_input = input / 640
        setpoint = 0.5
        error = setpoint - normalized_input
        P = Kp * error
        right_duty_cycle = ((P * 60) + 50)
        left_duty_cycle = ((-P * 60) + 50)
        left_duty_cycle = max(20, min(80, left_duty_cycle))
        right_duty_cycle = max(20, min(80, right_duty_cycle))
        left_motor.ChangeDutyCycle(left_duty_cycle)
        right_motor.ChangeDutyCycle(right_duty_cycle)
    else:
        left_motor.ChangeDutyCycle(1)
        right_motor.ChangeDutyCycle(1)

# Test PiCamera initialization before proceeding
test_picamera()

# Initialize GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
left_motor = GPIO.PWM(12, 1)
right_motor = GPIO.PWM(13, 1)
left_motor.start(0)
right_motor.start(0)

# Initialize the PiCamera2
picam2 = Picamera2()
video_config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(video_config)
encoder = H264Encoder(1000000, repeat=True)
encoder.output = CircularOutput()
picam2.start()
picam2.start_encoder(encoder)
time.sleep(0.1)

# Main loop
while True:
    frame_start = time.time()  # Start timer for the frame
    image = picam2.capture_buffer("main").reshape((480, 640, 3)).astype(np.uint8)
    difference_img = color_subtract(image, H_val)
    filtered_image = cv2.boxFilter(difference_img, ddepth=-1, ksize=(5, 5))
    _, binary_img = cv2.threshold(filtered_image, thold_val, 255, cv2.THRESH_BINARY_INV)
    center, radius = contours_localization(binary_img)
    x_coordinate, y_coordinate = center
    cv2.circle(image, center, radius, (0, 255, 0), 2)
    cv2.imshow("Image", image)
    cv2.imshow("Binary Image", binary_img)
    pid_controller(x_coordinate)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
picam2.stop_encoder()
