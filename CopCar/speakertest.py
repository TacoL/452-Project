import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(22, GPIO.OUT)
p = GPIO.PWM(22,2000)
p.start(50)
# BELOW IS CHIRP
# for x in range(1000, 5000):
#     p.ChangeFrequency(x)
#     time.sleep(.001)
time.sleep(5)
p.stop()
GPIO.cleanup()