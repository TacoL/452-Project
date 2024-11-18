import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn


i2c = busio.I2C(board.SCL, board.SDA)

ads = ADS.ADS1115(i2c)

ads.gain = 1
chan = AnalogIn(ads, ADS.P0)

while True:
    new_voltage = chan.voltage
    print(new_voltage)