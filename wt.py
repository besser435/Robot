import board
import busio
import time
#import adafruit_ms8607
from adafruit_ms8607 import MS8607
import adafruit_vl53l0x 


i2c = busio.I2C(board.SCL, board.SDA)


sensor = MS8607(i2c)
lid0 = adafruit_vl53l0x.VL53L0X(i2c)
lid0.measurement_timing_budget = 100000 # 200000 is the default, 200ms

lid0 = adafruit_vl53l0x.VL53L0X(i2c)

while True:
    print("Range: {}mm".format(lid0.range))
    print("Pressure: %.2f hPa" % sensor.pressure)
    print("Temperature: %.2f C" % sensor.temperature)
    print("Humidity: %.2f %% rH" % sensor.relative_humidity)
    time.sleep(0.1)
