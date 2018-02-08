from Adafruit_BME280 import *
from time import sleep
from datetime import datetime
import RPi.GPIO as GPIO
import serial
from os.path import exists
import picamera
import getpass
import os
import random
import string
import time
import errno
import multiprocessing as mp

GPIO.cleanup()

"""
18 pwm
24 fwd/rev

21 switch
20 switch

26 led
"""

run = True
count = 0
input_state = True
light_state = False
temp = 100
tm_dc = 50
temp_diff = -5

class Timer:
    def __init__(self, interval):
        self.interval = interval
        self.time = None

    def start(self):
        self.time = datetime.now()

    def stop(self):
        self.start = None

    @property
    def check(self):
        return (datetime.now() - self.time).total_seconds()

    @property
    def tdelta(self):
        return self.interval

    def __sub__(self, other):
        return (self.time - other.time)

    def __add__(self, other):
        return (self.time + other.time)

    def __str__(self):
        if self.time is not None:
            return str((datetime.now() - self.time).total_seconds())
        else:
            return "None"

timers = {
    "geiger": Timer(5),
    "bme280": Timer(1),
    "camera": Timer(3),
    "temp": Timer(6)
}

line = "Temp(c),Temp(f),Pressure(hPa),Humdity,time stamp\n"
out = "{},{},{},{},{}\n"
gline = "(count),(time)"
gout = "{},{}\n"



def get_geiger(part):
    if part.isOpen():
        pass

def callback(*args, **kwargs):
    global input_state
    input_state = not input_state
    print("\nInput state changed\nRun: {}\n".format(input_state))

def light_callback(*args, **kwargs):
    global light_state
    light_state = not light_state
    return light_state


if __name__ == "__main__":
    sensor = BME280(t_mode=BME280_OSAMPLE_8, p_mode=BME280_OSAMPLE_8, h_mode=BME280_OSAMPLE_8)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    

    file1 = None
    file2 = None

    c = picamera.PiCamera()
    c.resolution = (1920, 1080)
    c.framerate = 1
    

    for i in range(255):
        if not exists("/home/pi/Desktop/PiCommCode/data%d.xls" % i) or not exists("/home/pi/Desktop/PiCommCode/gdata%d.xls" % i):
            file1 = open("/home/pi/Desktop/PiCommCode/data%d.xls" % i, "a")
            file2 = open("/home/pi/Desktop/PiCommCode/gdata%d.xls" % i, "a")
            break
    else:
        print("Failed to create file, clear out existing files and try agian.")
        exit(-1)
    
    file1.write(line)
    file2.write(gline)

    geiger = serial.Serial()

    # Switch signal pin 21, 
    GPIO.setup(20, GPIO.OUT)
    GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(21, GPIO.RISING, callback=callback, bouncetime=300)

    # LED pin 26
    GPIO.setup(26, GPIO.OUT)

    # Pins for heater control, tm_dc is temp duty cycle
    GPIO.setup(18, GPIO.OUT)
    GPIO.setup(24, GPIO.OUT)

    control = GPIO.PWM(18, tm_dc)
    GPIO.output(24, GPIO.HIGH)
    
    print("Connected to BME280")
    
    print("Attempting to connect to geiger counter")
    for i in range(120):
        try:
            geiger = serial.Serial("/dev/ttyUSB%s" % i)
            break
        except serial.SerialException as error:
            pass
    print("Connection to %s Successful" % geiger.getPort())

    print("Starting processes")
    for i, timer in timers.items():
        timer.start()

    while True:
        try:
            if input_state:
                GPIO.output(20, GPIO.LOW)
                GPIO.output(26, GPIO.HIGH)

                if timers["bme280"].check > timers["bme280"].tdelta:
                    timers["bme280"].start()
                    
                    print("Logging bme280 data")
            
                    degrees = sensor.read_temperature()
                    df = sensor.read_temperature_f()
                    pascals = sensor.read_pressure()
                    hectopascals = pascals / 100
                    humidity = sensor.read_humidity()

                    # Modify pwm to increase/decrease heat as needed
                    heat_needed = temp - df
                    if timers["temp"].check > timers["temp"].tdelta:
                        if heat_needed > temp_diff:
                            control.ChangeFrequency(25)
                            control.ChangeDutyCycle(tm_dc)
                            control.start(18)
                        else:
                            control.stop(18)

                    print 'Temp      = {0:0.3f} deg C'.format(degrees)
                    print 'Temp      = {0:0.3f} deg F'.format(df)
                    print 'Pressure  = {0:0.2f} hPa'.format(hectopascals)
                    print 'Humidity  = {0:0.2f} %'.format(humidity)

                    print out.format(degrees, df, hectopascals, humidity, datetime.now())

                    file1.write(out.format(degrees, df, hectopascals, humidity, datetime.now()))

                    file1.flush()

                if timers["geiger"].check > timers["geiger"].tdelta:
                    try:
                        tdelta = timers["geiger"].check

                        timers["geiger"].start()

                        print("Logging %s secs worth of data" % tdelta)
                        print(gout.format(geiger.inWaiting(), tdelta))

                        file2.write(gout.format(geiger.inWaiting(), tdelta))
                        file2.flush()

                        geiger.flushInput()
                    except AttributeError as e:
                        print("*"*40)
                        print("Failed")
                        print("*"*40)
                    finally:
                        timers["geiger"].start()

                if timers["camera"].check > timers["camera"].tdelta:
                    timers["camera"].start()
                    try:
                        c.capture("/home/pi/Desktop/PiCommCode/images/picture%s.png" % count, format="png")
                        count += 1
                    except picamera.PiCameraError as e:
                        print("PiCamera Error: %s" % e)
            else:
                GPIO.output(20, GPIO.HIGH)
                GPIO.output(26, GPIO.LOW)
                control.stop(18)
        except:
            pass
