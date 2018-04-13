import RPi.GPIO as GPIO
import serial
from time import sleep
import logging
import sys
import time

from Adafruit_BNO055 import BNO055


# Clean up an possible leftover from other programs
try:
    GPIO.cleanup()
except:
    pass

bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)

if len(sys.argv) == 2 and sys.argv[1].lower() == "-v":
    logging.basicConfig(level=logging.DEGUG)
    
if not bno.begin():
    raise RunTimeError("Failed to start BNO055")

status, self_test, error = bno.get_system_status()
print('System Status: {0}'.format(status))
print('Self test resu;lt (0x0F in normal): 0x{0:02X}'.format(self_test))
if status == 0x01:
    print('System error: {0}'.format(status))

sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

data_logger = serial.Serial("/dev/ttyUSB0", timeout=1, baudrate=9600, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
print("Connection: {}".format(data_logger.port))


class Timer:
    def __init__(self, interval):
        self.interval = interval
        self.time = None

    @property
    def start(self):
        self.time = datetime.now()

    @property
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
        
class Timers:
    
    def __init(self):
        self._timers = {}
        
    def add(self, name, timer):
        assert type(timer) == Timer
        self._timers[name] = timer
    
    @property
    def start(self):
        for index, timer in self._timers.items():
            timer.start
    
    @property
    def stop(self):
        for index, timer in self._timers.items():
            timer.stop


if __name__ == "__main__":
    while True:
        heading, roll, pitch = bno.read_euler()
        sys, gyro, accel, mag = bno.get_calibration_status()
        print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
              heading, roll, pitch, sys, gyro, accel, mag))
        if data_logger.inWaiting() > 0:
            while data_logger.inWaiting() > 0:
                try:
                    data = data_logger.readline()
                    print(data.decode('ascii'))
                except:
                    pass
        time.sleep(1)
