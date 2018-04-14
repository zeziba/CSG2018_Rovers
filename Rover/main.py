import RPi.GPIO as GPIO
import serial
from time import sleep
import logging
import sys
import time
from datetime import  datetime
import smbus
import struct
import pigpio

# install pigpio for i2c control

from Adafruit_BNO055 import BNO055

if sys.version > '3':
    buffer = memoryview

BUS = 1
RUNTIME = 60.0

# Clean up an possible leftover from other programs
try:
    GPIO.cleanup()
except:
    pass

FWD = 1
REV = 0

SLAVE1 = 0x1
SLAVE2 = 0x2

PI = pigpio.pi()


def command(pi, worker, order, reg=0x00):
    I2C = pi.i2c_open(BUS, worker)
    if I2C > 0:
        pi.i2c_write_byte_data(I2C, reg, order)
    I2C.close(worker)


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
        return self.time - other.time

    def __add__(self, other):
        return self.time + other.time

    def __str__(self):
        if self.time is not None:
            return str((datetime.now() - self.time).total_seconds())
        else:
            return "None"


class Timers:

    def __init(self):
        self.timers = {}

    def add(self, name, interval):
        self.timers[name] = Timer(interval)

    @property
    def start(self):
        for index, timer in self.timers.items():
            timer.start

    @property
    def stop(self):
        for index, timer in self.timers.items():
            timer.stop

    def __getitem__(self, item):
        try:
            return self.timers[item]
        except:
            return -1


class Rover:

    def __init__(self):
        self._sensor_package = None
        self._9dof = None
        self._sensor_package_data = None
        self._9dof_data = {"x": -1000, "y": -1000, "z": -1000, "heading": -1000, "temp1": -1000, "temp2": -1000,
                           "humidity": -1000, }
        self._tasks = Timers()

    def find(self):
        if self._sensor_package_data["heading"] > -1 and self._sensor_package_data["beacon"] > 0:
            l_r = self._sensor_package_data["beacon"] - self._sensor_package_data["heading"]
            if abs(l_r) < 5:
                #Go fwd
                pass
            elif l_r < 0:
                # go right
                pass
            else:
                # go left
                pass
        self._tasks["find"].time.start()

    def start(self):
        self._start_9dof()
        self._start_sensor_package()
        self._tasks.add("find", 0.1)

    def _start_sensor_package(self):
        self._sensor_package = serial.Serial("/dev/ttyUSB0", timeout=1, baudrate=9600, parity=serial.PARITY_NONE,
                                             stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        print("Connection: {}".format(self._sensor_package.port))
        self._tasks.add("sensor_package", 1)

    def _start_9dof(self, port='/dev/ttyAMA0', rst=18):
        self._9dof = BNO055.BNO055(serial_port=port, rst=rst)
        if not self._9dof.begin():
            raise RuntimeError("Failed to initialize BNO055 sensor")
        self._tasks.add("9dof", 1)

    def stop(self):
        self._9dof.close()
        self._sensor_package.close()

    def _9dof_status(self):
        status, self_test, error = self._9dof.get_system_status()
        print('System Status: {0}'.format(status))
        print('Self test resu;lt (0x0F in normal): 0x{0:02X}'.format(self_test))
        if status == 0x01:
            print('System error: {0}'.format(status))

        sw, bl, accel, mag, gyro = self._9dof.get_revision()
        print('Software version:   {0}'.format(sw))
        print('Bootloader version: {0}'.format(bl))
        print('Accelerometer ID:   0x{0:02X}'.format(accel))
        print('Magnetometer ID:    0x{0:02X}'.format(mag))
        print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

    def _9dof_collect(self):
        self._9dof_data = self._9dof.read_euler()

    def _sensor_package_clear_line(self):
        self._sensor_package.flush()

    def set_package_data(self, **kwargs):
        self._sensor_package_data = {"x": kwargs["x"], "y": kwargs["y"], "z": kwargs["z"], "heading": kwargs["heading"],
                                     "temp1": kwargs["temp1"], "temp2": kwargs["temp2"], "humidity": kwargs["humidity"],
                                     "pressure": kwargs["pressure"], "altitude": kwargs["altitude"],
                                     "beacon": kwargs["beacon"]}

    def _sensor_package_gather(self):
        if self._sensor_package.inWaiting() > 0:
            while self._sensor_package.inWaiting() > 0:
                x, y, z, heading, temp1, temp2, humidity, pressure, altitude, beacon = \
                    self._sensor_package.readline().decode("ascii").split(",")
                data = {"x": x, "y": y, "z": z, "heading": heading, "temp1": temp1, "temp2": temp2,
                        "humidity": humidity, "pressure": pressure, "altitude": altitude, "beacon": beacon}
                for key, values in data.items():
                    if values == -1000:

                        data[key] = self._sensor_package_data[key]
                self.set_package_data(**data)
        else:
            return -1


if __name__ == "__main__":
    roverone = Rover()
    while True:
        pass
