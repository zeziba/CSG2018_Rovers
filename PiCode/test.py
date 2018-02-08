import RPi.GPIO as GPIO
from time import sleep
GPIO.cleanup()

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(16, GPIO.OUT)

def callback(*args, **kargs):
    print("Event")

GPIO.add_event_detect(18, GPIO.RISING, callback=callback)


while True:
    sleep(.005)
