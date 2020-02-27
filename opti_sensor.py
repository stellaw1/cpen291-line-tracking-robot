import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)


IRTrackingPin = 12


def setup():
    GPIO.setmode(GPIO.BCM) # Set the GPIO pins as BCM
    GPIO.setup(IRTrackingPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def loop():
    while True:
        if GPIO.input(IRTrackingPin) == GPIO.LOW:
            print('IR Tracking Test Code')
            print('------------------------------')
            print('The sensor detects white color line')

        else:
            print('IR Tracking Test Code')
            print('------------------------------')
            print('The sensor detects black color line')
        time.sleep(0.2)


def destroy():
    GPIO.cleanup() # Release resource


if __name__ == '__main__': # The Program will start from here
    setup()
try:
    loop()
except KeyboardInterrupt: # When control c is pressed child program destroy() will be executed.
    destroy()