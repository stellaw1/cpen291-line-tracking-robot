# CPEN 291 B_G11 Project 1
#
# Authors: Manek, Amir, Rain, Parsa, Stella, Arnold, Sanjeev
# Date: February 27th, 2020
#

#-----------------------------------------------------------------#
# import libraries
from picamera import PiCamera
from time import sleep



#-----------------------------------------------------------------#
# 2WD Mobile Platform Code


#-----------------------------------------------------------------#
# RPi code

#-----------------------------------------------------------------#
# Camera code
#
# Source: https://projects.raspberrypi.org/en/projects/getting-started-with-picamera/7

camera = PiCamera()

# image setup
camera.resolution = (500, 300)
camera.annotate_text_size = 30

# function that takes 1 photo, saves it as image-i.jpg. Overwrites previous captures if parameter i is repeated
def capture(i):
    camera.start_preview()
    camera.annotate_text = "B_G11"
    sleep(3)
    camera.capture('/home/pi/Desktop/image%s.jpg' % i)
    camera.stop_preview()


#-----------------------------------------------------------------#
# Sensors code

import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)


#IRTrackingPinLL = 12
IRTrackingPinL = 12
IRTrackingPinR = 16
#IRTrackingPinRR = 15
pins = [IRTrackingPinL, IRTrackingPinR]

def setupOptiSensor():
    GPIO.setmode(GPIO.BCM) # Set the GPIO pins as BCM
    # GPIO.setup(IRTrackingPinLL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(IRTrackingPinL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(IRTrackingPinR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPIO.setup(IRTrackingPinRR, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def getOptiValues():
    value = 0
    for pin in pins:
        value = value << 1
        value = value | GPIO.input(pin)
    #print(value)
    return value

def destroy():
    GPIO.cleanup() # Release resource






#-----------------------------------------------------------------#
# Motors code


#Version 1: motor code (TESTED & WORKING)

# motor1/M2 is RIGHT wheel
# motor2/M1 is LEFT wheel
import time
from adafruit_motorkit import MotorKit
kit = MotorKit()
kit.motor1.throttle = 1.0 # to throttle motor forward
time.sleep(0.5)
kit.motor1.throttle = 0 #to stop the motor


#Version 2: stepper motor (TESTED & BUGGY - motor 2 alternates between rotating forward and backwards when running kit.stepper2.onestep() )
import time
from adafruit_motorkit import MotorKit

kit = MotorKit()

for i in range(100):
    kit.stepper1.onestep()
    time.sleep(0.01)







#-----------------------------------------------------------------#
# Line tracking code

from PID import PID as pid

setupOptiSensor()

error = 0

def getError():
    data = getOptiValues()
    error = 0.0
    if data is 0b01:
        print("straight")
        error = 0
    elif data is 0b10:
        print("too left")
        error = 1
    elif data is 0b00:
        print("a bit left")
        error = 0.5
    elif data is 0b11:
        print("a bit right")
        error = -0.5
    else:
        print("invalid data")
        return 0
    return error
while True:
    pid.init(pid, Kp=0.01, Ki=0.001, Kd=0.001)
    output = pid.Update(pid, getError())
    time.sleep(0.01)
    print(output)