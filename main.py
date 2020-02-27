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
# Line tracking code


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

#-----------------------------------------------------------------#
# Motors code

#Sample 1---------
import time
from adafruit_motorkit import MotorKit
kit = MotorKit()
kit.motor1.throttle = 1.0
time.sleep(0.5)
kit.motor1.throttle = 0

#Sample 2---------stepper motor
import time
from adafruit_motorkit import MotorKit

kit = MotorKit()

for i in range(100):
    kit.stepper1.onestep()
    time.sleep(0.01)
