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
    return '/home/pi/Desktop/image'+i+'.jpg'


#-----------------------------------------------------------------#
# Sensors code

import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)

rightIRTrackingPinL = 12
rightIRTrackingPinR = 16

leftIRTrackingPinL = 20
leftIRTrackingPinR = 21

leftPins = [leftIRTrackingPinL, leftIRTrackingPinR]
rightPins = [rightIRTrackingPinL, rightIRTrackingPinR]

def setupOptiSensor():
    GPIO.setmode(GPIO.BCM) # Set the GPIO pins as BCM
    # GPIO.setup(IRTrackingPinLL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(IRTrackingPinL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(IRTrackingPinR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPIO.setup(IRTrackingPinRR, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def getOptiValues(pins):
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

def getErrorRight():
    dataR = getOptiValues(rightPins)
    error = 0.0
    if dataR is 0b01:
        print("straight")
        error = 0
    elif dataR is 0b10:
        print("too left")
        error = 1
    elif dataR is 0b00:
        print("a bit left")
        error = 0.5
    elif dataR is 0b11:
        print("a bit right")
        error = -0.5
    else:
        print("invalid data")
        return 0
    return error

def getErrorLeft():
    dataL = getOptiValues(leftPins)
    error = 0.0
    if dataL is 0b10:
        print("straight")
        error = 0
    elif dataL is 0b01:
        print("too right")
        error = -1
    elif dataL is 0b00:
        print("a bit right")
        error = -0.5
    elif dataL is 0b11:
        print("a bit left")
        error = 0.5
    else:
        print("invalid data")
        return 0
    return error

while True:
    pid.init(pid, Kp=0.01, Ki=0.001, Kd=0.001)
    output = pid.Update(pid, getErrorLeft() + getErrorRight)
    time.sleep(0.01)
    print(output)
#-----------------------------------------------------------------#
# Twitter code

#-----------------------------------------------------------------#

from twython import Twython

consumer_key = 'IbZYLMhINCxuxRLd4OyrM2Ph2'
consumer_secret = 'pBfsBvgDYBTjcXuYNgJXl5DhwXNEosZStpjCu4az7SgTvgyMcx'
access_token = '1220402110610604032-0Eca0tLBOjE2TKd1fPbh6BfZzMZ4u2'
access_token_secret = 'GH4fuU3riSkWVh0UGODAasDCI2gN8Qqpc7AkGlQnwzHQZ'

twitter = Twython(
    consumer_key,
    consumer_secret,
    access_token,
    access_token_secret
)

messages = [
    "Hello world",
    "Hi there",
    "What's up?",
    "How's it going?",
    "Have you been here before?",
    "Get a hair cut!",
]

# message = random.choice(messages)
# twitter.update_status(status=message)
# print("Tweeted: %s" % message)
# distance and angle are from sonar servo mount, state is to be defined as a string, imageFile contains
# the name of the photo taken by the camera in a .jpg/.png format
def postTweet(distance, angle, state, imageFile):
    if state == "end":
        message = "I have finished running the track!"
        if distance <= 10:
            message += "There is an object " + str(distance) + " cm away at " \
                       + str(angle) + " degrees from the right. We almost crashed!!"
        elif distance >= 100:
            message += "There are no obstacles in sight! The closest barrier is " \
                       + str(distance) + "cm away at " + str(angle) + " degrees from the right."
        image = open(imageFile, 'rb')
        response = twitter.upload_media(media=image)
        media_id = [response['media_id']]
        twitter.update_status(status=message, media_id=media_id)
        print("Tweeted: " + message)
