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
    GPIO.setup(leftIRTrackingPinL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(leftIRTrackingPinR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(rightIRTrackingPinL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(rightIRTrackingPinR, GPIO.IN, pull_up_down=GPIO.PUD_UP)


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

import time
from adafruit_motorkit import MotorKit
import adafruit_motor import stepper


kit = MotorKit()

def robot_stop():
    kit.motor1.throttle = 0.0
    kit.motor2.throttle = 0.0

def robot_move(motor1_config, motor2_config, time):
    for i in range(time):
        kit.motor1.throttle = motor1_config
        kit.motor2.throttle = motor2_config
    robot_stop()

def robot_dir(direction, time):
    if direction == "forward":
         robot_move(1.0, 1.0, time)
    elif direction == "backward":
         robot_move(-1.0, -1.0, time)
    elif direction == "left":
         robot_move(0.5, -0.5, time)
    elif direction == "right":
         robot_move(-0.5, 0.5, time)

def robot_ir(old_motor1, old_motor_2, adjuster, time, flag):
    if flag == 1:
        if adjuster==0:
            robot_move(old_motor1, old_motor_2)
        elif adjuster>0:
            robot_move(old_motor1-adjuster, old_motor_2, time)
        elif adjuster<0:
            robot_move(old_motor1, old_motor_2-adjuster, time)
    else:
        robot_stop()

#-----------------------------------------------------------------#
# Line tracking code

from PID import PID as pid
from math import atan

setupOptiSensor()

error = 0
dictRightTurns = {0b00: "a bit left", 0b01: "straight", 0b10: "too left", 0b11: "a bit right"}
dictRightErrors = {0b00: 0.5, 0b01: 0, 0b10: 1, 0b11: -.5}dictRightTurns = {0b00: "a bit left", 0b01: "straight", 0b10: "too left", 0b11: "a bit right"}
dictLeftTurns = {0b00: "a bit right", 0b10: "straight", 0b01: "too right", 0b11: "a bit left"}
dictLeftErrors = {0b00: -0.5, 0b10: 0, 0b01: -1, 0b11: .5}
def getErrorRight():
    dataR = getOptiValues(rightPins)
    print(dictRightTurns[dataR])
    error = dictRightErrors[dataR]
    return error

def getErrorLeft():
    # dataL = getOptiValues(leftPins)
    dataL = 0b10
    print(dictRightTurns[dataL])
    error = dictRightErrors[dataL]
    return error

while True:
    sampling_rate = 100
    speed = 1
    try:
        pid.init(pid, Kp=0.01, Ki=0.001, Kd=0.001)
        output = pid.Update(pid, getErrorLeft() + getErrorRight)
        time.sleep(1/sampling_rate)
        print(output)
        robot_ir(speed, speed, atan(output) + speed, 1, 1)
    except:
        robot_stop()
destroy()


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
