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
    return '/home/pi/Desktop/image'+i+'.jpg'


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
