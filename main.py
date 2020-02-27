# CPEN 291 B_G11 Project 1
#
# Authors:
# Date:
#

#----------------------#
# import libraries
from picamera import PiCamera
from time import sleep



#----------------------#
# 2WD Mobile Platform Code


#----------------------#
# RPi code


#----------------------#
# Line tracking code


#----------------------#
# Camera code
#
# Source: https://projects.raspberrypi.org/en/projects/getting-started-with-picamera/7
"""
# image setup
camera.resolution = (2592, 1944)
camera.framerate = 15
camera.annotate_text_size = 50
"""
camera = PiCamera()
# take 5 photos
camera.start_preview()
camera.annotate_text = "B_G11"
for i in range(5):
    sleep(5)
    camera.capture('/home/pi/Desktop/image%s.jpg' % i)
camera.stop_preview()
"""
# take a video
camera.start_preview()
camera.start_recording('/home/pi/Desktop/video.h264')
sleep(5)
camera.stop_recording()
camera.stop_preview()

"""

#----------------------#
# Sensors code

