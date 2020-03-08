import RPi.GPIO as GPIO
import time

#-----------------------------------------------------------------#
# PID controller code

class PID:

    def init(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.Ci = 0

        self.previous_time = time.time()
        self.previous_error = 0
        self.gap_count = 0

    def update(self, error):
        de = error - self.previous_error
        dt = time.time() - self.previous_time
        self.Ci += error * dt

        if dt <= 0.0:
            return 0

        output = self.Kp * error + self.Kd * de/dt + self.Ki * self.Ci

        if (output is 0):
            self.gap_count += 1
        else:
            self.gap_count = 0

        return output

#-----------------------------------------------------------------#
# Sensors code

GPIO.setwarnings(False)

rightIRTrackingPinL = 12
rightIRTrackingPinR = 16

leftIRTrackingPinL = 20
leftIRTrackingPinR = 21

# pins = [leftIRTrackingPinL, leftIRTrackingPinR, rightIRTrackingPinL, rightIRTrackingPinR]
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
# Sonar code

# setting the ports for ultrasonic sensor
TRIG = 26
ECHO = 19

def setupSonar():
    # setting the input and output
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

def getSonar():
    GPIO.output(TRIG, False)
    time.sleep(0.1)
    # time between the pulse 10uS
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    # starting the pulse
    pulse_start = time.time()
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    # ending the pulse
    pulse_end = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    # calculatGPIOn of distance
    sound_speed = 331.5 + (0.6*21)
    pulse_duratGPIOn = pulse_end - pulse_start
    distance = pulse_duratGPIOn * sound_speed * 50
    distance = round(distance, 2)
    return distance
#-----------------------------------------------------------------#




#-----------------------------------------------------------------#
# Twitter code
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

def postTweet(distance, speed, state, imageFile):
    if state == "end":
        message = "I have finished running the track!"
        if distance <= 10:
            message += "There is an object " + str(distance) + " cm away and we are approaching at " \
                       + str(speed) + " m/s. Brace for evasive maneuvers!!"
        elif distance >= 100:
            message += "There are no obstacles in sight! The closest barrier is " \
                       + str(distance) + "cm away and we are approaching at " + str(speed) + " m/s."
        image = open(imageFile, 'rb')
        response = twitter.upload_media(media=image)
        media_id = [response['media_id']]
        twitter.update_status(status=message, media_id=media_id)
        print("Tweeted: " + message)

#-----------------------------------------------------------------#

#-----------------------------------------------------------------#
# Camera code
from picamera import PiCamera
def takePhoto():
    camera = PiCamera()
    camera.capture('/home/pi/Desktop/image%s.jpg' % 1)
    return '/home/pi/Desktop/image%s.jpg' % 1

#-----------------------------------------------------------------#
# #-----------------------------------------------------------------#
# Motors code


#-----------------------------------------------------------------#

from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

kit = MotorKit()

def robot_stop():
    kit.motor1.throttle = 0.0
    kit.motor2.throttle = 0.0

def robot_move(left, right, delay):
    kit.motor2.throttle = left
    kit.motor1.throttle = right
    time.sleep(delay)
    #robot_stop()

factor = 1.1

def robot_ir(speed, adjuster, times, flag, blockade):
    left = speed
    right= speed
    if flag == 1 and blockade == 0:
        if adjuster == 0:
            robot_move(left*factor, right*factor, times)
        elif adjuster > 0:
            robot_move(left, right-adjuster, times)  # try = 0 case
            # time.sleep(0.001)
        elif adjuster < 0:
            robot_move(left+adjuster,right, times)  # try = 0 case
            # time.sleep(0.001)
    elif flag == 0 and blockade == 0:
        robot_stop()
    elif blockade == 1:
        robot_stop()
        time.sleep(1)
        robot_move(-left, -right, time)
        time.sleep(1)



#-----------------------------------------------------------------#
# Line tracking code

from PID import PID as pid
import math

setupOptiSensor()
setupSonar()

error = 0
dictRightTurns = {0b00: "a bit left", 0b01: "straight", 0b10: "too left", 0b11: "a bit right"}
dictLeftErrors = {0b00: 0.7, 0b01: 0, 0b11: -0.7,  0b10: -2}
dictLeftTurns = {0b00: "a bit right", 0b10: "straight", 0b01: "too right", 0b11: "a bit left"}
dictRightErrors = {0b00: -0.7, 0b10: 0, 0b11: 0.7, 0b01: 2}

# dictErrors = {0b0110: 0, 0b1100: -0.7, 0b1110: -1, 0b1111: 0, 0b0111: 1, 0b0011: 0.7, 0b0000: 2}

# def getError():
#     data = getOptiValues(pins)
#     print(data)
#     error =
global gap_count
gap_count = 0

def getErrorRight():
    dataR = getOptiValues(rightPins)
    #print(dictRightTurns[dataR])
    print(dataR)
    error = dictRightErrors[dataR]
    return error, dataR

def getErrorLeft():
    dataL = getOptiValues(leftPins)
    #dataL = 0b10
    #print(dictRightTurns[dataL])
    print(dataL)
    error = dictLeftErrors[dataL]
    return error, dataL

def getErrorOverall():
    errorL, dataL = getErrorLeft()
    errorR, dataR = getErrorRight()
    global gap_count
    if (dataL is 0b00 and dataR is 0b00):
        gap_count += 1
    else:
        gap_count = 0
    return errorL + errorR

#Still need to account for case when reach end of line 
#Crossover still needs to be handled 
lastMove90Right = 0
lastMove90left = 0
flag = 1
try:
    while True:
        sampling_rate = 2000
        speed = 0.4
        pid.init(pid, Kp=0.1, Ki=0, Kd=7)
        output = pid.Update(pid, getErrorOverall())
        #time.sleep(1/sampling_rate)
        if (gap_count >= 100/factor):
            robot_stop()
            break
        print(output)
        # print(2*math.atan(output)/math.pi*speed)
        robot_ir(speed, 2*math.atan(output)/math.pi*speed, 1/sampling_rate, flag, 0)
        # time.sleep(0.0001)
except KeyboardInterrupt:
    robot_stop()
# except:
#     print("IO error")
#     robot_stop()
destroy()

def main_robot(flag):
    sampling_rate = 500
    speed = 1
    PID.init(PID, Kp=0.001, Ki=0.001, Kd=0.001)
    output = PID.Update(PID, getErrorRight()+getErrorLeft())
    time.sleep(1/sampling_rate)
    print(output)

    distance = getSonar()
    blockade = distance <= 25

    if (PID.gap(PID, 100)):
        flag = 0
    robot_ir(speed, speed, 2*math.atan(output)/math.pi*speed, 1/sampling_rate, flag, blockade)
    if blockade:
        imageFile = takePhoto()
        postTweet(distance, speed, "end", imageFile)
    time.sleep(0.001)
    return flag

