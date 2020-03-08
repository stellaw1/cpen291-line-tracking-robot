import RPi.GPIO as GPIO
import time

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
# Motors code
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


def robot_ir(speed, adjuster, times, flag):
    left = speed
    right= speed
    if flag == 1:
        if adjuster==0:
            #time.sleep(0.001)
            robot_move(left, right, times)
           # time.sleep(0.001)
        elif adjuster>0:
            #time.sleep(0.001)
            robot_move(left, -adjuster, times) #try = 0 case
            #time.sleep(0.001)
        elif adjuster<0:
            
            robot_move(+adjuster, right, times) #try = 0 case
            #time.sleep(0.001)
    else:
        robot_stop()


#-----------------------------------------------------------------#
# Line tracking code

from PID import PID as pid
import math

setupOptiSensor()

error = 0
dictRightTurns = {0b00: "a bit left", 0b01: "straight", 0b10: "too left", 0b11: "a bit right"}
dictLeftErrors = {0b00: 0.7, 0b01: 0, 0b10: 1, 0b11: -.7}
dictLeftTurns = {0b00: "a bit right", 0b10: "straight", 0b01: "too right", 0b11: "a bit left"}
dictRightErrors = {0b00: -0.7, 0b10: 0, 0b01: -1, 0b11: .7}

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

flag = 1
while True:
    sampling_rate = 3000
    speed = 0.45
    pid.init(pid, Kp=0.1, Ki=0, Kd=0.001)
    output = pid.Update(pid, getErrorOverall())
    #time.sleep(1/sampling_rate)
    if (gap_count >= 200):
        time.sleep(0.2)
        robot_stop()
        break
    print(output)
    # print(2*math.atan(output)/math.pi*speed)
    robot_ir(speed, 2*math.atan(output)/math.pi*speed, 1/sampling_rate, flag)
    # time.sleep(0.0001)
destroy()
