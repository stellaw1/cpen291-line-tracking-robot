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
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

kit = MotorKit()

def robot_stop():
    kit.motor1.throttle = 0.0
    kit.motor2.throttle = 0.0

def robot_move(motor1, motor2, delay):
    kit.motor1.throttle = motor1
    kit.motor2.throttle = motor2
    time.sleep(delay)
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

def robot_ir(old_motor1, old_motor_2, adjuster, times, flag):
    if flag == 1:
        if adjuster==0:
            robot_move(1, 1, times)
            #time.sleep(0.001)
        elif adjuster>0:
            robot_move(-adjuster, old_motor_2, times) #try = 0 case
            time.sleep(0.001)
        elif adjuster<0:
            robot_move(old_motor1, -adjuster, times) #try = 0 case
            time.sleep(0.001)
    else:
        robot_stop()


#-----------------------------------------------------------------#
# Line tracking code

from PID import PID as pid
import math

setupOptiSensor()

error = 0
dictRightTurns = {0b00: "a bit left", 0b01: "straight", 0b10: "too left", 0b11: "a bit right"}
dictRightErrors = {0b00: 0.7, 0b01: 0, 0b10: 1, 0b11: -.7}
dictLeftTurns = {0b00: "a bit right", 0b10: "straight", 0b01: "too right", 0b11: "a bit left"}
dictLeftErrors = {0b00: -0.7, 0b10: 0, 0b01: -1, 0b11: .7}
def getErrorRight():
    dataR = getOptiValues(rightPins)
    #print(dictRightTurns[dataR])
    print(dataR)
    error = dictLeftErrors[dataR]
    return error

def getErrorLeft():
    dataL = getOptiValues(leftPins)
    #dataL = 0b10
    #print(dictRightTurns[dataL])
    print(dataL)
    error = dictRightErrors[dataL]
    return error

#Still need to account for case when reach end of line 
#Crossover still needs to be handled 

flag = 1
while True:
    sampling_rate = 500
    speed = 1
    pid.init(pid, Kp=0.001, Ki=0.001, Kd=0.001)
    output = pid.Update(pid, getErrorRight()+getErrorLeft())
    time.sleep(1/sampling_rate)
    print(output)
    if (pid.gap(pid, 100)):
        flag = 0
    robot_ir(speed, speed, 2*math.atan(output)/math.pi*speed, 1/sampling_rate, flag)
    time.sleep(0.001)
destroy()
