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
    kit.motor4.throttle = 0.0

def robot_move(motor1, motor4, delay):
    kit.motor1.throttle = motor1
    kit.motor4.throttle = motor4
    time.sleep(delay)
    robot_stop()

def robot_ir(old_motor1, old_motor_4, adjuster, times, flag):
    if flag == 1:
        if adjuster==0:
            robot_move(1, 1, times)
            #time.sleep(0.001)
        elif adjuster>0:
            robot_move(-adjuster, old_motor_4, times) #try = 0 case
            time.sleep(0.001)
        elif adjuster<0:
            robot_move(old_motor1, adjuster, times) #try = 0 case
            time.sleep(0.001)
    else:
        robot_stop()

#-----------------------------------------------------------------#
# Line tracking code

import math

setupOptiSensor()

# dictRightTurns = {0b00: "a bit left", 0b01: "straight", 0b10: "too left", 0b11: "a bit right"}
dictLeftErrors = {0b00: 1, 0b01: 0, 0b10: 1.5, 0b11: -1}
# dictLeftTurns = {0b00: "a bit right", 0b10: "straight", 0b01: "too right", 0b11: "a bit left"}
dictRightErrors = {0b00: -1, 0b10: 0, 0b01: -1.5, 0b11: 1}
def getErrorRight():
    dataR = getOptiValues(rightPins)
    #print(dictRightTurns[dataR])
    print(dataR)
    error = dictRightErrors[dataR]
    return error

def getErrorLeft():
    dataL = getOptiValues(leftPins)
    #dataL = 0b10
    #print(dictRightTurns[dataL])
    print(dataL)
    error = dictLeftErrors[dataL]
    return error

#Still need to account for case when reach end of line 
#Crossover still needs to be handled 

flag = 1
while True:
    flag = main_robot(flag)
destroy()

def main_robot(flag):
    sampling_rate = 500
    speed = 1
    PID.init(PID, Kp=0.001, Ki=0.001, Kd=0.001)
    output = PID.Update(PID, getErrorRight()+getErrorLeft())
    time.sleep(1/sampling_rate)
    print(output)
    if (pid.gap(pid, 100)):
        flag = 0
        return (flag, 2*math.atan(output)/math.pi)
    robot_ir(speed, speed, 2*math.atan(output)/math.pi*speed, 1/sampling_rate, flag)
    time.sleep(0.001)
    return (flag, 2*math.atan(output)/math.pi)
