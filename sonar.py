import RPi.GPIO as GPIO
import time

#import Adafruit_DHT

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(12, GPIO.OUT)
pwm = GPIO.PWM(12, 50)
pwm.start(0)

# setting the ports for ultrasonic sensor
TRIG = 26
ECHO = 19


def getSonar():
    # setting the input and output
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
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


# def setAngle(angle, speed):
#     GPIO.setmode(GPIO.BCM)
#     duty = angle / 18 + 2.5
#     GPIO.output(12, True)
#     pwm.ChangeDutyCycle(duty)
#     time.sleep(1 / speed)
#     GPIO.output(12, False)
#     pwm.ChangeDutyCycle(0)


def sendData(speed):
    try:
        for x in range(18):
            angle = x * 10
            # setAngle(angle, speed)
            distance = getSonar()
            print(distance)
            # if distance <= 10 and angle % 20 == 0:
            #     twit.postTweet(distance, angle)
            # elif distance >= 60 and distance <= 75 and angle % 20 == 10:
            #     twit.postTweet(distance, angle)
            # elif distance > 100:
            #     twit.postTweet(distance, angle)
            # if distance <= 15:
            #     loop(2)
            # elif distance <= 30:
            #     loop(3)
            # elif distance <= 45:
            #     loop(4)
            # else:
            #     loop(5)
    except KeyboardInterrupt:
        GPIO.cleanup()
        pwm.stop()

while True:
    print(getSonar())