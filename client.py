import glob
import time
from bluetooth import *
import math

from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

kit = MotorKit()

def robot_stop():
    kit.motor1.throttle = 0.0
    kit.motor2.throttle = 0.0

def robot_move(left, right):
    kit.motor2.throttle = left
    kit.motor1.throttle = right
    #time.sleep(delay)
    #robot_stop()

MAX_FORWARD = 1
MAX_BACKWARDS = -1

server_sock=BluetoothSocket( RFCOMM )
server_sock.bind(("",PORT_ANY))
server_sock.listen(1)

port = server_sock.getsockname()[1]

uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

advertise_service( server_sock, "LineTrackerServer",
                   service_id = uuid,
                   service_classes = [ uuid, SERIAL_PORT_CLASS ],
                   profiles = [ SERIAL_PORT_PROFILE ],
                   )
while True:
    print("Waiting for connection on RFCOMM channel ", port)

    client_sock, client_info = server_sock.accept()
    print ("Accepted connection from ", client_info)

    while True:
        try:
            data = client_sock.recv(1024)

            if len(data) == 0:
                print("no data")
                break

            direction = data.decode(encoding='UTF-8')

            if direction == 'Forward':
                robot_move(1, 1)
            elif direction == 'Backward':
                robot_move(-1, -1)
            elif direction == 'Left':
                robot_move(0.5,1)
            elif direction == 'Right':
                robot_move(1,0.5)
            elif direction == 'Stop':
                robot_stop()

        except IOError:
            break
        except KeyboardInterrupt:
            print("disconnected")
            client_sock.close()
            server_sock.close()
            print("all done")
            break
