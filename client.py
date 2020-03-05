import 
import glob
import time
from bluetooth import *
import math
import P1_FInal as motor

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
	print "Waiting for connection on RFCOMM channel %d" % port

	client_sock, client_info = server_sock.accept()
	print "Accepted connection from ", client_info

	try:
        data = client_sock.recv(1024)
        if len(data) == 0: 
            break
       
        motor_vals = get_data(data)
        speeds = get_speeds(motor_vals)

		left_speed = speeds[0]
        right_speed = speeds[1]

        print(left_speed, right_speed)

        motor.robot_move(left_speed, right_speed, 0.1)
        
	except IOError:
		pass

	except KeyboardInterrupt:
		print "disconnected"
		client_sock.close()
		server_sock.close()
		print "all done"
		break

    def get_data(data):
        tup = tuple(filter(None, data.split(',')))
        return tup

    def get_speeds(data):
        left_speed = 0
        right_speed = 0
        angle = data[0]
        if angle <= 90:
            left_speed = MAX_FORWARD
            right_speed = (angle % 91) / 90 * MAX_FORWARD
        elif angle <= 180:
            right_speed = MAX_FORWARD
            left_speed = ((angle - 90) % 91) / 90 * MAX_FORWARD
        elif angle <= 270:
            right_speed = MAX_BACKWARDS
            left_speed = ((angle - 180) % 91) / 90 * MAX_BACKWARDS
        else:
            left_speed = MAX_BACKWARDS
            right_speed = ((angle - 270) % 91) / 90 * MAX_BACKWARDS
        
        return (left_speed, right_speed)