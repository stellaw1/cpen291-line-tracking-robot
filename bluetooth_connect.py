import bluetooth

target_name = "Amir's S9"
target_address = None

port = 1

while (target_address == None):

    nearby_devices = bluetooth.discover_devices()

    for bdaddr in nearby_devices:
        if target_name == bluetooth.lookup_name( bdaddr ):
            target_address = bdaddr

print("Found target bluetooth device with address ", target_address)

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((target_address, port))

sock.send("hello!")

sock.close()
