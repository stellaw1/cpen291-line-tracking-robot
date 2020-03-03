import bluetooth

bd_addr = "01:23:45:67:89:AB" #pi's bluetooth address here?

port = 1

sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
sock.connect((bd_addr, port))

sock.send("hello asdf!!")

sock.close()