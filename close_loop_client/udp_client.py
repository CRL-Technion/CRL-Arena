import socket
from time import sleep
import json

msgFromClient = "Hello UDP Server"

bytesToSend = str.encode(msgFromClient)

serverAddressPort = ("132.68.36.158", 20001)

bufferSize = 32768

# Create a UDP socket at client side

UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Send to server using created UDP socket
while True:
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)

    msgFromServer = UDPClientSocket.recv(bufferSize)

    msg = "Message from Server {}".format(msgFromServer[0])
    marker_sets = json.loads(bytes.decode(msgFromServer))

    print(marker_sets)
    sleep(1)