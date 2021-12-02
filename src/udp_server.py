import errno
import socket
import time
from threading import Thread
from queue import Queue


class UDPServer(Thread):
    def __init__(self):
        super(UDPServer, self).__init__()
        self.localIP = "132.68.36.158"
        self.localPort = 20001
        self.bufferSize = 32768
        self.UDPServerSocket = None
        self._data_thread = None
        self._data = None
        self._queue = None

    def run(self):
        self.UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.UDPServerSocket.bind((self.localIP, self.localPort))
        self._queue = Queue()
        print("UDP server up and listening")

    def update_data(self, data):
        print("update: ", data)
        if self._queue.empty():
            self._queue.put(data)
        else:
            self._queue.get()
            self._queue.put(data)

    def send_data(self):
        data = self._queue.get()
        self.UDPServerSocket.setblocking(0)
        try:
            bytesAddressPair = self.UDPServerSocket.recvfrom(self.bufferSize)
        except socket.error as e:
            err = e.args[0]
            if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                # occurs when the client program is not running and nobody tries to pull data
                print('No client side data available - sleeping 1 second')
                time.sleep(1)
            else:
                # a "real" error occurred
                print(e)
        else:
            message = bytesAddressPair[0]
            address = bytesAddressPair[1]

            # print(self._queue.qsize())
            print("data at time of sending: ", data)
            bytesToSend = str.encode(data or "")
            self.UDPServerSocket.sendto(bytesToSend, address)
