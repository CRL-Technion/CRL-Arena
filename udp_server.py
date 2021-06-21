import socket
import threading
from threading import Thread
from queue import Queue


class UDPServer:
    def __init__(self):
        self.localIP = "132.68.36.158"
        self.localPort = 20001
        self.bufferSize = 32768
        self.UDPServerSocket = None
        self._data_thread = None
        self._data = None
        self._queue = None

    def start(self):
        self.UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.UDPServerSocket.bind((self.localIP, self.localPort))
        self._queue = Queue()
        self._data_thread = MyThread(self._queue, self)
        self._data_thread.start()
        print("UDP server up and listening")

    def update_data(self, data):
        self._queue.put(data)


print_lock = threading.Lock()


class MyThread(threading.Thread):
    def __init__(self, queue, server, args=(), kwargs=None):
        threading.Thread.__init__(self, args=(), kwargs=None, target=self.run)
        self.queue = queue
        self.server = server
        self.daemon = True

    def run(self):

        # Listen for incoming datagrams
        while True:
            data = self.queue.get()


            bytesAddressPair = self.server.UDPServerSocket.recvfrom(self.server.bufferSize)
            message = bytesAddressPair[0]
            address = bytesAddressPair[1]

            clientMsg = "Message from Client:{}".format(message)
            clientIP = "Client IP Address:{}".format(address)

            # print(clientMsg)
            # print(clientIP)

            self.send_data(data, address)

            # Sending a reply to client
            # msgFromServer = "Hello UDP Client. My Name is Nir."
            # bytesToSend = str.encode(self._data or "")
            # self.server.UDPServerSocket.sendto(bytesToSend, address)

    def send_data(self, data, address):
        print(f"Sending data to {address}")
        bytesToSend = str.encode(data or "")
        self.server.UDPServerSocket.sendto(bytesToSend, address)


