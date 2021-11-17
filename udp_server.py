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
        #self._data_thread = MyThread(self._queue, self)
        #self._data_thread.start()
        print("UDP server up and listening")

        # Listen for incoming datagrams
        # while True:
        #     data = self._queue.get()
        #     bytesAddressPair = self.UDPServerSocket.recvfrom(self.bufferSize)
        #     message = bytesAddressPair[0]
        #     address = bytesAddressPair[1]
        #
        #     clientMsg = "Message from Client:{}".format(message)
        #     clientIP = "Client IP Address:{}".format(address)
        #     self.send_data(data, address)

            # Sending a reply to client
            # msgFromServer = "Hello UDP Client. My Name is Nir."
            # bytesToSend = str.encode(self._data or "")
            # self.server.UDPServerSocket.sendto(bytesToSend, address)

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
                # time.sleep(1)
                print('No data available')
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



# class MyThread(threading.Thread):
#     def __init__(self, queue, server, args=(), kwargs=None):
#         threading.Thread.__init__(self, args=(), kwargs=None, target=self.run)
#         self.queue = queue #for now this is data test
#         self.server = server
#         self.daemon = True
#
#     def run(self):
#
#         # Listen for incoming datagrams
#         while True:
#             data = self.queue.get()
#             bytesAddressPair = self.server.UDPServerSocket.recvfrom(self.server.bufferSize)
#             message = bytesAddressPair[0]
#             address = bytesAddressPair[1]
#
#             clientMsg = "Message from Client:{}".format(message)
#             clientIP = "Client IP Address:{}".format(address)
#             self.send_data(data, address)
#
#             # Sending a reply to client
#             # msgFromServer = "Hello UDP Client. My Name is Nir."
#             # bytesToSend = str.encode(self._data or "")
#             # self.server.UDPServerSocket.sendto(bytesToSend, address)
#
#     def send_data(self, data, address):
#         print(self.queue.qsize())
#         print("data at time of sending: ", data)
#         bytesToSend = str.encode(data or "")
#         self.server.UDPServerSocket.sendto(bytesToSend, address)


