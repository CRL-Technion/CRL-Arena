import time

from natnet import MotionListener, MotionClient
import requests
from enum import Enum

SERVER_IP = '132.68.36.158'
BROADCAST_IP = '239.255.42.99'


class ListenerType(Enum):
    Local = 0
    Remote = 1


class Listener(MotionListener):
    """
    A class of callback functions that are invoked with information from NatNet server.
    """
    def __init__(self, type=ListenerType.Remote):
        super(Listener, self).__init__()
        self.bodies = []
        self.labeled_markers = []
        self.unlabeled_markers = []
        if type == ListenerType.Local:
            self.client = MotionClient(self, ip_local='127.0.0.1')
        else:
            self.client = MotionClient(self, ip_local=self.get_public_ip(), ip_multicast=BROADCAST_IP, ip_server=SERVER_IP)

    def start(self):
        self.client.get_data()

    def stop(self):
        self.client.disconnect()

    def get_public_ip(self):
        r = requests.get(r'http://jsonip.com')
        return r.json()['ip']

    def on_version(self, version):
        print('Version {}'.format(version))

    def on_rigid_body(self, bodies, time_info):
        # print('RigidBodies {}'.format(bodies))
        self.bodies = bodies

    def on_skeletons(self, skeletons, time_info):
        pass
        # print('Skeletons {}'.format(skeletons))

    def on_labeled_markers(self, markers, time_info):
        # print('Labeled marker {}'.format(markers))
        self.labeled_markers = markers

    def on_unlabeled_markers(self, markers, time_info):
        # print('Unlabeled marker {}'.format(markers))
        self.unlabeled_markers = markers


if __name__ == '__main__':
    # Create listener
    listener = Listener()

    # Create a NatNet client with IP address of your local network interface
    client = MotionClient(listener, ip_local=listener.get_public_ip(), ip_multicast='239.255.42.99', ip_server='132.68.36.158')

    # Data of rigid bodies and markers delivered via listener on a separate thread
    client.get_data()

    # Read version (optional)
    client.get_version()

    # The client continuously reads data until client.disconnect() is called
    time.sleep(5)

    # Stops data stream and disconnects the client
    client.disconnect()
