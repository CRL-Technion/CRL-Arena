import time

from natnet import MotionListener, MotionClient
import requests

SERVER_IP = '132.68.36.158'
BROADCAST_IP = '239.255.42.99'


class Listener(MotionListener):
    """
    A class of callback functions that are invoked with information from NatNet server.
    """
    def __init__(self):
        super(Listener, self).__init__()
        self.bodies = []
        self.labeled_markers = []
        self.unlabeled_markers = []
        self.callback = lambda t, arg1, arg2, arg3: print(f"received new data: {len(arg1)} bodies and {len(arg2+arg3)} markers at time {t}")
        self.data_changed = lambda t: self.callback(t, self.bodies, self.labeled_markers, self.unlabeled_markers)
        self.client = MotionClient(self, ip_local=self.get_public_ip(), ip_multicast=BROADCAST_IP, ip_server=SERVER_IP)

    def set_callback(self, cb):
        self.callback = cb

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
        self.data_changed(time_info.timestamp)

    def on_skeletons(self, skeletons, time_info):
        # print('Skeletons {}'.format(skeletons))
        self.data_changed(time_info.timestamp)

    def on_labeled_markers(self, markers, time_info):
        # print('Labeled marker {}'.format(markers))
        self.labeled_markers = markers
        self.data_changed(time_info.timestamp)

    def on_unlabeled_markers(self, markers, time_info):
        # print('Unlabeled marker {}'.format(markers))
        self.unlabeled_markers = markers
        self.data_changed(time_info.timestamp)


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
