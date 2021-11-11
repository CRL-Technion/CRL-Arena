import matplotlib.pyplot as plt

from Listener import Listener, ListenerType
from udp_server import UDPServer
import json
import sys
import numpy
import math

import time

def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    # x = quat.w
    # y = quat.x
    # z = quat.y
    # w = quat.z
    # print("starting with", quat)

    x = float(quat['x'])
    y = float(quat['y'])
    z = float(quat['z'])
    w = float(quat['w'])
    # print("wxyz", w, x, y, z)

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = numpy.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = numpy.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = numpy.arctan2(siny_cosp, cosy_cosp)


    return numpy.degrees(pitch), numpy.degrees(yaw), numpy.degrees(roll)
    return roll, pitch, yaw



# Create listener
listener = Listener(ListenerType.Local)

# start the listener
listener.start()

# start the server for sending Motive data
server = UDPServer()
server.start()

# stop the listener TODO not working...
# listener.stop()

# the main loop for plotting the environment
while True:
    bodies = listener.bodies
    marker_sets = listener.marker_sets
    to_send = []
    corners = []

    for body in bodies:
        if int(body.body_id) // 100 == 1: #if the body is a robot, we want to send its information via UDP
            # print("original quaternoin rotation: ", body.rotation)
            body_dict = body.to_dict()
            temp_x = body_dict['rotation']['x']
            body_dict['rotation']['x'] = body_dict['rotation']['w']
            body_dict['rotation']['w'] = body_dict['rotation']['z']
            body_dict['rotation']['z'] = body_dict['rotation']['y']
            body_dict['rotation']['y'] = temp_x
            # print("euler is:", euler_from_quaternion(body_dict['rotation']))
            temp_x = body_dict['position']['x']
            body_dict['position']['x'] = body_dict['position']['y'] * -1
            body_dict['position']['y'] = temp_x
            # print("x: ", body_dict['position']['x'], "y: ", body_dict['position']['y'])
    # print("what is being sent via UDP: ", to_send)
    sorted_tosend = sorted(to_send, key=lambda k: k['body_id'])
    server.update_data(json.dumps(sorted_tosend))
    time.sleep(0.1)