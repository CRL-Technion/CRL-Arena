import matplotlib.pyplot as plt

from Listener import Listener, ListenerType
from udp_server import UDPServer
import json
import sys
import numpy
import math

from Grid import Grid
import time

CELL_SIZE = 0.5

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


    return pitch, yaw, roll
    return roll, pitch, yaw



# Create listener
listener = Listener(ListenerType.Local)
if len(sys.argv) == 2:
    grid = Grid(cell_size = float(sys.argv[1]))
elif CELL_SIZE == None:
    grid = Grid(plan_filename="data/algorithm_output")
else:
    grid = Grid(cell_size = CELL_SIZE, plan_filename="data/algorithm_output")
grid.plot_init_heatmap()

# start the listener
listener.start()

server = UDPServer()
server.start()

# stop the listener TODO not working...
# listener.stop()

planner_udp = True #true is planner; false is UDP

# the main loop for plotting the environment
while True:
    if planner_udp:
        bodies = listener.bodies
        marker_sets = listener.marker_sets
        to_send = []
        corners = []
        grid.reset_grid()
        for ms in marker_sets:
            name = ms.name
            if name == "all":
                continue
            elif 'corner' in name.lower():
                corners.append(ms.to_dict())
                continue
            body_type = -1 if "obst" in name.lower() else int(name[name.index('-')+1::])
            markers = ms.positions
            set_coords = []
            for mi, marker_pos in enumerate(markers):
                loc = [marker_pos.x, marker_pos.y]
                set_coords.append(loc)
                # print("location marker #{mi}: ", loc)
            grid.add_body(body_type, set_coords, tolerance=0)
        grid.process_corners(corners)
        time.sleep(1)
        grid.plot_render()
        if grid.has_paths: #if we have planned something
            print("***SWITCHING TO UDP IN 3 SECONDS***")
            grid.plot_render()
            time.sleep(3)
            planner_udp = False
    else:
        bodies = listener.bodies
        marker_sets = listener.marker_sets
        to_send = []
        corners = []
        for body in bodies:
            if int(body.body_id) // 100 == 1:  # if the body is a robot, we want to send its information via UDP
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
                to_send.append(body_dict)
        # print("what is being sent via UDP: ", to_send)
        sorted_tosend = sorted(to_send, key=lambda k: k['body_id'])
        server.update_data(json.dumps(sorted_tosend))
        time.sleep(.1)

