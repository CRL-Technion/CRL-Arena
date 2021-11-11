import matplotlib.pyplot as plt

from Listener import Listener, ListenerType
from udp_server import UDPServer
import json
import sys
import os


from Grid import Grid
import time

CELL_SIZE = 0.3

# Create listener
listener = Listener(ListenerType.Local)
if len(sys.argv) == 2:
    grid = Grid(cell_size = float(sys.argv[1]))
elif CELL_SIZE == None:
    grid = Grid(plan_filename="data/algorithm_output")
else:
    grid = Grid(cell_size = CELL_SIZE, plan_filename="data/algorithm_output")
grid.plot_init_heatmap()

# start the Natnet listener (receives data from Motive)
listener.start()

#starts the UDP server (sends data to Ubuntu computer)
server = UDPServer()
server.start()

planner_udp = True #true is planner; false is UDP
udp_freq = 1 #defines sleep period between data transmissions

corners_tosend = []
# the main loop for plotting the environment
to_file = True
while True:
    bodies = listener.bodies  # pull bodies from natnet multicast
    marker_sets = listener.marker_sets  # also pull markersets
    if planner_udp: #if we are in "planner mode"
        if to_file:
            to_file = False
            with open("mock_bodies.txt", 'w') as f:
                f.write(str(listener.bodies))
            with open("mock_markers.txt", 'w') as f:
                f.write(str(listener.marker_sets))
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
            grid.add_body(body_type, set_coords, tolerance=0) #add robots and obstacles
        grid.process_corners(corners)
        corners_tosend = corners
        #time.sleep(1)
        grid.plot_render()
        if grid.has_paths: #if we have planned something and are now ready to UDP broadcast
            print("***SWITCHING TO UDP IN 3 SECONDS***")
            grid.plot_render()
            time.sleep(1)
            planner_udp = False
    else:
        to_send = []
        for body in bodies:
            if int(body.body_id) // 100 == 1:  # if the body IS a robot, we want to send its information via UDP
                body_dict = body.to_dict()
                temp_x = body_dict['rotation']['x']
                body_dict['rotation']['x'] = body_dict['rotation']['w'] #need to flip what motive sends us a bit (see documentation)
                body_dict['rotation']['w'] = body_dict['rotation']['z']
                body_dict['rotation']['z'] = body_dict['rotation']['y']
                body_dict['rotation']['y'] = temp_x
                temp_x = body_dict['position']['x']
                body_dict['position']['x'] = body_dict['position']['y'] * -1 #y-coordinate is also flipped based on observation
                body_dict['position']['y'] = temp_x
                to_send.append(body_dict)
        #sort so that robots are processed in numerical order
        sorted_tosend = sorted(to_send, key=lambda k: k['body_id'])
        sorted_tosend.append(grid.solution_paths_translated)
        sorted_tosend.append(corners_tosend)
        server.update_data(json.dumps(sorted_tosend))
        time.sleep(udp_freq)

