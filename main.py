import matplotlib.pyplot as plt

from Listener import Listener, ListenerType
from udp_server import UDPServer
import json
import sys

from Grid import Grid
import time

CELL_SIZE = .5

# Create listener
listener = Listener(ListenerType.Local)
if len(sys.argv) == 2:
    grid = Grid(cell_size = float(sys.argv[1]))
elif CELL_SIZE == None:
    grid = Grid()
else:
    grid = Grid(cell_size = CELL_SIZE)
grid.plot_init_heatmap()

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
    # all_bods = [body.to_dict() for body in bodies]
    to_send = []

    for body in bodies:
        # print(body.body_id)
        if int(body.body_id) // 100 == 1:
            to_send.append(body.to_dict())
    print(to_send)
    server.update_data(json.dumps(to_send))
    grid.reset_grid()
    for ms in marker_sets:
        name = ms.name
        if name == "all":
            continue
        body_type = -1 if "obst" in name.lower() else int(name[name.index('-')+1::])
        markers = ms.positions
        # get list of all cells it touches
        # for obstacles, paint all cells it touches
        # for robots, paint the cells with majority points, and if they're all the same paint it green
        set_coords = []
        for mi, marker_pos in enumerate(markers):
            loc = [marker_pos.x, marker_pos.y]
            set_coords.append(loc)
            # print("location marker #{mi}: ", loc)
        grid.add_body(body_type, set_coords, tolerance=0)
    time.sleep(1)

    grid.plot_render()
    # grid.make_map()
    # grid.make_scen()
    # plt.pause(2)
    time.sleep(1)