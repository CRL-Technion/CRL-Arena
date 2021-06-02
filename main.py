import matplotlib.pyplot as plt

from Listener import Listener, ListenerType
from Visualizer import Visualizer
from MapMaker import MapMaker

# Create listener
listener = Listener(ListenerType.Local)
vis = Visualizer()
vis.initialize_heatmap()

# start the listener
listener.start()

# stop the listener TODO not working...
# listener.stop()

# the main loop for plotting the environment
while True:
    bodies = listener.bodies
    vis.reset_grid()
    for body in bodies:
        print("new body")
        id = body.body_id
        loc = [body.position.x, body.position.y]
        vis.add_coord(loc, id)
        cell_loc = vis.xy_to_cell(loc)
        print(cell_loc)
    vis.render()
    plt.pause(0.1)