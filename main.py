import matplotlib.pyplot as plt

from Listener import Listener, ListenerType

from Grid import Grid

# Create listener
listener = Listener(ListenerType.Local)
grid = Grid()
grid.plot_init_heatmap()

# start the listener
listener.start()


# stop the listener TODO not working...
# listener.stop()

# the main loop for plotting the environment
while True:
    bodies = listener.bodies
    grid.reset_grid()
    for body in bodies:
        print("new body")
        id = body.body_id
        loc = [body.position.x, body.position.y]
        grid.add_coord(loc, id)
        cell_loc = grid.xy_to_cell(loc)
        print(cell_loc)
    grid.plot_render()
    grid.make_map()
    grid.make_scene()
    plt.pause(0.1)