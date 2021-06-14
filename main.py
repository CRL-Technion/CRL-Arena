import matplotlib.pyplot as plt

from Listener import Listener, ListenerType

from Grid import Grid

# Create listener
listener = Listener(ListenerType.Local)
grid = Grid(cell_size=1)
grid.plot_init_heatmap()

# start the listener
listener.start()


# stop the listener TODO not working...
# listener.stop()

# the main loop for plotting the environment
while True:
    bodies = listener.bodies
    marker_sets = listener.marker_sets
    grid.reset_grid()
    for ms in marker_sets:
        name = ms.name
        if name == "all":
            continue
        print("Name:", name)
        body_type = "obstacle" if "obstacle" in name.lower() else 'robot'  # TODO should be enum and probably more robust parsing
        markers = ms.positions
        #get list of all cells it touches
        #for obstacles, paint all cells it touches
        #for robots, paint the cells with majority points, and if they're all the same paint it green
        for mi, marker_pos in enumerate(markers):
            loc = [marker_pos.x, marker_pos.y]
            print("location marker #{mi}: ", loc)
            grid.add_coord(loc, body_type)
            cell_loc = grid.xy_to_cell(loc)
            print(cell_loc)
    grid.plot_render()
    # grid.make_map()
    # grid.make_scen()
    plt.pause(1)