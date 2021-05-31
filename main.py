from Listener import Listener, ListenerType
from Visualizer import Visualizer

# Create listener
listener = Listener(ListenerType.Local)
vis = Visualizer()
vis.render()


# set a custom callback on the listener
def example_cb(t, bodies, markers, unlabeled_markers):
    # add all the bodies to it
    for body in bodies:
        print("new body")
        id = body.body_id
        loc = [body.position.x, body.position.y]
        vis.add_coord(loc, id)
    vis.render()


listener.set_callback(example_cb)

# start the listener
listener.start()

# stop the listener TODO not working...
listener.stop()