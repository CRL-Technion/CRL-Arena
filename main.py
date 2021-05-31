from Listener import Listener, ListenerType

# Create listener
listener = Listener(ListenerType.Remote)


# set a custom callback on the listener
def example_cb(t, bodies, markers, unlabeled_markers):
    print(f'time: {t}')
    print(bodies)


listener.set_callback(example_cb)

# start the listener
listener.start()

# stop the listener TODO not working...
listener.stop()