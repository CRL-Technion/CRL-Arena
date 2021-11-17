import time

from udp_server import UDPServer
import json
import numpy as np

from Listener import Listener, ListenerType
from planner_controller import PlannerController
from threading import Condition

import mockup


def get_robots_state_to_send(robots_bodies):
    to_send = []

    for robot_body in robots_bodies:
        body_dict = robot_body.to_dict()
        temp_x = body_dict['rotation']['x']

        # need to flip what motive sends us a bit (see documentation)
        body_dict['rotation']['x'] = body_dict['rotation']['w']
        body_dict['rotation']['w'] = body_dict['rotation']['z']
        body_dict['rotation']['z'] = body_dict['rotation']['y']
        body_dict['rotation']['y'] = temp_x
        temp_x = body_dict['position']['x']

        # y-coordinate is also flipped based on observation
        body_dict['position']['x'] = body_dict['position']['y'] * -1
        body_dict['position']['y'] = temp_x
        to_send.append(body_dict)

    sorted_tosend = sorted(to_send, key=lambda k: k['body_id'])
    #sorted_tosend.append(grid.solution_paths_translated)
    #sorted_tosend.append(corners_tosend)
    return sorted_tosend


def main():

    # Create listener to get data from Motive
    listener = Listener(ListenerType.Local)

########################################
    # Mockup listener for offline tests
    # listener = mockup.simple_listener_mock
    # comment out to run online with Motive listener
########################################

    # start the listener
    listener.start()

    # initialize a condition variable for starting to broadcast solution.
    # the variable is being pass to Grid object (from PlannerController)
    # and is notified if the user presses the "Broadcast solution data" button.
    broadcast_solution_cond = Condition()

    # initialize a planner: sets up grid object, draws the arena and allows to run solution planning
    planner_controller = PlannerController(listener=listener, broadcast_cond=broadcast_solution_cond)
    planner_controller.start()

    # if the user presses "Broadcast solution data" button then the program will continue,
    # otherwise it'll wait at this point until shut down
    # TODO: set a different "quit" mechanism
    #  (maybe shut down button that also notifies the variable but move it to a shut down code path)
    with broadcast_solution_cond:
        broadcast_solution_cond.wait()

    # start the server for transmit Motive data via UDP protocol.
    # we use this data to guide the robot (from the Ubuntu computer) and for visualization tools.
    server = UDPServer()
    server.start()

    print("Waiting 2 seconds for the system to stabilized")
    time.sleep(2)
    while True:
        # the filter here is based on a convention -
        # all rigid bodies which represent robots have sequential ids starting from 101
        # TODO: remove this convention and replace with generic solution
        robots_bodies = [body for body in listener.bodies if int(body.body_id) // 100 == 1]
        message = get_robots_state_to_send(robots_bodies)
        server.update_data(json.dumps(message))
        server.send_data()
        time.sleep(0.1)




if __name__ == "__main__":
    main()