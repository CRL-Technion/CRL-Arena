import sys
import time
import json
import argparse

import mockup

from threading import Condition

from arguments_parser import ArgumentsParser
from natnet.protocol import MarkerSetType
from udp_server import UDPServer
from Listener import Listener, ListenerType
from planner_controller import PlannerController


def get_robots_state_to_send(robots_bodies, solution_paths, corners):
    """
    Args:
        robots_bodies: a list of robots' markers positions
        solution_paths: a dict (robot_id, solution) with solution paths
        corners: a list of corners' markers positions

    Returns: a message to send - a list of objects encoded as dictionaries
    """
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
    #sorted_tosend.append(solution_paths)
    #sorted_tosend.append(corners)
    return sorted_tosend


def main():
    # TODO: define and describe demo behavior (for easy run)
    # Parse command-line arguments
    # cell size, goal locations, solver, height, width
    # NOTE: currently, it is not possible to specify a complete scenario or map, and the actual scenario is
    # being generated automatically according to Motive data.
    # It is only possible to specify a pre-defined goal locations.
    parser = argparse.ArgumentParser()
    ap = ArgumentsParser(parser)

    # Create listener to get data from Motive
    listener = Listener(ListenerType.Local)

    ########################################
    # Mockup listener for offline tests
    listener = mockup.simple_listener_mock
    # comment out to run online with Motive listener
    ########################################

    # start the listener
    listener.start()

    # initialize a condition variable for starting to broadcast solution.
    # the variable is being pass to Grid object (from PlannerController)
    # and is notified if the user presses the "Broadcast solution data" button.
    broadcast_solution_cond = Condition()

    # initialize a planner: sets up grid object, draws the arena and allows to run solution planning
    planner_controller = PlannerController(arguments_parser=ap, listener=listener,
                                           broadcast_cond=broadcast_solution_cond)
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

        # duplication - running also under the planner in each iteration
        # TODO: try to eliminate the duplicated calls
        # need to remove 'type' from marker object because it is not serializable
        corners = [ms.to_dict() for ms in listener.marker_sets if ms.type == MarkerSetType.Corner]
        corners = [{k: v for k, v in corner.items() if k != 'type'} for corner in corners]

        # the transmitted message includes (in this order):
        #   - robots positions
        #   - solutions path (if exists, i.e., the planner was executed)
        #   - corners positions
        # we need the additional data (beside the robots) for the arena visualization tool.
        # TODO: maybe find a different way to get the paths which does not include involving
        #  the planner and grid objects (cohesion)
        message = get_robots_state_to_send(robots_bodies, planner_controller.grid.solution_paths_translated, corners)
        server.update_data(json.dumps(message))
        server.send_data()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
