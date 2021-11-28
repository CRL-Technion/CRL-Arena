import sys
import time
import json
import argparse

import pygame

import mockup

from threading import Condition

from arguments_parser import ArgumentsParser
from button import Button
from globals import SCREENSIZE, LEFT_SCREEN_ALIGNMENT, BUTTON_BASE_FONT_SIZE, GEN_SCENE_BUTTON_COLOR, \
    RUN_PLANNER_BUTTON_COLOR, BROADCAST_BUTTON_COLOR
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


def check_events(buttons, grid):
    """
    Checks for events on the screen
    """
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        # checks if a mouse is clicked
        if event.type == pygame.MOUSEBUTTONDOWN:
            # if the mouse is clicked on a - capture which one and activate relevant function
            if buttons["random_scene"].is_hover():
                grid.make_scen()
            elif buttons["goals_from_scene"].is_hover():
                grid.init_from_scene()
            elif buttons["goals_from_file"].is_hover():
                grid.init_from_file()
            elif buttons["run_planner"].is_hover():
                grid.run_planner()
            elif buttons["broadcast"].is_hover():
                grid.broadcast_solution()


def set_buttons(surface, grid_bottom_left):
    """
    Sets a dictionary with buttons to use in the program.
    """
    buttons_size = (110, 70)
    left_gep = 10
    top_gap = 20
    buttons = {
        "random_scene": Button(text=['Make Random', 'Scene'],
                               pos=(LEFT_SCREEN_ALIGNMENT, grid_bottom_left + top_gap),
                               size=buttons_size, color=GEN_SCENE_BUTTON_COLOR,
                               surface=surface,
                               font_size=BUTTON_BASE_FONT_SIZE),
        "goals_from_scene": Button(text=['Load Goals', 'from Scene'],
                                pos=(LEFT_SCREEN_ALIGNMENT + buttons_size[0] + left_gep,
                                     grid_bottom_left + top_gap),
                                size=buttons_size, color=GEN_SCENE_BUTTON_COLOR,
                                surface=surface,
                                font_size=BUTTON_BASE_FONT_SIZE),
        "goals_from_file": Button(text=['Load Goals', 'from File'],
                                pos=(LEFT_SCREEN_ALIGNMENT + 2*(buttons_size[0] + left_gep),
                                     grid_bottom_left + top_gap),
                                size=buttons_size, color=GEN_SCENE_BUTTON_COLOR,
                                surface=surface,
                                font_size=BUTTON_BASE_FONT_SIZE),
        "run_planner": Button(text=['Run Planner'],
                                pos=(LEFT_SCREEN_ALIGNMENT + 3*(buttons_size[0] + left_gep),
                                     grid_bottom_left + top_gap),
                                size=buttons_size, color=RUN_PLANNER_BUTTON_COLOR,
                                surface=surface,
                                font_size=BUTTON_BASE_FONT_SIZE),
        "broadcast": Button(text=['Broadcast Solution', 'and Data'],
                                pos=(LEFT_SCREEN_ALIGNMENT + 4*(buttons_size[0] + left_gep),
                                     grid_bottom_left + top_gap),
                                size=buttons_size, color=BROADCAST_BUTTON_COLOR,
                                surface=surface,
                                font_size=14)
    }

    return buttons


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

    # init pygame
    pygame.init()
    surface = pygame.display.set_mode(SCREENSIZE)

    # initialize a condition variable for starting to broadcast solution.
    # the variable is being pass to Grid object (from PlannerController)
    # and is notified if the user presses the "Broadcast solution data" button.
    broadcast_solution_cond = Condition()
    broadcast_solution = False

    # initialize a planner: sets up grid object, updates it and allows to run solution planning
    planner_controller = PlannerController(arguments_parser=ap, listener=listener,
                                           broadcast_cond=broadcast_solution_cond, surface=surface)
    # set buttons to draw on the screen (to add buttons - modify this method)
    buttons = set_buttons(surface, grid_bottom_left=planner_controller.grid.bottomleft)

    # Main pyGame loop
    while True:
        check_events(buttons, planner_controller.grid)

        # this call takes care of setting every grid-related thing that is being drawn to screen and draw it
        planner_controller.set_grid()

        # draw buttons
        for button_name, button in buttons.items():
            button.show()

        pygame.display.update()

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

    if broadcast_solution is True:
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
