import sys
import time
import json
import argparse
import pygame

from src import mockup
from src.arguments_parser import ArgumentsParser

from src.button import Button
from src.globals import SCREENSIZE, LEFT_SCREEN_ALIGNMENT, BUTTON_BASE_FONT_SIZE, GEN_SCENE_BUTTON_COLOR, \
    RUN_PLANNER_BUTTON_COLOR, BROADCAST_BUTTON_COLOR, BASE_WIDTH, BUTTON_BASE_WIDTH, WIDTH, BUTTON_BASE_HEIGHT, \
    BASE_HEIGHT, HEIGHT

from src.udp_server import UDPServer
from src.Listener import Listener, ListenerType
from src.planner_controller import PlannerController


def get_robots_state_to_send(robots_bodies):
    """
    Args:
        robots_bodies: a list of robots' markers positions

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
                grid.init_random_scene()
            elif buttons["goals_from_scene"].is_hover():
                grid.init_goals_from_scene()
            elif buttons["goals_from_file"].is_hover():
                grid.init_goals_from_file()
            elif buttons["run_planner"].is_hover():
                grid.run_planner()
            elif buttons["broadcast"].is_hover():
                grid.broadcast_solution()


def set_buttons(surface, grid_bottom_left):
    """
    Sets a dictionary with buttons to use in the program.
    """
    button_width_scale = BUTTON_BASE_WIDTH / BASE_WIDTH
    button_height_scale = BUTTON_BASE_HEIGHT / BASE_HEIGHT
    buttons_size = (button_width_scale * WIDTH, button_height_scale * HEIGHT)
    font_size_scale = BUTTON_BASE_FONT_SIZE / BASE_WIDTH
    font_size = int(font_size_scale * WIDTH)

    # TODO: changae to relative gap size according to window size
    left_gep = 10
    top_gap = 50

    buttons = {
        "random_scene": Button(text=['Generate Random', 'Goals'],
                               pos=(LEFT_SCREEN_ALIGNMENT, grid_bottom_left + top_gap),
                               size=buttons_size, color=GEN_SCENE_BUTTON_COLOR,
                               surface=surface,
                               font_size=font_size - 2),
        "goals_from_scene": Button(text=['Load Goals', 'from Scene'],
                                pos=(LEFT_SCREEN_ALIGNMENT + buttons_size[0] + left_gep,
                                     grid_bottom_left + top_gap),
                                size=buttons_size, color=GEN_SCENE_BUTTON_COLOR,
                                surface=surface,
                                font_size=font_size),
        "goals_from_file": Button(text=['Load Goals', 'from File'],
                                pos=(LEFT_SCREEN_ALIGNMENT + 2*(buttons_size[0] + left_gep),
                                     grid_bottom_left + top_gap),
                                size=buttons_size, color=GEN_SCENE_BUTTON_COLOR,
                                surface=surface,
                                font_size=font_size),
        "run_planner": Button(text=['Run Planner'],
                                pos=(LEFT_SCREEN_ALIGNMENT + 3*(buttons_size[0] + left_gep),
                                     grid_bottom_left + top_gap),
                                size=buttons_size, color=RUN_PLANNER_BUTTON_COLOR,
                                surface=surface,
                                font_size=font_size),
        "broadcast": Button(text=['Broadcast Solution', 'and Data'],
                                pos=(LEFT_SCREEN_ALIGNMENT + 4*(buttons_size[0] + left_gep),
                                     grid_bottom_left + top_gap),
                                size=buttons_size, color=BROADCAST_BUTTON_COLOR,
                                surface=surface,
                                font_size=font_size - 2)
    }

    return buttons


def main():
    # Parse command-line arguments
    # cell size, goal locations, solver, height, width
    # NOTE: currently, it is not possible to specify a complete scenario or map, and the actual scenario is
    # being generated automatically according to Motive data.
    # It is only possible to specify a pre-defined goal locations.
    parser = argparse.ArgumentParser()
    ap = ArgumentsParser(parser)

    # Create listener to get data from Motive
    listener = Listener(ListenerType.Local)

    # Create a udp server for transmitting the data
    # It'll only be activated later if user pressed the relevant button on screen
    # Remove for tests of path planning side outside of the lab
    server = UDPServer()

    ########################################
    # Mockup listener for offline tests
    # listener = mockup.simple_listener_mock
    # print("RUNNING MOCKUP SCENARIO!")
    # comment out to run online with Motive listener
    ########################################

    # start the listener
    listener.start()

    # init pygame
    pygame.init()
    pygame.display.set_caption('CRL Robots System')
    crl_icon = pygame.image.load('crl_logo.png')
    pygame.display.set_icon(crl_icon)
    surface = pygame.display.set_mode(SCREENSIZE)

    # flag for transmitting solution data. being set to True if 'broadcast solution data' button is pressed.
    broadcast_solution_active = False

    # initialize a planner: sets up grid object, updates it and allows to run solution planning
    planner_controller = PlannerController(arguments_parser=ap, listener=listener, surface=surface)
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

        # if the user presses "Broadcast solution data" button it'll initate the UDP server and start transmitting
        if planner_controller.grid.broadcast_solution_init:
            # set to False so it won't start new server each cycle
            planner_controller.grid.broadcast_solution_init = False

            # start the server for transmit Motive data via UDP protocol.
            # we use this data to guide the robot (from the Ubuntu computer) and for visualization tools.
            server.start()
            print("UDP server initiated, waiting 2 seconds for the system to stabilized...")
            time.sleep(2)

            broadcast_solution_active = True

        # if broadcast has been activated, it'll send the data in each cycle
        if broadcast_solution_active:
            # the filter here is based on a convention -
            # all rigid bodies which represent robots have sequential ids starting from 101
            robots_bodies = [body for body in listener.bodies if int(body.body_id) // 100 == 1]

            # the transmitted message includes (in this order):
            #   - robots positions
            #   - solutions path (if exists, i.e., the planner was executed)
            # we need the additional data (beside the robots) for the arena visualization tool.
            message = get_robots_state_to_send(robots_bodies)
            server.update_data(json.dumps(message))
            server.send_data()
            time.sleep(0.1)


if __name__ == "__main__":
    main()
