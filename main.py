import time

import mockup

from Listener import Listener, ListenerType
from planner_controller import PlannerController


def main():

    # Create listener to get data from Motive
    listener = Listener(ListenerType.Local)

########################################
    # Mockup listener for offline tests
    listener = mockup.simple_listener_mock
    # comment out to run online with Motive listener
########################################

    # start the listener
    listener.start()
    # initialize a planner: draws the arena and allows to run solution planning
    planner_controller = PlannerController(listener=listener)
    planner_controller.start()

    # planner_controller.join()


if __name__ == "__main__":
    main()