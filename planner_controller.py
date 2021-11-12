from threading import Thread
from Listener import Listener, ListenerType
import sys
import numpy

from Grid import Grid
import time

from natnet.protocol import MarkerSetType


class PlannerController(Thread):

    def __init__(self, listener, cell_size=0.3, plan_filename='data/algorithm_output'):
        super(PlannerController, self).__init__()

        self.listener = listener
        self.cell_size = cell_size

        # set initial grid
        # TODO: allow more argument to build the grid
        self.grid = Grid(cell_size=self.cell_size, plan_filename=plan_filename)


    def run(self):
        # draw the base empty grid
        self.grid.plot_init_heatmap()

        # TODO: This is bad practice and exhausting the CPU. Find a different way to run until stopped, maybe use
        #  'asynio' lib or other way of event-looping (with threading, not processes)
        while True:
            marker_sets = self.listener.marker_sets
            corners = [ms.to_dict() for ms in marker_sets if ms.type == MarkerSetType.Corner]
            obstacles = [ms for ms in marker_sets if ms.type == MarkerSetType.Obstacle]
            robots = [ms for ms in marker_sets if ms.type == MarkerSetType.Robot]

            self.grid.reset_grid() # TODO: understand how to re draw without resetting every time

            self.grid.add_obstacles(obstacles)
            self.grid.add_robots(robots, tolerance=0)
            self.grid.process_corners(corners)

            time.sleep(1)

            self.grid.plot_render()



