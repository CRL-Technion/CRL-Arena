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

        # TODO: allow more argument to build the grid
        # TODO: find out how to get the distance between corners (from motive data) and pass it as x_dim and y_dim
        #  to create a grid with the exact size as the arena.
        #  later need to remove "restrict_arena" and fix coordinates translation everywhere
        self.grid = Grid(cell_size=self.cell_size, plan_filename=plan_filename)

    def get_adjusted_markers_positions(self, marker_set):
        """
        Saves markers from listeners with 4 digits after decimal point
        """
        for i in range(len(marker_set.positions)):
            marker_set.positions[i].x = float("{:.4f}".format(marker_set.positions[i].x))
            marker_set.positions[i].y = float("{:.4f}".format(marker_set.positions[i].y))
            marker_set.positions[i].z = float("{:.4f}".format(marker_set.positions[i].z))
        return marker_set

    def run(self):
        # draw the base empty grid
        #self.grid.plot_init_heatmap()

        # TODO: This is bad practice and exhausting the CPU. Find a different way to run until stopped, maybe use
        #  'asynio' lib or other way of event-looping (with threading, not processes)
        while True:
            # marker_sets = self.listener.marker_sets
            marker_sets = []
            for ms in self.listener.marker_sets:
                marker_sets.append(self.get_adjusted_markers_positions(ms))

            corners = [ms.to_dict() for ms in marker_sets if ms.type == MarkerSetType.Corner]
            obstacles = [ms for ms in marker_sets if ms.type == MarkerSetType.Obstacle]

            # (robot_id, MarkersSet)
            robots = [(ms.name[ms.name.index('-')+1::], ms) for ms in marker_sets if ms.type == MarkerSetType.Robot]

            self.grid.reset_grid()  # TODO: understand how to re-draw (update) without resetting every time
            self.grid.process_corners(corners)  # TODO: only if corners changed
            self.grid.add_obstacles(obstacles)  # TODO: only if obstacles changed
            self.grid.add_robots(robots, tolerance=0)  # TODO: only if robots moved

            time.sleep(1)

            self.grid.plot_render()



