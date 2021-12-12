import ast
import os
import numpy as np
import pygame

from src.Grid import Grid
from natnet.protocol import MarkerSetType

DATA_PATH = "data/"  # TODO: move to shared "util" files for global variables or make a class variable
UBUNTU_DIR = "crl-user@crl-mocap2:/home/crl-user/turtlebot3_ws/src/multi_agent/run/setup_files"
# TODO: move to shared "util" files for global variables or make a class variable


class PlannerController():
    def __init__(self, arguments_parser, listener, surface):
        super(PlannerController, self).__init__()

        self.listener = listener

        self.algorithm_output = DATA_PATH + 'algorithm_output'
        self.scenario_data = DATA_PATH + 'scenario_data'
        self.scene_name = arguments_parser.scene.split('.')[0]  # clean scenario name without .scen suffix
        self.paths_filename = DATA_PATH + self.scene_name + '_paths.txt'
        self.arguments_parser = arguments_parser
        self.SEND_SOLUTION = True

        self.rows = np.floor(self.arguments_parser.height / self.arguments_parser.cell_size)
        self.cols = np.floor(self.arguments_parser.width / self.arguments_parser.cell_size)

        pygame.init()
        self.grid = Grid(cell_size=self.arguments_parser.cell_size,
                         rows=self.rows,
                         cols=self.cols,
                         map_filename=DATA_PATH + self.arguments_parser.map,
                         scene_filename=DATA_PATH + self.arguments_parser.scene,
                         goal_locations=DATA_PATH + self.arguments_parser.goals,
                         algorithm_output=self.algorithm_output,
                         paths_filename=self.paths_filename,
                         surface=surface)
        self.grid.reset_grid()

    def set_grid(self):
        """
        Parses the data from the listener and sets the grid 2D array with relevent values in cells.
        Also calls for pyGame methods to draw the grid.
        """
        marker_sets = []
        for ms in self.listener.marker_sets:
            marker_sets.append(self.get_adjusted_markers_positions(ms))

        obstacles = [ms for ms in marker_sets if ms.type == MarkerSetType.Obstacle]

        # (robot_id, MarkersSet)
        robots = [(ms.name[ms.name.index('-')+1::], ms) for ms in marker_sets if ms.type == MarkerSetType.Robot]

        self.grid.reset_grid()  # TODO: find a way to clean grid objject inplace instead of reset every cycle
        self.grid.add_obstacles(obstacles)  # TODO: only if obstacles changed
        self.grid.add_robots(robots, tolerance=0)  # TODO: only if robots moved

        self.grid.surface.fill((245, 245, 245))  # fill screen background with light-gray color
        self.grid.draw_grid()
        self.grid.place_objects_on_grid()

        # if 'run planner' button is clicked, then running the planner one time
        if self.grid.run_planner_cond:
            self.run_planner()
            self.grid.run_planner_cond = False

        # draw paths to screen after solution was found (currently remains after robots start moving)
        # first time is a bit slow because the program first sends the solution to the second computer
        # before updating the screen
        if self.grid.has_paths:
            self.grid.draw_paths()

    def run_planner(self):
        """
        Running the MAPF planner and sending the solution to ubuntu computer if SEND_SOLUTION flag is turned on
        """
        print("Planner Called")
        os.system(f'wsl ~/CBSH2-RTC/cbs -m {DATA_PATH + self.arguments_parser.map} '
                  f'-a {DATA_PATH + self.arguments_parser.scene} -o test.csv --outputPaths={self.paths_filename} '
                  f'-k {len(self.grid.bots)} -t 60')
        print("Planner finished!")
        self.grid.has_paths = True
        self.paths_to_plan()

        # sending the solution and additional acenario data to ubuntu computer for execution
        if self.SEND_SOLUTION:
            os.system(f'pscp -pw qawsedrf {self.algorithm_output} {UBUNTU_DIR}')  # send solution paths

            with open(self.scenario_data, 'w') as scenario_data_file:
                # prepare scenario data file to be used for automatically running the robots from ubuntu computer.
                # add here additional data required in pre-defined format
                # (need to follow the conventions so it could be parsed).
                robots_ids = [body.body_id for body in self.listener.bodies if int(body.body_id) // 100 == 1]
                robots_ids.sort()
                scenario_data_file.write(f"robots:")  # format: "robots:<id>,<id>..."
                for rid in robots_ids:
                    scenario_data_file.write(f"{rid},")
                scenario_data_file.write("\n")
                scenario_data_file.write(f"cell_size:{self.arguments_parser.cell_size}")  # format: "cell_size:<cell_size>"
            os.system(f'pscp -pw qawsedrf {self.scenario_data} {UBUNTU_DIR}')  # send scenario peripheral data

    def paths_to_plan(self):
        """
        Converts the output of the CBS planner to the input of Hadar's ROS code and saves it in a file by the name of
        self.algorithm_output, to send to ubuntu computer (no other need for this file since the formatted solution is
        saved in _paths.txt file
        """
        paths_file = open(self.paths_filename, "r")
        plan_file = open(self.algorithm_output, "w")

        plan_file.write("schedule:\n")
        all_robots_starts_at_zero_zero = True

        for line in paths_file:
            # get and write agent number
            space_idx = line.index(" ")
            colon_idx = line.index(":")
            agent_id = line[space_idx:colon_idx].replace(" ", "")
            plan_file.write("\tagent" + agent_id + ":\n")

            # get sequence of coordinates
            path_string = line[colon_idx + 2::]
            path_list = path_string.split('->')
            # parse and write out coordinates
            start_location = []
            counter = 0

            path_list_locs = list(filter(lambda loc: loc != '\n' and loc != '', path_list))

            # save paths in class (as tuples not strings)
            clean_path = list(map(lambda loc: ast.literal_eval(loc), path_list_locs))
            self.grid.solution_paths_on_grid[agent_id] = clean_path

            for coord in path_list_locs:
                # this is to compensate for the flipped coordinates that the planner outputs
                y, x = ast.literal_eval(coord)

                if all_robots_starts_at_zero_zero:
                    if not start_location:
                        start_location = [x, y]
                    x = str(int(x) - int(start_location[0]))
                    y = str(-(int(y) - int(start_location[1])))
                plan_file.write('\t\t- x: ' + x + '\n\t\t y: ' + y + '\n\t\t t: ' + str(counter) + '\n')
                counter = counter + 1
        plan_file.close()
        paths_file.close()

    def get_adjusted_markers_positions(self, marker_set):
        """
        Saves markers from listeners with 4 digits after decimal point
        """
        for i in range(len(marker_set.positions)):
            marker_set.positions[i].x = float("{:.4f}".format(marker_set.positions[i].x))
            marker_set.positions[i].y = float("{:.4f}".format(marker_set.positions[i].y))
            marker_set.positions[i].z = float("{:.4f}".format(marker_set.positions[i].z))
        return marker_set

