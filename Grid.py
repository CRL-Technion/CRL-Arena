import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from statistics import mode
import time
import astar
import os

from matplotlib.widgets import Button

from natnet.protocol import RigidBody, LabeledMarker, Position, Rotation
import random

import itertools
from shapely.geometry import LineString
from enum import Enum

from paths_to_plan_func import paths_to_plan

class CellVal(Enum):
    EMPTY = 0
    ORIGIN = 1
    ROBOT_FULL = 2 #entire robot is on one cell
    ROBOT_PARTIAL = 3 #robot is spread out over multiple cells
    OBSTACLE_REAL = 4
    OBSTACLE_ART = 5 #artificial obstacle


class Grid:
    # A class that holds a grid and can visualize this grid, export it as a map file, or export it as scene file
    def __init__(self, x_dim:int=10, y_dim:int=6, cell_size:float=1.0, map_filename:str='my_map.map', scen_filename:str= 'my_scene.scen', paths_filename:str = 'paths.txt', from_scen = False):
        # arena dimensions (within greater 12x12 scope)
        self.x_dim = min(x_dim, 12)  # m
        self.y_dim = min(y_dim, 12)  # m
        self.cell_size = cell_size  # m
        self.rows = int(12 / cell_size)  # TODO shouldn't this be x_dim instead of 12?
        self.cols = int(12 / cell_size)  # TODO shouldn't this be y_dim instead of 12?
        self.origin_cell = [int(np.floor(self.cols / 2)), int(np.floor(self.rows / 2))]

        # the grid itself with values
        self.grid = []
        self.reset_grid()

        # variables related to matplotlib visualization
        self.fig = None
        self.ax = None
        self.cMap = mpl.colors.ListedColormap([(1,1,1), (0,0,0), (0,1,0), (1,0,0), (0,0,0), (.5,.5,.5)])
        self.heatmap = None
        self.anns = []

        # variables related to exporting map and scene files and path file
        self.mapfile = map_filename
        self.scenfile = scen_filename
        self.pathsfile = paths_filename
        self.endspots = []
        self.has_paths = False


        #association of robots with their ID
        self.bots = {} #maps bot IDs to current spot
        self.end_bots = {} #maps bot ID's to end spot (if one exists)
        self.bad_bots = [] #simple list of all robots that aren't completely on one cell
        self.out_of_bounds_bots = [] #simple list of all robots that aren't completely within the bounds of the arena
        self.bot_boxes = [] #all the text boxes representing robots
        self.end_boxes = [] #all the text boxes representing the robots' end locations


    def reset_grid(self):
        # reset the grid so that all values are 0 (meaning nothing is in the box)
        self.grid = []
        for i in range(int(self.rows)):
            self.grid.append([CellVal.EMPTY.value for i in range(int(self.cols))])
        self.bots = {}
        self.bad_bots = []

    def getBlockedCells(self, vertices_list, dr=0.01): #TODO: make it so that this only uses the outer vertices??? waste of time if there's a body with a lot of inner markers
        blocked_cells = []
        for pair in itertools.product(vertices_list, repeat=2):
            line_blocked_cells = self.lineGridIntersection(pair[0], pair[1], dr)
            # blocked_cells.append(line_blocked_cells)
            blocked_cells += line_blocked_cells
        return list(set(blocked_cells))

    def add_body(self, type, body_coords, tolerance=1):
        #a word on tolerance: it describes how "strict" the system will be with requiring a robot to be in one cell in order to count it as a robot
        #   tolerance of 0: zero tolerance, all of a robot's markers must be in one cell
        #   tolerance of 1: all markers but one should be in the same cell
        #   tolerance of 2: higher flexibility, just goes with the majority\
        #if the robot's configuration is outside of the specified tolerance, it will highlight all the cells the robot touches
        # if type is obstacle, then color all the cells it touches
        if type == -1: #obstacle
            blocked_cells = self.getBlockedCells(body_coords)
            for coord in blocked_cells:
               self.grid[coord[0]][coord[1]] = CellVal.OBSTACLE_REAL.value
        # if type is robot, then if it's mostly concentrated on one cell color it, if it's spread out color all of them
        else: #robot
            relevant_cells = [self.xy_to_cell(coord) for coord in body_coords]
            #check if the robot is out of bounds
            in_bounds = True
            for cell in relevant_cells:
                x = int(self.x_dim / self.cell_size)
                y = int(self.y_dim / self.cell_size)
                if (cell[0] > (self.origin_cell[0] + y // 2) or cell[0] < (self.origin_cell[0] - y // 2)) or cell[1] > (self.origin_cell[1] + x // 2 or cell[1]) < (self.origin_cell[1] - x // 2):
                    self.out_of_bounds_bots.append(type)
                    in_bounds = False
                    break
            if in_bounds:
                mode_cell = mode(relevant_cells)
                majority_count = len(relevant_cells) / 2
                if tolerance == 0 and relevant_cells.count(mode_cell) == len(relevant_cells):#all cells are in one
                    self.grid[mode_cell[0]][mode_cell[1]] = CellVal.ROBOT_FULL.value
                    self.bots[type] = [mode_cell[0], mode_cell[1]]
                elif tolerance == 1 and relevant_cells.count(mode_cell) >= len(relevant_cells) - 1:#all cells but one are in the same cell
                    self.grid[mode_cell[0]][mode_cell[1]] = CellVal.ROBOT_FULL.value
                    self.bots[type] = [mode_cell[0], mode_cell[1]]
                elif tolerance == 2 and relevant_cells.count(mode_cell) >= majority_count:#majority cells are in the same cell
                    self.grid[mode_cell[0]][mode_cell[1]] = CellVal.ROBOT_FULL.value
                    self.bots[type] = [mode_cell[0], mode_cell[1]]
                else:
                    self.bad_bots.append(type) #add its id to the list of bad robots
                    # highlight all the cells it touches
                    for cell in relevant_cells:
                        self.grid[cell[0]][cell[1]] = CellVal.ROBOT_PARTIAL.value

    def lineGridIntersection(self, p1, p2, dr):
        ls = LineString([p1, p2])
        points_on_line = []
        line_length = np.ceil(ls.length)
        num_samples = int(line_length / dr)
        linespace = [x * dr for x in range(0, num_samples + 1)]
        for f in linespace:
            p = ls.interpolate(f).coords[0]

            points_on_line.append(p)

        ar = np.array(points_on_line, 'f')

        cells = list(map(lambda p: self.xy_to_cell(p), ar))
        return cells

    def restrict_arena(self):
        # goal: place artificial obstacles around the perimeter of our desired arena
        x = int(self.x_dim / self.cell_size)
        y = int(self.y_dim / self.cell_size)
        # corners
        top_right = [(self.origin_cell[0] - y // 2), self.origin_cell[1] + x // 2]
        bottom_right = [(self.origin_cell[0] + y // 2), self.origin_cell[1] + x // 2]
        top_left = [(self.origin_cell[0] - y // 2), self.origin_cell[1] - x // 2]
        bottom_left = [(self.origin_cell[0] + y // 2), self.origin_cell[1] - x // 2]
        corners = [top_left, top_right, bottom_right, bottom_left]
        borders = corners
        for i in range(top_left[1], top_right[1]): # top border
            borders.append([top_right[0], i])
        for i in range(bottom_left[1], bottom_right[1]): # bottom border
            borders.append([bottom_right[0], i])
        for i in range(top_left[0], bottom_left[0]): # left border
            borders.append([i, bottom_left[1]])
        for i in range(top_right[0], bottom_right[0]): # right border
            borders.append([i, bottom_right[1]])
        # indicate all relevant cells in the grid
        for cell in borders:
            self.grid[cell[0]][cell[1]] = CellVal.OBSTACLE_ART.value

    def plot_init_heatmap(self):
        # initialize heatmap to be used to display the plot; should be called before plot_render()
        self.fig, self.ax = plt.subplots(1, 1)
        bounds = range(self.cMap.N)
        norm = mpl.colors.BoundaryNorm(bounds, self.cMap.N)
        data = self.grid
        self.heatmap = self.ax.pcolor(data, edgecolors='k', linewidths=1, cmap=self.cMap, vmin=0, vmax=5)
        self.fig.canvas.draw()
        plt.gca().invert_yaxis()
        self.fig.show()

    def init_from_scene(self, event=None):
        #takes an existing .scen file and initializes end spots from it
        scen_file = open(self.scenfile, 'r')
        for line in scen_file:
            if line[0] == 'v':
                continue
            data = line.split('\t')
            if int(data[0]) in self.bots:
                self.end_bots[int(data[0])] = [int(data[7]), int(data[6])]

    def plot_render(self):

        if len(self.out_of_bounds_bots) > 0:
            print("the following robots are out of bounds and will not be shownin the visualization: ", self.out_of_bounds_bots)
        if self.heatmap == None:
            self.plot_init_heatmap()
        # re-plot grid with up-to-date values; should be called after updating/adding values
        self.restrict_arena()
        data = self.grid
        # color origin cell
        # data[self.origin_cell[0]][self.origin_cell[1]] = CellVal.ORIGIN.value



        bounds = range(self.cMap.N)
        norm = mpl.colors.BoundaryNorm(bounds, self.cMap.N)
        self.heatmap = self.ax.pcolormesh(data, edgecolors='k', linewidths=1, cmap=self.cMap, vmin=0, vmax=5)

        if self.has_paths:
            for ann in self.anns:
                ann.remove()
            self.anns = []
            #first, open paths file
            # pathsfile = open('paths4.txt', 'r')
            pathsfile = open(self.pathsfile, 'r')

            #for each line, parse it and create a list of the coordinates
            for line in pathsfile:
                colon_idx = line.index(":")
                path_string = line[colon_idx + 2::]
                paths_list = path_string.split('->')
                #draw lines between each two
                paths_clean = []
                for coord in paths_list:
                    if coord == '\n' or coord == '':
                        paths_list.remove(coord)
                    else:
                        clean = coord[1:-1]
                        clean = clean.split(',')
                        paths_clean.append(clean)
                for i in range(len(paths_clean) - 1):
                    ann = self.ax.annotate("", xy = (int(paths_clean[i][1]) + 0.5,int(paths_clean[i][0]) + 0.5), xytext=(int(paths_clean[i+1][1]) + 0.5,int(paths_clean[i+1][0])+0.5), arrowprops=dict(arrowstyle='-', connectionstyle='arc3'))
                    # print("annotating from", (int(paths_clean[i][1]) + 0.5,int(paths_clean[i][0]) + 0.5), "to ", (int(paths_clean[i+1][1]) + 0.5,int(paths_clean[i+1][0])+0.5) )
                    self.anns.append(ann)

        for key, value in self.bots.items():
            new = True
            for text in self.bot_boxes:
                if plt.getp(text, 'text') == str(key):
                    new = False
                    text.set_position((value[1] + 0.5, value[0] + 0.5))
            if new:
                newtxt = self.ax.text(value[1] + 0.5, value[0] + 0.5, str(key), size=12 * self.cell_size, ha="center", va="center", bbox=dict(ec=(0, 0, 0),  boxstyle='circle', fc=(0, 1, 0)))
                self.bot_boxes.append(newtxt)
        for key, value in self.end_bots.items():
            new = True
            for text in self.end_boxes:
                if plt.getp(text, 'text') == str(key):
                    new = False
                    text.set_position((value[1] + 0.5, value[0] + 0.5))
            if new:
                newtxt = self.ax.text(value[1] + 0.5, value[0] + 0.5, str(key), size=12 * self.cell_size, ha="center", va="center", bbox=dict(ec=(0, 0, 0),  boxstyle='circle', fc=(1, .8, 0.8)))
                self.end_boxes.append(newtxt)
        axplan = plt.axes([0.46, -0.01, 0.1, 0.075])
        axinit = plt.axes([0.58, -0.01, 0.1, 0.075])
        axprev = plt.axes([0.7, -0.01, 0.1, 0.075])
        axnext = plt.axes([0.81, -0.01, 0.1, 0.075])
        bnext = Button(axnext, 'Map')
        bnext.on_clicked(self.make_map)
        bprev = Button(axprev, 'Scenario')
        bprev.on_clicked(self.make_scen)
        binit = Button(axinit, 'Load')
        binit.on_clicked(self.init_from_scene)
        bplan = Button(axplan, 'Plan')
        bplan.on_clicked(self.run_planner)

        self.ax.draw_artist(self.ax.patch)
        self.ax.draw_artist(self.heatmap)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()
        #for each robot, check if there's already a text box. if there is, change it. if there isn't, add one
        # for text in self.textboxes:
        #     # print("new text that says", text.get_s())
        #     text.set_visible(False)
        #     print(plt.getp(text, 'text'))
        # # for key, value in self.end_bots.items():
        #     # plt.text(-6.53 + .65*(value[0]+1), 11.05 - 0.86*(value[1]-1), str(key), size=13 * self.cell_size, ha="center", va="center", bbox=dict(ec=(0, 0, 0),  boxstyle='circle', fc=(1., 0.8, 0.8), ))
        #     newtxt = plt.text(-6.53 + .65*(value[1]), 11.05 - 0.86*(value[0]), str(key), size=12 * self.cell_size, ha="center", va="center", bbox=dict(ec=(0, 0, 0),  boxstyle='circle', fc=(1., 0.8, 0.8)))
        #     self.textboxes.append(newtxt)


        # t_end = time.time()
        plt.pause(0.2)

    def xy_to_cell(self, loc):
        # convert x and y from Motive to new coordinate system
        x = -loc[0]
        y = -loc[1]
        return (int(self.origin_cell[0] + np.round(x / self.cell_size)), int(self.origin_cell[1] + np.round(y / self.cell_size)))

    def run_planner(self, plan=True, event=None):
        #make plan parameter false if you only want to export the paths file (and don't want to convert to plan)
        os.system('wsl ~/CBSH2-RTC/cbs -m {0} -a {1} -o test.csv --outputPaths={2} -k 2 -t 60'.format(self.mapfile, self.scenfile, self.pathsfile))
        self.has_paths = True
        paths_to_plan(paths=self.pathsfile)

    def make_map(self, event=None):
        f = open(self.mapfile, "w")
        f.write("type octile\n")
        f.write("height " + str(self.rows) + '\n')
        f.write("width " + str(self.cols) + '\n')
        f.write("map\n")
        for i in range(int(self.rows)):
            for j in range(int(self.cols)):
                if self.grid[i][j] == CellVal.OBSTACLE_ART.value or self.grid[i][j] == CellVal.OBSTACLE_REAL.value:
                    f.write('@')
                else:
                    f.write('.')
            f.write('\n')
        f.close()

    def make_scen(self, event=None):
        #first, warn the user that the scene file is incomplete if there are robots that aren't in cells
        if len(self.bad_bots) > 0:
            print("Robots with the following ID's are not aligned with a single cell and won't be included in the .SCEN file: ", self.bad_bots)
        # for each ROBOT on the grid (meaning its grid value is 1), make a line with all its info
        f = open(self.scenfile, "w")
        f.write("version 1\n")
        count = 0
        self.end_bots = {}
        for key, value in self.bots.items():
            count+=1
            # bucket
            f.write(str(key)+'\t')
            # .map file name
            f.write(str(self.mapfile) + '\t')
            # dimensions of the grid
            f.write(str(int(self.rows)) + '\t' + str(int(self.cols)) + '\t')
            # starting position
            f.write(str(value[1]) + '\t' + str(value[0]) + '\t')
            # ending position
            x, y = self.get_empty_spot()
            self.end_bots[key] = [y, x]
            f.write(str(x) + '\t' + str(y) + '\t')
            # optimal distance
            f.write("\n")
            # f.write(f'{self.get_optimal_length((i, j), (x, y))}\n')
        f.close()
        self.has_paths = False
        print(".scen file generated")


    def get_empty_spot(self):
        x = int(self.x_dim / self.cell_size)
        y = int(self.y_dim / self.cell_size)
        try_x = -1
        try_y = -1
        while True: #choose within borders of arena, not the 12x12
            try_x = random.randint((self.origin_cell[1] - x // 2)+1, self.origin_cell[1] + x // 2)
            try_y = random.randint((self.origin_cell[0] - y // 2)+1, (self.origin_cell[0] + y // 2))
            if self.grid[try_y][try_x] != 0:
                continue
            for coord in self.endspots: # now need to make sure no two ending spots align
                if coord[1] == try_x and coord[0] == try_y:
                    continue
            break
        #print(try_x, try_y)
        self.endspots.append([try_y, try_x])
        return try_x, try_y

    def get_optimal_length(self, loc1, loc2):
        path = list(astar.find_path(loc1, loc2,
                                    neighbors_fnct=lambda loc: self.neighbors(loc, True),
                                    heuristic_cost_estimate_fnct=self.heuristic,
                                    distance_between_fnct=self.distance))

        dist = 0
        prev = path[0]
        for p in path[1:]:
            dist += self.distance(prev, p)
            prev = p

        return dist

    def neighbors(self, loc, diagonal_moves=False):
        moves = [
            np.array([0, 1]),
            np.array([1, 0]),
            np.array([0, -1]),
            np.array([-1, 0])
        ]
        if diagonal_moves:
            moves += [
                np.array([1, 1]),
                np.array([-1, 1]),
                np.array([1, -1]),
                np.array([-1, -1])
            ]
        neighbors = []
        for move in moves:
            new_loc = np.array(loc) + move
            if 0 <= new_loc[0] <= self.cols-1 and 0 <= new_loc[1] <= self.rows-1 and self.grid[new_loc[0]][new_loc[1]] == 0:
                neighbors.append(tuple(new_loc))

        return neighbors

    def heuristic(self, loc, goal):
        return np.linalg.norm(np.array(loc)-np.array(goal), 2)

    def distance(self, loc1, loc2):
        return np.linalg.norm(np.array(loc1)-np.array(loc2), 2)


if __name__ == "__main__":
    # Testing the optimal length function
    env = Grid(12, 12)
    m, n = env.rows, env.cols
    start_x = np.random.randint(0, m)
    start_y = np.random.randint(0, n)
    goal_x = np.random.randint(0, m)
    goal_y = np.random.randint(0, n)

    print(env.get_optimal_length((start_x, start_y), (goal_x, goal_y)))


