import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import astar
import os
import random
import itertools

from enum import Enum
from statistics import mode
from matplotlib.colors import ListedColormap
from matplotlib.widgets import Button
from shapely.geometry import LineString


class CellVal(Enum):
    # represents the values that populate the grid (and match with the colors that appear on the grid)
    EMPTY = 0
    COLLISION = 1  #a robot and a real obstacle occupy the same cell
    ROBOT_FULL = 2  #entire robot is on one cell
    ROBOT_PARTIAL = 3  #robot is spread out over multiple cells
    OBSTACLE_REAL = 4
    OBSTACLE_ART = 5  #artificial obstacle


class Corner(Enum):
    # values associated with corner markers
    TOPLEFT = 1
    TOPRIGHT = 2
    BOTTOMLEFT = 3
    BOTTOMRIGHT = 4


# TODO: Clean the entire class code - remove unnecessary lines and methods and simplify logic
class Grid:
    """
    This class is the visual representation of the arena, including robots, obstacles, and borders.
    A class that holds a grid and can visualize this grid, export it as a map file, or export it as scene file.
    """
    def __init__(self, x_dim:int=10, y_dim:int=6, cell_size:float=1.0, map_filename:str='data/map.map',
                 scen_filename:str= 'data/scenario.scen', end_locations_filename:str = 'data/end_locations.txt',
                 paths_filename:str = 'data/paths.txt', plan_filename:str='data/plan.txt',
                 ubuntu_dir:str="crl-user@crl-mocap2:/home/crl-user/turtlebot3_ws/src/multi_agent"):
        """
        x_dim: in meters, size of overall arena; maximum 12
        y_dim: in meters, size of overall arena; maximum 12
        cell_size: in meters
        map_filename: name of .map file that is output
        scen_filename: name of .scen file that is output
        end_locations_filename: name of .txt file containing each robot's end location
        paths_filename: name of .txt file where planner will output paths
        plan_filename: name of .txt file to be sent to ubuntu
        ubuntu_dir: name of directory in the ubuntu computer to send plan
        """

        # arena dimensions (within greater 12x12 scope)
        self.x_dim = min(x_dim, 12)  # m
        self.y_dim = min(y_dim, 12)  # m
        self.corners = {}

        self.cell_size = cell_size  # m
        # rows and columns are like the larger grid, while x and y dim are the smaller arena
        self.rows = int(12 / cell_size)
        self.cols = int(12 / cell_size)
        self.origin_cell = [int(np.floor(self.cols / 2)), int(np.floor(self.rows / 2))]
        # x and y ranges defined below are default values (will be replaced if corner markers present)
        self.x_range = [self.origin_cell[0] - self.x_dim / self.cell_size // 2, self.origin_cell[0] + self.x_dim / self.cell_size // 2]
        self.y_range = [self.origin_cell[1] - self.y_dim / self.cell_size // 2, self.origin_cell[1] + self.y_dim / self.cell_size // 2]
        # the grid itself with values
        self.grid = []
        self.reset_grid()

        # variables related to matplotlib visualization
        self.fig = None
        self.ax = None

        # [white, green, red, black, gray]
        self.cMap = mpl.colors.ListedColormap([(1,1,1), (1, 0.4, 0.57), (0,1,0), (1,0,0), (0,0,0), (.5,.5,.5)])

        self.heatmap = None
        self.anns = []
        self.cid = None

        # variables related to exporting map and scene files and path file
        self.mapfile = map_filename
        self.scenfile = scen_filename
        self.pathsfile = paths_filename
        self.planfile = plan_filename
        self.end_locations_file = end_locations_filename
        self.endspots = []
        self.has_paths = False
        self.ubuntu_host_dir = ubuntu_dir

        # association of robots with their ID
        self.bots = {}  # maps bot IDs to current spot
        self.end_bots = {}  # maps bot ID's to end spot (if one exists)
        self.bad_bots = []  # simple list of all robots that aren't completely on one cell
        self.out_of_bounds_bots = []  # simple list of all robots that aren't completely within the bounds of the arena
        self.bot_boxes = []  # all the text boxes representing robots
        self.end_boxes = []  # all the text boxes representing the robots' end locations

        # saves path after running the solver
        self.solution_paths_translated = {}
        self.SEND_SOLUTION = False

    def reset_grid(self):
        """
        Resets the grid so that all values are 0 (meaning nothing is in the box)
        """
        self.grid = []
        for i in range(int(self.rows)):
            self.grid.append([CellVal.EMPTY.value for i in range(int(self.cols))])
        self.bots = {}
        self.bad_bots = []

    def __get_blocked_cells(self, vertices_list, dr=0.01):
        """
        Returns a list of grid cells that are blocked by an obstacle.
        traverses in small steps through each line between any two corners of the obstacle
        (according to the markers positions) and marks each grid cell it steps within as blocked.
        Notice that it might miss blocked cells if the step size is to large, but if the cells are not very small it
        supposed to be accurate enough.
        """
        blocked_cells = []
        for pair in itertools.product(vertices_list, repeat=2):
            line_blocked_cells = self.__line_grid_intersection(pair[0], pair[1], dr)
            # blocked_cells.append(line_blocked_cells)
            blocked_cells += line_blocked_cells
        return list(set(blocked_cells))

    def get_positions_list(self, marker_positions):
        """
        Returns a set of [x,y] coordinates of the cell the marker lays within
        """
        set_coords = []
        for mi, marker_pos in enumerate(marker_positions):
            loc = [marker_pos.x, marker_pos.y]  # TODO: change to tuple
            set_coords.append(loc)
        return set_coords

    def add_obstacles(self, obstacles):
        """
        Colors all the cells that are blocked by obstacles.
        """
        for obst in obstacles:
            obst_cord = self.get_positions_list(obst.positions)
            blocked_cells = self.__get_blocked_cells(obst_cord)
            for coord in blocked_cells:
                self.grid[coord[0]][coord[1]] = CellVal.OBSTACLE_REAL.value

    def add_robots(self, robots, tolerance=1):
        """
        A word on tolerance: it describes how "strict" the system will be in order to recognize a robot
        tolerance of 0: all markers must be in one cell
        tolerance of 1: all markers but one must be in the same cell
        tolerance of 2: majority of markers must be in one cell
        If the robot's configuration is outside of the specified tolerance, it will highlight all the cells the robot touches
        """
        for id, robot_markers in robots:
            robot_cords = self.get_positions_list(robot_markers.positions)
            relevant_cells = [self.xy_to_cell(coord) for coord in robot_cords]
            # check if the robot is out of bounds
            in_bounds = True
            for cell in relevant_cells:
                if (cell[0] >= self.y_range[1] or cell[0] <= self.y_range[0] or cell[1] >= self.x_range[1] or cell[1] <=
                        self.x_range[0]):
                    print("y: ", cell[0], "x: ", cell[1])
                    self.out_of_bounds_bots.append(type)
                    in_bounds = False
                    break
            if in_bounds:
                # robot is in bounds; we color it depending on whether it is fully inside a cell or not
                # also need to check for collisions
                mode_cell = mode(relevant_cells)
                majority_count = len(relevant_cells) / 2
                if tolerance == 0 and relevant_cells.count(mode_cell) == len(relevant_cells):  # all cells are in one
                    if self.grid[mode_cell[0]][mode_cell[1]] == CellVal.EMPTY.value:
                        self.grid[mode_cell[0]][mode_cell[1]] = CellVal.ROBOT_FULL.value
                    elif self.grid[mode_cell[0]][mode_cell[1]] == CellVal.OBSTACLE_REAL.value:
                        self.grid[mode_cell[0]][mode_cell[1]] = CellVal.COLLISION.value
                    self.bots[id] = [mode_cell[0], mode_cell[1]]

                elif tolerance == 1 and relevant_cells.count(mode_cell) >= len(
                        relevant_cells) - 1:  # all cells but one are in the same cell
                    if self.grid[mode_cell[0]][mode_cell[1]] == CellVal.EMPTY.value:
                        self.grid[mode_cell[0]][mode_cell[1]] = CellVal.ROBOT_FULL.value
                    elif self.grid[mode_cell[0]][mode_cell[1]] == CellVal.OBSTACLE_REAL.value:
                        self.grid[mode_cell[0]][mode_cell[1]] = CellVal.COLLISION.value
                    self.bots[id] = [mode_cell[0], mode_cell[1]]

                elif tolerance == 2 and relevant_cells.count(
                        mode_cell) >= majority_count:  # majority cells are in the same cell
                    if self.grid[mode_cell[0]][mode_cell[1]] == CellVal.EMPTY.value:
                        self.grid[mode_cell[0]][mode_cell[1]] = CellVal.ROBOT_FULL.value
                    elif self.grid[mode_cell[0]][mode_cell[1]] == CellVal.OBSTACLE_REAL.value:
                        self.grid[mode_cell[0]][mode_cell[1]] = CellVal.COLLISION.value

                    self.bots[id] = [mode_cell[0], mode_cell[1]]
                else:  # if it does not qualify as being in one cell
                    self.bad_bots.append(id)  # add its id to the list of bad robots
                    # highlight all the cells it touches
                    for cell in relevant_cells:
                        self.grid[cell[0]][cell[1]] = CellVal.ROBOT_PARTIAL.value


    def __line_grid_intersection(self, p1, p2, dr):
        """
        Being called from '__get_blocked_cells' to find intersection of line with a grid cell.
        """
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

    def process_collisions(self):
        """
        If there are any robots that are overlapping with obstacles, color them with a certain color.
        Goes through list of self bots and check theirs coordinates.
        It checks if there is an obstacle on any of their coordinates.
        If there is, change the cell's value to some other color.
        """
        for key, value in self.bots.items():
            if self.grid[value[0]][value[1]] == CellVal.OBSTACLE_REAL:
                print('found collision at ', value)
                self.grid[value[0]][value[1]] = CellVal.COLLISION


    #
    def restrict_arena(self):
        """
        Places artificial obstacles around the desired arena
        TODO: Not sure that it is necessary, try to draw only a grid that represents the actual arena
        """
        #if we have our corner markers in the scene, use those; otherwise, use the built-in parameters
        if len(self.corners) == 4:
            x_min = max(self.corners[Corner.TOPLEFT.value][0], self.corners[Corner.BOTTOMLEFT.value][0])
            x_max = min(self.corners[Corner.TOPRIGHT.value][0], self.corners[Corner.BOTTOMRIGHT.value][0])
            y_min = max(self.corners[Corner.TOPRIGHT.value][1], self.corners[Corner.TOPLEFT.value][1])
            y_max = max(self.corners[Corner.BOTTOMRIGHT.value][1], self.corners[Corner.BOTTOMLEFT.value][1])

            self.x_range = [int(x_min), int(x_max)]
            self.y_range = [int(y_min), int(y_max)]

            top_right = [y_min, x_max]
            top_left = [y_min, x_min]
            bottom_right = [y_max, x_max]
            bottom_left = [y_max, x_min]
        else:
            #create artificial based on given parameters
            x = int(self.x_dim / self.cell_size)
            y = int(self.y_dim / self.cell_size)
            # corners
            top_left = [(self.origin_cell[0] - y // 2), self.origin_cell[1] + x // 2] #NOT cartesian
            bottom_left = [(self.origin_cell[0] + y // 2), self.origin_cell[1] + x // 2]
            top_right = [(self.origin_cell[0] - y // 2), self.origin_cell[1] - x // 2]
            bottom_right = [(self.origin_cell[0] + y // 2), self.origin_cell[1] - x // 2]
        corners_to_use = [top_left, top_right, bottom_right, bottom_left]
        borders = corners_to_use
        if len(self.corners)==4:
            for i in range(top_left[1], top_right[1]):  # top border
                borders.append([top_right[0], i])
            for i in range(bottom_left[1], bottom_right[1]):  # bottom border
                borders.append([bottom_right[0], i])
        else:
            for i in range(top_left[1], top_right[1], -1): # top border
                borders.append([top_right[0], i])
            for i in range(bottom_left[1], bottom_right[1], -1): # bottom border
                borders.append([bottom_right[0], i])
        for i in range(top_left[0], bottom_left[0]): # left border
            borders.append([i, bottom_left[1]])
        for i in range(top_right[0], bottom_right[0]): # right border
            borders.append([i, bottom_right[1]])
        # indicate all relevant cells in the grid
        for cell in borders:
            self.grid[cell[0]][cell[1]] = CellVal.OBSTACLE_ART.value

    def xy_to_cell(self, loc):
        """
        Converts x and y from Motive to new coordinate system
        """
        x = -loc[0]
        y = -loc[1]
        return (int(self.origin_cell[0] + np.round(x / self.cell_size)), int(self.origin_cell[1] + np.round(y / self.cell_size)))

    def process_corners(self, corners, dist = 0.15):
        """
        Sets the boundary of the arena according to the corners positions.
        """
        #dist is in cm
        for corner in corners:
            name = corner['name']
            grid_positions = []
            if name[name.index('-')+1::] == 'TL':
                for position in corner['positions']:
                    position['x'] = position['x'] - dist
                    position['y'] = position['y'] - dist
                    x, y = self.xy_to_cell([position['x'], position['y']])
                    grid_positions.append([x, y])
                x = max(position[0] for position in grid_positions)
                y = max(position[1] for position in grid_positions)
                id = 1
            elif name[name.index('-')+1::] == 'BR':
                for position in corner['positions']:
                    position['x'] = position['x'] + dist
                    position['y'] = position['y'] + dist
                    x, y = self.xy_to_cell([position['x'], position['y']])
                    grid_positions.append([x, y])
                x = min(position[0] for position in grid_positions)
                y = min(position[1] for position in grid_positions)
                id = 4
            elif name[name.index('-')+1::] == 'TR':
                for position in corner['positions']:
                    position['x'] = position['x'] - dist
                    position['y'] = position['y'] + dist
                    x, y = self.xy_to_cell([position['x'], position['y']])
                    grid_positions.append([x, y])
                x = min(position[0] for position in grid_positions)
                y = max(position[1] for position in grid_positions)
                id = 2
            elif name[name.index('-')+1::] == 'BL':
                for position in corner['positions']:
                    position['x'] = position['x'] + dist
                    position['y'] = position['y'] - dist
                    x, y = self.xy_to_cell([position['x'], position['y']])
                    grid_positions.append([x, y])
                x = max(position[0] for position in grid_positions)
                y = min(position[1] for position in grid_positions)
                id = 3
            self.corners[id] = [y, x]  # here we swap the x's and y's. switching them to cartesian (visual)

    def plot_init_heatmap(self):
        """
        Initializes a heatmap to be used to display the plot; should be called before plot_render()
        """
        self.fig, self.ax = plt.subplots(1, 1)
        bounds = range(self.cMap.N)
        # norm = mpl.colors.BoundaryNorm(bounds, self.cMap.N)
        data = self.grid
        self.heatmap = self.ax.pcolor(data, edgecolors='k', linewidths=0.01, cmap=self.cMap, vmin=0, vmax=5)
        self.fig.canvas.draw()
        plt.gca().invert_yaxis()
        self.fig.show()

    def init_from_scene(self, event=None):
        """
        Takes an existing .scen file and loads goal locations from it.
        TODO: test and verify it's working correctly
        """
        scen_file = open(self.scenfile, 'r')
        for line in scen_file:
            if line[0] == 'v':
                continue
            data = line.split('\t')
            if int(data[0]) in self.bots:
                self.end_bots[int(data[0])] = [int(data[7]), int(data[6])]

    def init_from_file(self, event=None):
        """
        Takes an existing .txt file and initializes goal locations from it (user's choice), also creates a new .scen file
        TODO: test and verify it's working correctly. Also, what is the difference from "init_from_scene'?
        """
        txt_file = open(self.end_locations_file, 'r')
        for line in txt_file:
            data = line.split('   ')  # python doesn't recognize tab characters for some reason
            print("data: ", data)
            if int(data[0]) in self.bots:
                print('recognized robot', data[0])
                if (self.grid[int(data[2])][int(data[1])] != 0):  # if the spot isn't available
                    print("not available")
                    continue
                if (int(data[1]) >= self.x_range[1] or int(data[1]) <= self.x_range[0] or
                        int(data[2]) >= self.y_range[1] or int(data[2]) <= self.y_range[0]):
                    # if the cell is out of bounds
                    print("space requested is out of bounds")
                    continue
                print("coordinates for robot ", data[0], "will be ", int(data[2]), int(data[1]))
                self.end_bots[int(data[0])] = [int(data[2]), int(data[1])]
        if len(self.end_bots) > 0:
            self.make_scen(from_scratch=False)
            print("scene made")
        else:
            print("ERROR: Scenario file could not be generated because end locations were not found or out of bounds.")
        txt_file.close()

    def plot_render(self):

        if len(self.out_of_bounds_bots) > 0:
            print("the following robots are out of bounds and will not be shownin the visualization: ", self.out_of_bounds_bots)
        if self.heatmap == None:
            self.plot_init_heatmap()
        # re-plot grid with up-to-date values; should be called after updating/adding values
        self.restrict_arena()
        data = self.grid

        #related to color map
        bounds = range(self.cMap.N)
        norm = mpl.colors.BoundaryNorm(bounds, self.cMap.N)
        self.heatmap = self.ax.pcolormesh(data, edgecolors='k', linewidths=0.01, cmap=self.cMap, vmin=0, vmax=5)

        #collision processing
        self.process_collisions()

        #paths
        if self.has_paths:
            for ann in self.anns:
                ann.remove()
            self.anns = []
            #first, open paths file
            # pathsfile = open('paths4.txt', 'r')
            pathsfile = open(self.pathsfile, 'r')

            #for each line, parse it and create a list of the coordinates
            for id, line in enumerate(pathsfile):
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
                    self.anns.append(ann)
                # self.solution_paths_raw[id] = []
                self.solution_paths_translated[id] = paths_clean

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

        ax_planner = plt.axes([0.46, -0.01, 0.1, 0.075])
        ax_from_scen = plt.axes([0.58, -0.01, 0.1, 0.075])
        ax_make_scen = plt.axes([0.7, -0.01, 0.1, 0.075])
        ax_end_locs = plt.axes([0.35, -0.01, 0.1, 0.075])
        button_scenario = Button(ax_make_scen, 'Make .SCEN')
        button_scenario.label.set_fontsize(5)
        button_scenario.on_clicked(self.make_scen)
        button_from_scen = Button(ax_from_scen, 'Load from .SCEN')
        button_from_scen.on_clicked(self.init_from_scene)
        button_from_scen.label.set_fontsize(5)
        button_planner = Button(ax_planner, 'Run Planner')
        button_planner.on_clicked(self.run_planner)
        button_planner.label.set_fontsize(6)
        button_end_locs = Button(ax_end_locs, 'Choose End Locs')
        button_end_locs.on_clicked(self.init_from_file)
        button_end_locs.label.set_fontsize(5)

        self.ax.draw_artist(self.ax.patch)
        self.ax.draw_artist(self.heatmap)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()


        # t_end = time.time()
        plt.pause(0.5)
        self.out_of_bounds_bots = []

    def run_planner(self, event=None):
        print("planner called")
        os.system(f'wsl ~/CBSH2-RTC/cbs -m {self.mapfile} -a {self.scenfile} -o test.csv --outputPaths={self.pathsfile} -k {len(self.bots)} -t 60')
        print("planner finished")
        self.has_paths = True
        self.paths_to_plan()
        if self.SEND_SOLUTION:
            os.system(f'pscp -pw qawsedrf {self.planfile} {self.ubuntu_host_dir}')

    def __make_map(self):
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

    def make_scen(self, from_scratch=True, event=None):
        self.__make_map()
        print(".map file generated")
        #first, warn the user that the scene file is incomplete if there are robots that aren't in cells
        if len(self.bad_bots) > 0:
            print("Robots with the following ID's are not aligned with a single cell and won't be included in the .SCEN file: ", self.bad_bots)
        # for each ROBOT on the grid (meaning its grid value is 1), make a line with all its info
        f = open(self.scenfile, "w")
        f.write("version 1\n")
        count = 0
        # self.end_bots = {} #this resets every time this is the problem
        for key, value in self.end_bots.items():
            if key not in self.bots:
                self.end_bots.pop(key)
        items = sorted(self.bots.items())
        print(items)
        for key, value in items:
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
            if (from_scratch == False) and key in self.end_bots: #if there is already a prepared end location in the dictionary waiting for use
                print("will pull from end bots for: ", key)
                x, y = self.end_bots[key][1], self.end_bots[key][0]
            else:
                print("Generating Random Spot for robot ", key)
                x, y = self.get_empty_spot()
                self.end_bots[key] = [y, x]
                self.end_bots[key] = [y, x]
            f.write(str(x) + '\t' + str(y) + '\t')
            # optimal distance
            print("value: ", value)
            print("x_range: ", self.x_range)
            print("y_range: ", self.y_range)
            print("y x", y, " ", x)
            # f.write(f'{self.get_optimal_length((value[1], value[0]), (0, 0))}\n')
            f.write(f'{self.get_optimal_length((value[1], value[0]), (y, x))}\n')
        f.close()
        self.has_paths = False
        print(".scen file generated")

    def get_empty_spot(self):
        while True: #choose within borders of arena, not the 12x12
            try_x = random.randint(self.x_range[0], self.x_range[1])
            try_y = random.randint(self.y_range[0], self.y_range[1])
            if self.grid[try_y][try_x] != 0:
                continue
            for coord in self.endspots: # now need to make sure no two ending spots align
                if coord[1] == try_x and coord[0] == try_y:
                    continue
            break
        self.endspots.append([try_y, try_x])
        return try_x, try_y

    def get_optimal_length(self, loc1, loc2):
        # path = list(astar.find_path(loc1, loc2, neighbors_fnct=lambda loc: self.neighbors(loc, True), heuristic_cost_estimate_fnct=self.heuristic, distance_between_fnct=self.distance))
        #should be the line above, but sometimes causes issues; line below is a band-aid solution only!
        path = list(astar.find_path((4,7), (6, 8), neighbors_fnct=lambda loc: self.neighbors(loc, True), heuristic_cost_estimate_fnct=self.heuristic, distance_between_fnct=self.distance))
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

    def paths_to_plan(self):
        """
        Converts the output of the CBS planner to the input of Hadar's ROS code and saves it in a file by the name of
        grid.planfile
        TODO: not sure it belongs to 'Grid' class, consider moving to some general 'utility' class
        """
        paths_file = open(self.pathsfile, "r")
        plan_file = open(self.planfile, "w")

        plan_file.write("schedule:\n")
        all_robots_starts_at_zero_zero = True

        for line in paths_file:
            # get and write agent number
            space_idx = line.index(" ")
            colon_idx = line.index(":")
            id = line[space_idx:colon_idx]
            plan_file.write("\tagent" + id.replace(" ", "") + ":\n")

            # get sequence of coordinates
            path_string = line[colon_idx + 2::]
            path_list = path_string.split('->')

            # parse and write out coordinates
            start_location = []
            counter = 0
            for coord in path_list:
                if coord == '\n' or coord == '':
                    continue
                else:
                    coord = coord[1:-1]
                    # this is to compensate for the flipped coordinates that the planner outputs
                    y, x = coord.split(',')

                    if all_robots_starts_at_zero_zero:
                        if not start_location:
                            start_location = [x, y]
                        x = str(int(x) - int(start_location[0]))
                        y = str(-(int(y) - int(start_location[1])))
                    plan_file.write('\t\t- x: ' + x + '\n\t\t y: ' + y + '\n\t\t t: ' + str(counter) + '\n')
                    counter = counter + 1
        plan_file.close()
        return self.pathsfile


if __name__ == "__main__":
    # Testing the optimal length function
    env = Grid(12, 12)
    m, n = env.rows, env.cols
    start_x = np.random.randint(0, m)
    start_y = np.random.randint(0, n)
    goal_x = np.random.randint(0, m)
    goal_y = np.random.randint(0, n)

    print(env.get_optimal_length((start_x, start_y), (goal_x, goal_y)))


