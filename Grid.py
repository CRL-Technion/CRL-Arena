import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import astar
import random
import itertools

from enum import Enum
from statistics import mode
from matplotlib.colors import ListedColormap
from matplotlib.widgets import Button
from shapely.geometry import LineString

import pygame
from pygame.locals import KEYDOWN, K_q

# CONSTANTS:
SCREENSIZE = WIDTH, HEIGHT = 800, 600
BLACK = (0, 0, 0)
GREY = (160, 160, 160)
WHITE = (255, 255, 255)




class CellVal(Enum):
    """
    Represents the state of scenario object to use for drawing the grid
    """
    EMPTY = 0
    COLLISION = 1  # a robot and a real obstacle occupy the same cell
    ROBOT_FULL = 2  # entire robot is on one cell
    ROBOT_PARTIAL = 3  # robot is spread out over multiple cells
    OBSTACLE_REAL = 4
    OBSTACLE_ART = 5  # artificial obstacle


class Corner(Enum):
    """
    Values associated with corner markers
    """
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
    def __init__(self, cell_size, rows, cols,
                 broadcast_cond,
                 map_filename,
                 scene_filename,
                 goal_locations,
                 paths_filename,
                 algorithm_output,
                 x_dim: int = 10, y_dim: int = 6,
                 ):
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



        self.broadcast_cond = broadcast_cond
        self.run_planner_cond = False

        # arena dimensions (within greater 12x12 scope)
        self.x_dim = min(x_dim, 12)  # meters
        self.y_dim = min(y_dim, 12)  # meters
        self.corners = {}

        self.cell_size = cell_size  # meters TODO: consider remove from grid after removing arena restriction
        # TODO: changed from what originally was here - validate everywhere + documentation
        self.rows = int(rows)
        self.cols = int(cols)

        print(rows)
        print(cols)
        # OUR GRID MAP:
        #self.cellMAP = np.random.randint(2, size=(self.rows, self.cols))

        # VARS:
        self._VARS = {'surf': pygame.display.set_mode(SCREENSIZE), 'gridWH': 400, 'gridScale': SCREENSIZE[0] / 2,
                 'gridOrigin': (10, 10), 'gridCells': self.rows*self.cols, 'lineWidth': 2}
        # # rows and columns are like the larger grid, while x and y dim are the smaller arena
        # self.rows = int(12 / cell_size)
        # self.cols = int(12 / cell_size)
        # TODO: consider remove origin_cell and change xy to cell translation based on new grid
        self.grid_origin_cell = [int(np.floor(self.cols / 2)), int(np.floor(self.rows / 2))]
        print(self.grid_origin_cell)
        self.screen_grid_origin = (10, 10)  # this is the place in the window where the top-left corner of the grid is placed
        self.cell_dim = 30  # the dimension of a square grid cell (not depented on the actual grid cell in meters, this is just for visualization)

        # TODO: changed from what originally was here - validate everywhere + documentation
        # x and y ranges are aligned with the LAB's coordinates system
        self.y_range = [int(- np.floor(self.rows / 2)), int(np.ceil(self.rows / 2))]
        self.x_range = [int(- np.floor(self.cols / 2)), int(np.ceil(self.cols / 2))]
        # # x and y ranges defined below are default values (will be replaced if corner markers present)
        # self.x_range = [self.origin_cell[0] - self.x_dim / self.cell_size // 2, self.origin_cell[0] + self.x_dim / self.cell_size // 2]
        # self.y_range = [self.origin_cell[1] - self.y_dim / self.cell_size // 2, self.origin_cell[1] + self.y_dim / self.cell_size // 2]
        # the grid itself with values

        # This is a 2D array representing the grid (!!!)
        # NOTE (!) that accessing with (y,x) to be aligned with LAB's coordinates
        self.grid = []
        #self.reset_grid()

        # variables related to matplotlib visualization
        self.fig = None
        self.ax = None

        # [white, green, red, black, gray]
        self.cMap = mpl.colors.ListedColormap([(1,1,1), (1, 0.4, 0.57), (0,1,0), (1,0,0), (0,0,0), (.5,.5,.5)])

        self.heatmap = None
        self.cid = None

        # variables related to exporting map and scene files and path file
        self.mapfile = map_filename
        self.scenfile = scene_filename
        self.pathsfile = paths_filename
        self.algorithm_output = algorithm_output
        self.end_locations_file = goal_locations
        self.endspots = []
        self.has_paths = False

        # association of robots with their ID
        self.bots = {}  # maps bot IDs to current spot, NOTE that locations are saved as (y,x)
        self.end_bots = {}  # maps bot ID's to end spot (if one exists)
        self.bad_bots = []  # simple list of all robots that aren't completely on one cell
        self.out_of_bounds_bots = []  # simple list of all robots that aren't completely within the bounds of the arena
        self.bot_boxes = []  # all the text boxes representing robots
        self.end_boxes = []  # all the text boxes representing the robots' end locations

        # saves path after running the solver
        self.solution_paths_translated = {}

    # NEW METHOD FOR ADDING CELLS :
    def placeCells(self):
        # GET CELL DIMENSIONS...
        cellBorder = self.cell_dim / 10
        celldimX = celldimY = self.cell_dim - (cellBorder * 2)
        # DOUBLE LOOP
        for row in range(self.rows):
            for column in range(self.cols):
                # Is the grid cell tiled ?
                if (self.grid[row][column] == CellVal.OBSTACLE_REAL.value):
                    self.drawSquareCell(
                        self.screen_grid_origin[0] + (self.cell_dim * column)
                        + cellBorder + self._VARS['lineWidth'] / 2,
                        self.screen_grid_origin[1] + (self.cell_dim * row) + cellBorder + self._VARS['lineWidth'] / 2,
                        celldimX, celldimY)
                    # self.drawSquareCell(
                    #     self.screen_grid_origin[0] + (celldimY * row)
                    #     + cellBorder + (2 * row * cellBorder) + self._VARS['lineWidth'] / 2,
                    #     self.screen_grid_origin[1] + (celldimX * column)
                    #     + cellBorder + (2 * column * cellBorder) + self._VARS['lineWidth'] / 2,
                    #     celldimX, celldimY)

    # Draw filled rectangle at coordinates
    def drawSquareCell(self, x, y, dimX, dimY):
        pygame.draw.rect(
            self._VARS['surf'], BLACK,
            (x, y, dimX, dimY)
        )

    def drawSquareGrid(self):

        cont_x, cont_y = self.screen_grid_origin
        grid_height = int(self.rows) * self.cell_dim
        grid_width = int(self.cols) * self.cell_dim

        # DRAW Grid Border:
        # TOP lEFT TO RIGHT
        pygame.draw.line(
            self._VARS['surf'], BLACK,
            (cont_x, cont_y),
            (grid_width + cont_x, cont_y), self._VARS['lineWidth'])
        # # BOTTOM lEFT TO RIGHT
        pygame.draw.line(
            self._VARS['surf'], BLACK,
            (cont_x, grid_height + cont_y),
            (grid_width + cont_x,
             grid_height + cont_y), self._VARS['lineWidth'])
        # # LEFT TOP TO BOTTOM
        pygame.draw.line(
            self._VARS['surf'], BLACK,
            (cont_x, cont_y),
            (cont_x, cont_y + grid_height), self._VARS['lineWidth'])
        # # RIGHT TOP TO BOTTOM
        pygame.draw.line(
            self._VARS['surf'], BLACK,
            (grid_width + cont_x, cont_y),
            (grid_width + cont_x,
             grid_height + cont_y), self._VARS['lineWidth'])

        # VERTICAL DIVISIONS: (0,1,2) for grid(3) for example
        for x in range(self.cols):
            pygame.draw.line(
                self._VARS['surf'], BLACK,
                (cont_x + (self.cell_dim * x), cont_y),
                (cont_x + (self.cell_dim * x), grid_height + cont_y), 2)
        # HORIZONTAl DIVISIONS
        for x in range(self.rows):
            pygame.draw.line(
                self._VARS['surf'], BLACK,
                (cont_x, cont_y + (self.cell_dim * x)),
                (cont_x + grid_width, cont_y + (self.cell_dim * x)), 2)

    def checkEvents(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            elif event.type == KEYDOWN and event.key == K_q:
                pygame.quit()
                sys.exit()

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
                print("prev: " + str(coord))
                new_coord = self.cell_to_grid_cell(coord)
                print("after: " + str(new_coord))
                self.grid[new_coord[1]][new_coord[0]] = CellVal.OBSTACLE_REAL.value

    def add_robots(self, robots, tolerance=1):
        """
        Adds a robot to the grid and colors the relevant cell accordingly.
        A word on tolerance: it describes how "strict" the system will be in order to recognize a robot
        tolerance of 0: all markers must be in one cell
        tolerance of 1: all markers but one must be in the same cell
        tolerance of 2: majority of markers must be in one cell
        If the robot's configuration is outside of the specified tolerance, it will highlight all the cells the robot touches
        """
        for id, robot_markers in robots:
            # calculate the grid cells that the robots markers lay within
            robot_cords = self.get_positions_list(robot_markers.positions)
            relevant_markers_cells = [self.xy_to_cell(coord) for coord in robot_cords]

            # check if the robot is out of the grid's bounds
            # consider out of bounds if one of the markers is out of bounds
            in_bounds = all(self.marker_in_bound(marker_cell) for marker_cell in relevant_markers_cells)
            if not in_bounds:
                self.out_of_bounds_bots.append((id, relevant_markers_cells))

            else:
                # robot is in bounds.
                # the color that the robot would appear with is depending on whether it is fully inside a cell or not.
                # we also check for collisions with obstacles or between robots
                mode_cell = mode(relevant_markers_cells)  # get the most common grid cell from the markers list
                majority_count = len(relevant_markers_cells) / 2  # how many markers count as a majority

                if relevant_markers_cells.count(mode_cell) == len(relevant_markers_cells):
                    # all markers are in one cell
                    self.set_cell_color(id, mode_cell)

                elif relevant_markers_cells.count(mode_cell) >= len(relevant_markers_cells) - 1 \
                        and tolerance == 1:
                    # all markers but one are in the same cell
                    # if tolerance is 1 then it is ok
                    self.set_cell_color(id, mode_cell)

                elif relevant_markers_cells.count(mode_cell) >= majority_count and tolerance == 2:
                    # majority markers are in the same cell
                    # if tolerance is 2 then it is ok
                    self.set_cell_color(id, mode_cell)

                else:
                    # the current location of the markers does not qualify as being in one cell
                    # according to the provided tolerance
                    self.bad_bots.append(id)  # add its id to the list of bad robots
                    # highlight all the cells it touches
                    for cell in relevant_markers_cells:
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

    def restrict_arena(self):
        """
        Places artificial obstacles around the desired arena
        TODO: Not sure that it is necessary, try to draw only a grid that represents the actual arena
        TODO: simplify method
        """
        # if we have our corner markers in the scene, use those; otherwise, use the built-in parameters
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
        return (int(self.grid_origin_cell[0] + np.round(x / self.cell_size)), int(self.grid_origin_cell[1] + np.round(y / self.cell_size)))

    def cell_to_grid_cell(self, loc):
        new_origin = (self.x_range[0], self.y_range[1])
        print("new origin: " + str(new_origin))
        # temp = (-loc[0]+new_origin[0], -loc[1]+new_origin[1])
        # return -temp[0], temp[1]
        return int(np.abs(new_origin[0] - loc[0])), int(np.abs(new_origin[1] - loc[1]))

    def process_corners(self, corners):
        """
        Sets the boundary of the arena according to the corners positions.
        TODO: simplify method
        """
        #dist is in cm
        dist = self.cell_size / 2.0

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
        Initializes a heatmap to be used to display the plot; should be called before plot_render().
        """
        self.fig, self.ax = plt.subplots(1, 1)
        # bounds = range(self.cMap.N)
        # norm = mpl.colors.BoundaryNorm(bounds, self.cMap.N)
        # self.grid[0][0] = CellVal.OBSTACLE_ART.value
        data = self.grid
        self.heatmap = self.ax.pcolor(data, edgecolors='k', linewidths=0.01, cmap=self.cMap, vmin=0, vmax=5)
        self.fig.canvas.draw()
        #plt.gca().invert_yaxis()
        self.fig.show()

    def init_from_scene(self, event=None):
        """
        Takes an existing .scen file and loads ONLY goal locations from it.
        Note that it does not load the start locations and map, and won't create a new .scen and .map files.
        TODO: test and verify it's working correctly
        TODO: check validity of input (number of agents, file format) otherwise report and abort
        """
        if self.scenfile != "scene.scen":
            # a scene file was given to the program
            try:
                with open(self.scenfile, 'r') as scen_file:
                    print("Loading (ONLY) goal locations from .scen file.")
                    print("Note that new .scen and .map files would not be generated.")
                    for line in scen_file:
                        if line[0] == 'v':
                            continue
                        data = line.split('\t')
                        if int(data[0]) in self.bots:
                            self.end_bots[int(data[0])] = [int(data[7]), int(data[6])]
            except IOError as e:
                print(f"Couldn't open file {e}.")
        else:
            # no valid .scen was given to the program
            print("No valid .scen was given to the program! \n"
                  "Use random goal locations generation or provide a valid .scen file at command line args.")

    def init_from_file(self, event=None):
        """
        Takes an existing .txt file with specified goal locations and initializes a scenario.
        Note that a new .scen and .map files will be created.
        TODO: test and verify it's working correctly
        TODO: check validity of input (number of agents, file format) otherwise report and abort
        """
        if self.end_locations_file != "":
            # a valid goals file was given
            with open(self.end_locations_file, 'r') as txt_file:
                print("Loading goal locations from file and generating new scenario (.scen and .map files).")
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
                            print("location requested is out of bounds")
                            continue
                        print("coordinates for robot ", data[0], "will be ", int(data[2]), int(data[1]))
                        self.end_bots[int(data[0])] = [int(data[2]), int(data[1])]
            if len(self.end_bots) > 0:  # TODO: check this part is actually working correctly
                self.make_scen(from_scratch=False)
                print("Scene generated")
            else:
                print("ERROR: Scenario file could not be generated because end locations were not found "
                      "or out of bounds.")
        else:
            # no valid goals file was given to the program
            print("No valid goals file was given to the program! \n"
                  "Use random goal locations generation or provide a valid goals file at command line args.")

    def broadcast_solution(self, event=None):
        """
        Called when pressing the "Broadcast solution data" button.
        notifies the main thread to initialize data transmission via UDP.
        """
        with self.broadcast_cond:
            self.broadcast_cond.notify()

    # TODO: add detailed explaination about the method
    def plot_render(self):
        # Notify about robots that are out of bounds
        for robot_id, markers in self.out_of_bounds_bots:
            print(f"Robot {robot_id} is out of bounds and will not be shown in the visualization.\n"
                  f"Its markers are located in cells: {markers}")

        if not self.heatmap:
            self.plot_init_heatmap()
        # re-plot grid with up-to-date values; should be called after updating/adding values
        #self.restrict_arena()
        data = self.grid

        # create base grid
        self.heatmap = self.ax.pcolormesh(data, edgecolors='k', linewidths=0.01, cmap=self.cMap, vmin=0, vmax=5)

        # process robot-robot and robot-obstacles collisions
        self.process_collisions()

        # draw paths if solution exists
        if self.has_paths:
            # open paths file
            pathsfile = open(self.pathsfile, 'r')

            # each line in the file contains a path for the matching agent (starting with 0)
            # parsing the line to get the grid locations along the paths
            for id, line in enumerate(pathsfile):
                colon_idx = line.index(":")
                path_string = line[colon_idx + 2::]
                paths_list = path_string.split('->')

                # draw lines between each two
                paths_clean = []

                for coord in paths_list:
                    if coord == '\n' or coord == '':
                        paths_list.remove(coord)
                    else:
                        clean = coord[1:-1]
                        clean = clean.split(',')  # a list ['x', 'y'] representing a location on a path
                        paths_clean.append((int(clean[0]), int(clean[1])))  # inserting a tuple of int type

                # draw solution paths on grid (with annotations - draws '-' at each (x,y) location on the path)
                for i in range(len(paths_clean) - 1):
                    self.ax.annotate("", xy = (paths_clean[i][1] + 0.5, paths_clean[i][0] + 0.5),
                                     xytext=(paths_clean[i+1][1] + 0.5, paths_clean[i+1][0]+0.5),
                                     arrowprops=dict(arrowstyle='-', connectionstyle='arc3'))

                # save paths with grid cells translation for later passing it to the arena visualizer
                self.solution_paths_translated[id] = paths_clean

        # TODO: understand exactly what's going here and simplify
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

        # TODO: check the warning about opening multiple axes objects
        ax_make_scen = plt.axes([0.8, -0.01, 0.1, 0.075])
        ax_from_scen = plt.axes([0.65, -0.01, 0.1, 0.075])
        ax_end_locs = plt.axes([0.5, -0.01, 0.1, 0.075])
        ax_planner = plt.axes([0.35, -0.01, 0.1, 0.075])
        ax_udp = plt.axes([0.2, -0.01, 0.1, 0.075])

        button_scenario = Button(ax_make_scen, 'Make Random Scene')
        button_scenario.label.set_fontsize(4)
        button_scenario.on_clicked(self.make_scen)

        # to load goal location from a pre-generated scene
        button_from_scen = Button(ax_from_scen, 'Load Goals from Scene')
        button_from_scen.on_clicked(self.init_from_scene)
        button_from_scen.label.set_fontsize(5)

        # to load goal locations from a pre-defined dedicated goals file
        button_end_locs = Button(ax_end_locs, 'Load Goals from File')
        button_end_locs.on_clicked(self.init_from_file)
        button_end_locs.label.set_fontsize(5)

        button_planner = Button(ax_planner, 'Run Planner')
        button_planner.on_clicked(self.run_planner)
        button_planner.label.set_fontsize(5)

        button_udp = Button(ax_udp, 'Broadcast Solution \n and Data')
        button_udp.on_clicked(self.broadcast_solution)
        button_udp.label.set_fontsize(5)

        self.ax.draw_artist(self.ax.patch)
        self.ax.draw_artist(self.heatmap)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()

        # t_end = time.time()
        plt.pause(0.5)
        self.out_of_bounds_bots = []

    # TODO: add descriptions and clean from here
    def run_planner(self, event=None):
        """
        Turns on the flag that tells the PlannerController loop to run the planner.
        TODO: not the ideal way to notify on async event from a different class,
                maybe try to modify if we find a different way.
        """
        self.run_planner_cond = True

    def __make_map(self):
        """
        Generates a .map file of the projected scenario according to the map file conventions.
        Currently, we follow the conventions required to run the common benchmarks MAPF kit.
        """
        with open(self.mapfile, "w") as f:
            # required headers
            f.write("type octile\n")
            f.write("height " + str(self.rows) + '\n')
            f.write("width " + str(self.cols) + '\n')
            f.write("map\n")

            # write the map to file row-by-row
            for i in range(int(self.rows)):
                for j in range(int(self.cols)):
                    if self.grid[i][j] == CellVal.OBSTACLE_ART.value or self.grid[i][j] == CellVal.OBSTACLE_REAL.value:
                        f.write('@')  # blocked cell
                    else:
                        f.write('.')  # free cell
                f.write('\n')

    def make_scen(self, from_scratch=True, event=None):
        """
        Generates a .scen file of the projected scenario according to the scenario file conventions.
        Currently, we follow the conventions required to run the common benchmarks MAPF kit.

        - from_scratch - specifies that a new random end locations are required
        TODO: check that this is actually what from_scratch does and that it works
            (also when trying to get pre-defined end locations and from_scratch is False)
        """

        self.__make_map()
        print("Map file generated (.map file)")

        # first, warn the user that the scene file is incomplete if there are robots that aren't in cells
        if len(self.bad_bots) > 0:
            print("Robots with the following ID's are not aligned with a single cell "
                  "and won't be included in the scenario file: ", self.bad_bots)

        # for each ROBOT on the grid (meaning its grid value is 1), add a line to the scenario file with all its info
        f = open(self.scenfile, "w")
        f.write("version 1\n")  # scenario file convention

        # this is relevant only if we changed the arena after already generating goal locations
        for key, value in self.end_bots.items():
            if key not in self.bots:
                self.end_bots.pop(key)

        items = sorted(self.bots.items())
        for key, value in items:
            # bucket
            f.write(str(key)+'\t')
            # .map file name
            f.write(str(self.mapfile) + '\t')
            # dimensions of the grid
            f.write(str(int(self.rows)) + '\t' + str(int(self.cols)) + '\t')
            # start location
            f.write(str(value[1]) + '\t' + str(value[0]) + '\t')
            # goal location
            # if there is already a prepared goal location in the dictionary waiting for use
            if (from_scratch == False) and key in self.end_bots:
                print("will pull goal location from already existing goal locations set for robot ", key)
                x, y = self.end_bots[key][1], self.end_bots[key][0]
            else:
                print("Generating random goal location for robot ", key)
                x, y = self.get_empty_spot()
                self.end_bots[key] = [y, x]
                self.end_bots[key] = [y, x]
            f.write(str(x) + '\t' + str(y) + '\t')
            # optimal distance to goal
            # NOTE!!! we don't need to switch start location x and y
            # because it is already saved in self.bots as required (y,x)
            f.write(f'{self.get_optimal_length((value[0], value[1]), (y, x))}\n')
        f.close()
        self.has_paths = False
        print("Scenario file generated (.scen file)")

    def get_empty_spot(self):
        """
        Returns a random empty cell on the grid.
        Also checks that it has not been generated before.
        """
        while True:  # choose within borders of arena, not the 12x12 TODO: modify after removing borders
            try_x = random.randint(self.x_range[0], self.x_range[1])
            try_y = random.randint(self.y_range[0], self.y_range[1])
            if self.grid[try_y][try_x] != 0:
                continue
            for coord in self.endspots:  # make sure no two goal locations are aligned
                if coord[1] == try_x and coord[0] == try_y:
                    continue
            break
        self.endspots.append([try_y, try_x])
        return try_x, try_y

    def get_optimal_length(self, loc1, loc2):
        """
        Returns the shortest distance between two location on the grid using A* algorithm.
        """
        # TODO: need to debug this call and verify that it's not crashing from time to time
        res = astar.find_path(loc1, loc2,
                              neighbors_fnct=lambda loc: self.neighbors(loc, diagonal_moves=False),
                              heuristic_cost_estimate_fnct=self.heuristic,
                              distance_between_fnct=self.distance)
        path = list(res)
        # calculate path's total distance
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
            if 0 <= new_loc[0] <= self.cols-1 and \
                    0 <= new_loc[1] <= self.rows-1 and \
                    self.grid[new_loc[0]][new_loc[1]] == 0:
                neighbors.append(tuple(new_loc))

        return neighbors

    def heuristic(self, loc, goal):
        """
        Currently using the Euclidean distance to the goal as heuristic
        """
        return np.linalg.norm(np.array(loc)-np.array(goal), 2)

    def distance(self, loc1, loc2):
        """
        Euclidean distance between two locations on the grid
        """
        return np.linalg.norm(np.array(loc1)-np.array(loc2), 2)

    def marker_in_bound(self, marker_cell):
        """
        Checks that a marker is located within the grid's boundaries
        """
        # TODO: verify x and y correctness
        return (self.y_range[1] > marker_cell[0] > self.y_range[0]) \
               and (self.x_range[1] > marker_cell[1] > self.x_range[0])

    def set_cell_color(self, robot_id, mode_cell):
        """
        Sets a robot's cell color according to the actual situation
        """
        if self.grid[mode_cell[0]][mode_cell[1]] == CellVal.EMPTY.value:
            self.grid[mode_cell[0]][mode_cell[1]] = CellVal.ROBOT_FULL.value
        elif self.grid[mode_cell[0]][mode_cell[1]] == CellVal.OBSTACLE_REAL.value:
            self.grid[mode_cell[0]][mode_cell[1]] = CellVal.COLLISION.value
        self.bots[robot_id] = [mode_cell[0], mode_cell[1]]


if __name__ == "__main__":
    # Testing the optimal length function
    env = Grid(12, 12)
    m, n = env.rows, env.cols
    start_x = np.random.randint(0, m)
    start_y = np.random.randint(0, n)
    goal_x = np.random.randint(0, m)
    goal_y = np.random.randint(0, n)

    print(env.get_optimal_length((start_x, start_y), (goal_x, goal_y)))


