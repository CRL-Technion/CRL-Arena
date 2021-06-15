import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from statistics import mode
import time

from matplotlib.widgets import Button

from natnet.protocol import RigidBody, LabeledMarker, Position, Rotation
import random

import itertools
from shapely.geometry import LineString






class Grid:
    # A class that holds a grid and can visualize this grid, export it as a map file, or export it as scene file
    def __init__(self, x_dim:int=12, y_dim:int=12, cell_size:float=1.0, map_filename:str='my_map.map', scen_filename:str= 'my_scene.scen'):
        # starting dimensions, etc
        self.x_dim = x_dim #m
        self.y_dim = y_dim #m
        self.cell_size = cell_size #m
        self.rows = int(y_dim / cell_size)
        self.cols = int(x_dim / cell_size)
        self.origin_cell = [int(np.floor(self.cols / 2)), int(np.floor(self.rows / 2))]

        # the grid itself with values
        self.grid = []
        self.reset_grid()

        # variables related to matplotlib visualization
        self.fig = None
        self.ax = None
        self.cMap = mpl.colors.ListedColormap(['w', 'r', 'k', 'y', 'c', 'm'])
        self.heatmap = None

        # variables related to exporting map and scene files
        self.mapfile = map_filename
        self.scenfile = scen_filename
        self.endspots = []

    def reset_grid(self):
        # reset the grid so that all values are 0 (meaning nothing is in the box)
        self.grid = []
        for i in range(int(self.rows)):
            self.grid.append([0 for i in range(int(self.cols))])

    def getBlockedCells(self, vertices_list, dr=0.01): #TODO: make it so that this only uses the outer vertices??? waste of time if there's a body with a lot of inner markers
        blocked_cells = []
        for pair in itertools.product(vertices_list, repeat=2):
            line_blocked_cells = self.lineGridIntersection(pair[0], pair[1], dr)
            #blocked_cells.append(line_blocked_cells)
            blocked_cells += line_blocked_cells
        return list(set(blocked_cells))

    def add_body(self, type, body_coords):
        #if type is obstacle, then paint all the cells
        if type == 2: #obstacle
            blocked_cells = self.getBlockedCells(body_coords)
            for coord in blocked_cells:
               self.grid[coord[0]][coord[1]] = 1
        elif type == 1: #robot
            relevant_cells = [self.xy_to_cell(coord) for coord in body_coords]
            mode_cell = mode(relevant_cells)
            majority_count = len(relevant_cells) / 2
            print(relevant_cells)
            if relevant_cells.count(mode_cell) >= majority_count: #TODO: add some tolerance adjustability here
                #just highlight one cell in the grid
                self.grid[mode_cell[0]][mode_cell[1]] = 2
            else:
                #highlight all the cells it touches
                for cell in relevant_cells:
                    self.grid[cell[0]][cell[1]] = 3
            #if half or more than half of markers are in the same cell, then paint that one cell one color
            #otherwise, paint the rest of the cells another color


    def lineGridIntersection(self, p1, p2, dr):
        ls = LineString([p1, p2])
        points_on_line = []
        # for f in range(0, int(np.ceil(ls.length)) + 1):
        # linespace = [x * dr for x in range(0, int(np.ceil(ls.length)) + 1)]
        line_length = np.ceil(ls.length)
        num_samples = int(line_length / dr)
        linespace = [x * dr for x in range(0, num_samples + 1)]
        for f in linespace:
            p = ls.interpolate(f).coords[0]

            points_on_line.append(p)

        ar = np.array(points_on_line, 'f')

        cells = list(map(lambda p: self.xy_to_cell(p), ar))
        return cells

    def plot_init_heatmap(self):
        # initialize heatmap to be used to display the plot; should be called before plot_render()
        self.fig, self.ax = plt.subplots(1, 1)
        bounds = range(self.cMap.N)
        norm = mpl.colors.BoundaryNorm(bounds, self.cMap.N)
        data = self.grid


        self.heatmap = self.ax.pcolor(data, edgecolors='k', linewidths=1, cmap=self.cMap, norm=norm)
        self.fig.canvas.draw()
        plt.gca().invert_yaxis()

        axprev = plt.axes([0.7, 0.05, 0.1, 0.075])
        axnext = plt.axes([0.81, 0.05, 0.1, 0.075])
        bnext = Button(axnext, 'Map')
        bnext.on_clicked(self.make_map)
        bprev = Button(axprev, 'Scenario')
        bprev.on_clicked(self.make_scen)

        self.fig.show()

    def plot_render(self):

        # re-plot grid with up-to-date values; should be called after updating/adding values
        data = self.grid
        # t_start = time.time()
        data[self.origin_cell[0]][self.origin_cell[1]] = 5
        if self.heatmap == None:
            self.plot_init_heatmap()
        bounds = range(self.cMap.N)
        norm = mpl.colors.BoundaryNorm(bounds, self.cMap.N)
        self.heatmap = self.ax.pcolormesh(data, edgecolors='k', linewidths=1, cmap=self.cMap, norm=norm)



        self.ax.draw_artist(self.ax.patch)
        self.ax.draw_artist(self.heatmap)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()
        # t_end = time.time()
        plt.pause(0.2)

    def xy_to_cell(self, loc):
        # convert x and y from Motive to new coordinate system
        x = -loc[0]
        y = -loc[1]
        return (int(self.origin_cell[0] + np.round(x / self.cell_size)), int(self.origin_cell[1] + np.round(y / self.cell_size)))

    def add_coord(self, loc: list, body_type: str):  # TODO should be enum
        # convert the coordinate to cell coordinates
        newloc = self.xy_to_cell(loc)
        if body_type == "robot":  # then the first number is a 1, making it a robot
            self.grid[newloc[0]][newloc[1]] = 1
        elif body_type == "obstacle":  # then the first number is a 2, making it an obstacle
            self.grid[newloc[0]][newloc[1]] = 2

    def make_map(self):
        f = open(self.mapfile, "w")
        f.write("type octile\n")
        f.write("height " + str(self.rows) + '\n')
        f.write("width " + str(self.cols) + '\n')
        f.write("map\n")
        for i in range(int(self.rows)):
            for j in range(int(self.cols)):
                if self.grid[i][j] == 0 or self.grid[i][j] == 1: #empty space
                    f.write('.')
                elif self.grid[i][j] == 2: #obstacle
                    f.write('@')
            f.write('\n')
        f.close()

    def get_empty_spot(self):
        try_x = -1
        try_y = -1
        while True:
            try_x = random.randint(0, self.cols)
            try_y = random.randint(0, self.rows)
            if self.grid[try_x][try_y] != 0:
                continue
            for coord in self.endspots: # now need to make sure no two ending spots align
                if coord[0] == try_x and coord[1] == try_y:
                    continue
            break
        self.endspots.append([try_x, try_y])
        return try_x, try_y

    def make_scen(self):
        # for each ROBOT on the grid (meaning its grid value is 1), make a line with all its info
        f = open(self.scenfile, "w")
        f.write("version 1\n")
        for i in range(int(self.rows)):
            for j in range(int(self.cols)):
                if self.grid[i][j] == 1:
                    # bucket
                    f.write('0\t')
                    # .map file name
                    f.write(str(self.mapfile) + '\t')
                    # dimensions of the grid
                    f.write(str(int(self.rows)) + '\t' + str(int(self.cols)) + '\t')
                    # starting position
                    f.write(str(i) + '\t' + str(j) + '\t')
                    # ending position
                    x, y = self.get_empty_spot()
                    f.write(str(x) + '\t' + str(y) + '\t')
                    # distance thing??
                    f.write('PLACEHOLDER\n')
        f.close()


