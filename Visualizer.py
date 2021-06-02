import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from MapMaker import MapMaker
import time
from natnet.protocol import RigidBody, LabeledMarker, Position, Rotation


class Visualizer():
    def __init__(self, x_dim:int=11, y_dim:int=11, cell_size:float=1.0):
        self.x_dim = x_dim #m
        self.y_dim = y_dim #m
        self.cell_size = cell_size #m
        self.rows = x_dim / cell_size
        self.cols = y_dim / cell_size
        self.origin_cell = [np.floor(self.rows / 2), np.floor(self.cols / 2)]
        self.grid = []
        self.reset_grid()
        self.fig = None
        self.ax = None
        self.cMap = mpl.colors.ListedColormap(['w', 'r', 'b', 'b'])
        self.heatmap = None

    def reset_grid(self):
        self.grid = []
        for i in range(int(self.rows)):
            self.grid.append([0 for i in range(int(self.cols))])

    def initialize_heatmap(self):
        self.fig, self.ax = plt.subplots(1, 1)
        bounds = range(self.cMap.N)
        norm = mpl.colors.BoundaryNorm(bounds, self.cMap.N)
        data = self.grid
        self.heatmap = self.ax.pcolor(data, edgecolors='k', linewidths=1, cmap=self.cMap, norm=norm)
        self.fig.canvas.draw()
        self.fig.show()
        plt.show()

    def render(self):
        data = self.grid
        # t_start = time.time()
        bounds = range(self.cMap.N)
        norm = mpl.colors.BoundaryNorm(bounds, self.cMap.N)
        self.heatmap = self.ax.pcolor(data, edgecolors='k', linewidths=1, cmap=self.cMap, norm=norm)
        self.ax.draw_artist(self.ax.patch)
        self.ax.draw_artist(self.heatmap)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()
        # t_end = time.time()
        plt.pause(1)


    def xy_to_cell(self, loc):
        #convert x and y to new coordinates
        x = loc[0]
        y = loc[1]
        return (int(self.origin_cell[0] + np.round(x / self.cell_size)), int(self.origin_cell[1] + np.round(y / self.cell_size)))


    def add_coord(self, loc:list, id:int):
        #convert the coordinate to cell coordinates
        newloc = self.xy_to_cell(loc)
        if id // 100 == 1:  # then the first number is a 1, making it a robot
            self.grid[newloc[0]][newloc[1]] = 1
        elif id // 100 == 2:  # then the first number is a 2, making it an obstacle
            self.grid[newloc[0]][newloc[1]] = 2



if __name__ == "__main__":


    bodies = [RigidBody(body_id=101, position=Position(1, 2, 3), rotation=Rotation(1, 2, 3, 4)), RigidBody(body_id=102, position=Position(5, 2, 3), rotation=Rotation(1, 2, 3, 4))]

    v = Visualizer()
    v.initialize_heatmap()

    #v.render()