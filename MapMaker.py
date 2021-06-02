class MapMaker:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.grid = []

    def display(self):
        print("-------")
        for i in range(self.rows):
            for j in range(self.cols):
                print(self.grid[i][j], end='')
            print()
        print("-------")

    def set_grid(self, grid):
        self.rows = len(grid)
        self.cols = len(grid[0])
        for i in range(self.rows):
            self.grid.append([])
            for j in range(self.cols):
                self.grid[i].append('.')
        print(self.rows, "rows and ", self.cols, "cols")
        for i in range(self.rows):
            for j in range(self.cols):
                if grid[i][j] == 1:
                    self.grid[i][j] = '@'
                elif grid[i][j] == 0:
                    self.grid[i][j] = '.'
                elif grid[i][j] == 2:
                    self.grid[i][j] = 'X'

    def make_file(self, file="my_map.map"):
        f = open(file, "w")
        for i in range(self.rows):
            for j in range(self.cols):
                f.write(str(self.grid[i][j]))
            f.write('\n')
        f.close()
        print("successful")