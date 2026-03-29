import numpy as np

class GridWorld:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = np.zeros((width, height)) # 0 là trống, 1 là vật cản

    def set_obstacle(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[x, y] = 1

    def is_blocked(self, x, y):
        return self.grid[x, y] == 1