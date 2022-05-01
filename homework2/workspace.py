import numpy as np

# Change the grid positions to -1 where the intersect with the obstacles
def add_obstacles_to_grid(obstacles: list, grid: np.matrix, precision:float) -> None:
    rows, cols = grid.shape
    for obs in obstacles:
        for i in range(0, rows):
            for j in range(0, cols):
                if ((i*precision-obs[0])/obs[2]) ** 2 + ((j*precision-obs[1])/obs[3])**2 - 1 <= 0:
                    grid[i, j] = -1
                    obstacle_sub_grid.append((i,j))