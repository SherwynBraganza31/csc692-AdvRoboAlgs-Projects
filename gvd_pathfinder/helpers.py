import numpy as np
import matplotlib.pyplot as plt

def generate_grid(n):
    grid = np.zeros((n, n))
    grid = rectangle(grid, 15, 45, 20, 50, -1)
    grid = circle(grid, 75.0, 30.0, 12, -2)
    grid = circle(grid, 50.0, 75.0, 16, -3)

    for i in range(n):
      grid[i][0] = -4
      grid[i][n - 1] = -4
      grid[0][i] = -4
      grid[n - 1][i] = -4

    return grid

def rectangle(grid, x1, x2, y1, y2, id):
  grid[x1: x2,y1: y2] = id
  return grid

def circle(grid, cx, cy, r, id):
  for x in range(grid.shape[0]):
    for y in range(grid.shape[1]):
      if np.sqrt((x - cx)**2 + (y - cy)**2) < r:
        grid[int(x), int(y)] = id
  return grid

def get_neighbors(grid, i, j):
    neighbors = []

    dirs = [(-1, 0), (0, 1), (1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    for dir in dirs:
        if 0 <= i + dir[0] < grid.shape[0] and 0 <= j + dir[1] < grid.shape[1]:
          neighbors.append((i + dir[0], j + dir[1]))
    
    return neighbors

def get_max_neighbor(neighbors, grid) -> tuple:
    max_val = 0
    max_node = tuple()

    for x in neighbors:
        if grid[x] > max_val:
            max_node = x
            max_val = grid[x]

    return max_node

def get_random_start_and_end(grid):
  n = grid.shape[0]
  start = (np.random.randint(0, n), np.random.randint(0, n))
  end = (np.random.randint(0, n), np.random.randint(0, n))

  while grid[start[0]][start[1]] == -1 or grid[end[0]][end[1]] == -1 and start != end:
    start = (np.random.randint(0, n), np.random.randint(0, n))
    end = (np.random.randint(0, n), np.random.randint(0, n))

  return start, end


def plot_gvd(grid, GVD, path):
  fig, ax = plt.subplots()
  GVD_grid = np.copy(grid)
  if GVD:
    GVD_x, GVD_y = zip(*GVD)
    GVD_grid[GVD_x,GVD_y] = 20

  img1 = ax.imshow(GVD_grid)
  obstacles = GVD_grid.copy()
  obstacles[obstacles < 0] = -2.0
  masked_data = np.ma.masked_where(obstacles > 0, obstacles)
  img2 = ax.imshow(masked_data)
  plt.title("GVD")

  path_x, path_y = zip(*path)
  GVD_grid[path_x,path_y] = 40.0
  grid_path = GVD_grid.copy()
  grid_path = np.ma.masked_where(grid_path != 40.0, grid_path)
  img3 = ax.imshow(grid_path, cmap="cool_r", interpolation="nearest") 

  plt.show()