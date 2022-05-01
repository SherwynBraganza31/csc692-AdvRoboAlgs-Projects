import matplotlib.pyplot as plt
import numpy as np

from helpers import *

def GVD_path(grid, GVD, A, B):
    GVD = set(GVD)
    open_set = [A]
    closed_set = set([])
    back_pointers = {}

    while open_set:
        curr = open_set.pop(0)
        neighbors = GVD.intersection(set(get_neighbors(grid, curr[0], curr[1]))) - closed_set
        if B in neighbors:
            back_pointers[B] = curr
            path = [B]
            while path[-1] != A:
                next = back_pointers[path[-1]]
                path.append(next)
            return list(reversed(path))

        open_set += list(neighbors)
        closed_set.update(neighbors)
        back_pointers.update(dict.fromkeys(list(neighbors), curr))

    return None

# Was Testing another way, realized I was attempting to do wavefront
def get_obstacles(grid):
    temp_grid = np.copy(grid)
    n = len(grid)

    def dfs(obstacle, x, y, id):
        obstacle.append((x, y))
        for i, j in [(1,0),(-1,0),(0,1),(0,-1)]:
            if 0 <= x+i < n and 0 <= y+j < n and temp_grid[x+i][y+j] == id:
                obstacle.append((x+i, y+j))
                temp_grid[x+i][y+j] = 0
                obstacle = dfs(obstacle, x+i, y+j, id)
        return obstacle

    id = -1
    obstacles = []
    for i in range(n):
        for j in range(n):
            if temp_grid[i][j] == id:
                obstacle = dfs([], i, j, id)
                obstacles.append(obstacle)
                id -= 1

    return obstacles 

def brushfire_with_gvd(grid, n):
    dist_grid = np.empty_like(grid)
    dist_grid[:] = grid
    back_pointers = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            if dist_grid[i][j] < 0:
                back_pointers[i][j] = dist_grid[i][j]
                dist_grid[i][j] = 1


    plt.title("Distance Matrix - Before Brushfire")
    plt.imshow(dist_grid)
    plt.show()

    queue = []

    for i in range(n):
        for j in range(n):
            if dist_grid[i][j] == 1:
                queue.append((i, j))

    while queue:
        curr = queue.pop(0)
        neighbors = get_neighbors(dist_grid, curr[0], curr[1])
        for neighbor in neighbors:
            if dist_grid[neighbor[0]][neighbor[1]] == 0:
                dist_grid[neighbor[0]][neighbor[1]] = dist_grid[curr[0]][curr[1]] + 1
                back_pointers[neighbor[0]][neighbor[1]] = back_pointers[curr[0]][curr[1]]
                queue.append(neighbor)

    plt.title("Distance Matrix - After Brushfire")
    plt.imshow(dist_grid)
    plt.show()

    plt.title("Back Pointers")
    plt.imshow(back_pointers)
    plt.show()
    

    gvd = []
    for i in range(n):
        for j in range(n):
            neighbors = get_neighbors(dist_grid, i, j)
            visited = []
            for neighbor in neighbors:
                if back_pointers[neighbor[0]][neighbor[1]] not in visited:
                    visited.append(back_pointers[neighbor[0]][neighbor[1]])
            if len(visited) > 1:
                gvd.append((i, j))

    # Get Two Random points from the GVD
    A = gvd[np.random.randint(0, len(gvd))]
    B = gvd[np.random.randint(0, len(gvd))]
    path = GVD_path(grid, gvd, A, B)
    return dist_grid, gvd, path