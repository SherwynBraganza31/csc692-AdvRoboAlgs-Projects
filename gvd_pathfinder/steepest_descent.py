from brushfire import *
from helpers import *

def naive_steepest_descent(grid, brush, gvd, n):
    ################### Picking Start and End #######################
    start = tuple()
    is_obstacle = True
    while is_obstacle:
        start = (np.random.randint(0, n), np.random.randint(0, n))
        if grid[start] >= 0 and not(start in gvd):
            is_obstacle = False

    is_obstacle = True
    end = tuple()
    while is_obstacle:
        end = (np.random.randint(0, n), np.random.randint(0, n))
        if grid[end] >= 0 and not(end in gvd):
            is_obstacle = False



    # finding the steepest descent from the start to a point on the gvd
    on_gvd = False
    start_descent = [start]
    while not on_gvd:
        working_node = start_descent[-1]
        if start_descent[-1] in gvd:
            on_gvd = True
            break
        start_descent.append(get_max_neighbor(get_neighbors(grid, working_node[0], working_node[1]), brush))

    on_gvd = False
    end_descent = [end]
    while not on_gvd:
        working_node = end_descent[-1]
        if end_descent[-1] in gvd:
            on_gvd = True
            break
        end_descent.append(get_max_neighbor(get_neighbors(grid, working_node[0], working_node[1]), brush))


    ################### Pathfinding #######################
    path = GVD_path(grid, gvd, start_descent[-1], end_descent[-1])
    start_descent.pop(-1)
    end_descent.pop(-1)

    try:
        start_descent.extend(path)
        start_descent.extend(reversed(end_descent))
        path = start_descent
    except Exception:
        print("Empty GVD path")

    return path