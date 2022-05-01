import math
from gui import *
from grid_generator import *
import sys


########################################################################
# Class: Node
# (tuple) index - A tuple of the x and y coods.
# (float) traversed_distance - The distance or path length moved since
#                              the start.
# (float) heuristic_distance - The distance from the target (the
#                              Euclidean distance
# (tuple) previous - The previous node/cartesian coods it arrived from
#
########################################################################
class Node:
    def __init__(self, index, traversed_distance, heuristic_distance, previous):
        self.index = tuple(index)
        self.traversed_distance = float(traversed_distance)
        self.heuristic_distance = float(heuristic_distance)
        self.previous = tuple(previous)

    def __repr__(self):
        return repr((self.index, self.traversed_distance, self.heuristic_distance, self.previous))


# # finds the start position aka the position marked by a 1
# # returns -1,-1 if no start position exists
# def locate_start(grid: np.matrix) -> tuple:
#     for i in range(0, 16):
#         for j in range(0, 16):
#             if grid[i, j] == 1:
#                 return (i, j)
#     return -1, -1
#
#
# # finds the end position aka the position marked by a 2
# # returns -1,-1 if no end position exists
# def locate_end(grid: np.matrix) -> tuple:
#     for i in range(0, 16):
#         for j in range(0, 16):
#             if grid[i, j] == 2:
#                 return i, j
#     return -1, -1


########################################################################
# Function: locate_neighbors()
# Locates the neighbors of the cell being considered. In map terms,
# locating all the adjacent nodes. Treats obstacles as foreigners
#
# @params: (tuple) index - The cartesian coods of the node being considered.
#          (np.matrix) grid - The grid containing all the data
#
# @returns: list - A list containing the coods of all the nighbors
########################################################################
def locate_neighbors(index: tuple, grid: np.matrix) -> list:
    neighbors = []
    width, height = grid.shape

    # Consider diagonal blocks as neighbors
    # for i in range(-1,2):
    #     for j in range(-1,2):
    #         if grid[index[0]+i, index[1]+j] != -1 and (not(i == 0 and j == 0)) \
    #                 and (index[0]+i >= 0) and (index[0]+i < 16) and (index[1]+j >= 0) and (index[1]+j < 16):
    #                 neighbors.append((index[0]+i, index[1]+j))

    # This block checks for neighbors only in the cardinal directions
    cardinal = [(-1, 0), (0, -1), (0, 1), (1, 0)]
    for x in cardinal:
        if 0 <= index[0] + x[0] < width and 0 <= index[1] + x[1] < height:
            if grid[index[0] + x[0], index[1] + x[1]] != -1:
                neighbors.append((index[0] + x[0], index[1] + x[1]))
    return neighbors


########################################################################
# Function: get_heuristic_distance()
# Calculates the Euclidean distance from the current position (index) to
# the target.
# @params: (tuple) index - The cartesian coods of the node being considered.
#          (tuple) end - The cartesian coods of the end node
#
# @returns: float - The Euclidean distance between the current and end
########################################################################
def get_distance_heuristic(index: tuple, end: tuple) -> float:
    return math.sqrt((index[0] - end[0]) ** 2 + (index[1] - end[1]) ** 2)


########################################################################
# Function: find_node_by_index()
# Finds the index of the node in the min_heap that corresponds to the
# index given
#
# @params: (Node) node - The index of the node to be found
#          (list) min_heap - The min_heap in which the index needs
#                            to be found.
# @returns int - the index if found. -1 if not.
########################################################################
def find_node_by_index(node: Node, min_heap: list) -> int:
    for i in range(0, len(min_heap)):
        if node.index == min_heap[i].index:
            return i
    return -1


########################################################################
# Function: get_whole_path()
# Retraces the shortest path from the end to the start and returns
# a list containing it.
#
# @params (tuple) start: The start node coods
#         (list) stack: A list containing all the nodes on which the
#                       the A* alg was performed
#
# @returns:
########################################################################
def get_whole_path(start: tuple, stack: list) -> list:
    path = list()
    curr_node = stack[-1]
    while curr_node.index != start:
        path.append(curr_node.index)
        for i in stack:
            if i.index == curr_node.previous:
                curr_node = i
                break

    path.append(start)
    return path


def a_star(start: tuple, end: tuple, grid: np.matrix):
    goal_attained = False
    min_heap = list()
    min_heap.append(Node(start, 0, get_distance_heuristic(start, end), (0, 0)))
    finished_stack = list()

    iteration = 0
    while not goal_attained:
        iteration = iteration + 1
        if iteration == 26:
            iteration = 26
        try:
            working_node = min_heap[0]
        except IndexError:
            goal_attained = False
            break

        if working_node.index == end:
            goal_attained = True
            finished_stack.append(working_node)
            break
        neighbors = locate_neighbors(working_node.index, grid)
        for i in neighbors:
            neighbor_node = Node(i, working_node.traversed_distance + 1,
                                 get_distance_heuristic(i, end) + working_node.traversed_distance + 1,
                                 working_node.index)
            # check if node is already in min heap
            neighbor_node_idx = find_node_by_index(neighbor_node, min_heap)
            if neighbor_node_idx == -1 and find_node_by_index(neighbor_node, finished_stack) == -1:
                min_heap.append(neighbor_node)
            else:
                if neighbor_node_idx != -1 and \
                        min_heap[neighbor_node_idx].traversed_distance > neighbor_node.traversed_distance:
                    min_heap[neighbor_node_idx] = neighbor_node

        finished_stack.append(min_heap.pop(0))
        # resorting the list to make it a min heap
        min_heap = sorted(min_heap, key=lambda node: node.heuristic_distance, reverse=False)

    if goal_attained:
        return get_whole_path(start, finished_stack)
    else:
        return None


if __name__ == '__main__':

    if len(sys.argv) < 2:
        print("Program run in incorrect format.\n"
              "Correct format:\n"
              "1) a_star_alg.py rand\n"
              "2) a_star_alg.py <filename>\n")
        sys.exit(1)

    filename = list(sys.argv)

    if filename[1] == 'rand':
        width = 64
        height = 64
        grid = generate_grid(width, height, 0.6)

        start = (0,0)
        grid[start] = 0

        end = (33,33)
        grid[end] = 0
        while grid[end[0], end[1]] == -1:
            end = (rand.randint(0, width-1), rand.randint(0, height-1))
        grid[end] = 0

    else:
        grid = input_grid_from_img(filename[1])
        width, height = grid.shape[0] - 1, grid.shape[1] - 1
        start = (1, 0)
        grid[start] = 0
        end = width-1, height
        grid[end] = 0

    path = a_star(start, end, grid)

    if path is not None:
        for i in path:
            grid[i[0], i[1]] = 9

        grid[start] = 7
        grid[end] = 8
        setup_gui(grid)
    else:
        print("Endpoint cant be reached")
        grid[start] = 7
        grid[end] = 8
        setup_gui(grid)
