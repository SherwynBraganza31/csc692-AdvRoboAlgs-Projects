import math
import numpy as np

########################################################################
# Class: Node
# (tuple) index - A tuple of the x and y coods.
# (float) traversed_distance - The distance or path length moved since
#                              the start.
# (float) heuristic_distance - The distance from the target (the
#                              Euclidean distance
# (tuple) previous - The previous node/cartesian coods it arrived from
########################################################################
class Node:
    def __init__(self, index, traversed_distance, heuristic_distance, previous):
        self.index = tuple(index)
        self.traversed_distance = float(traversed_distance)
        self.heuristic_distance = float(heuristic_distance)
        self.previous = tuple(previous)

    def __repr__(self):
        return repr((self.index, self.traversed_distance, self.heuristic_distance, self.previous))


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

    # This block checks for neighbors in cardinal and diagonal directions
    cardinal = [(-1, 0), (0, -1), (0, 1), (1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]
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



def a_star(start: tuple, end: tuple, grid: np.matrix) -> list:
    goal_attained = False
    min_heap = list()
    min_heap.append(Node(start, 0, get_distance_heuristic(start, end), (0, 0)))
    finished_stack = list()

    while not goal_attained:

        # try to the get the top
        try:
            working_node = min_heap[0]
        except IndexError:
            goal_attained = False
            break

        # break if you working node is the end node
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
        return []