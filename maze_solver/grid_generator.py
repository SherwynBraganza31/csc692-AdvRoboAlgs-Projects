import string
import numpy as np
import random as rand
from gui import *
from PIL import Image as img

########################################################################
# Function: generate_grid()
# Generates a maze of size specified by width and height and given
# obstacle density
#
# @params: (int) width - Width of the maze
#          (int) height - Height of the maze
#          (float) obstacle_density - A value between 0 and 0.6 that
#                                     gives a rough estimate of how
#                                     much of the maze is obstacles
#
# @returns: (np.matrix) grid - The np.matrix containing grid data.
########################################################################
def generate_grid(width: int, height: int, obstacle_density: float) -> np.matrix:
    grid = np.matrix(np.zeros((width, height), dtype=int))

    # cap obstacle density
    if obstacle_density > 0.6:
        obstacle_density = 0.6

    for i in range(0, width):
        for j in range(0, rand.randint(5, int(width * obstacle_density))):
            grid[i, rand.randint(0, width - 1)] = -1
    return grid

########################################################################
# Function: input_grid_from_image
# Inputs an image and converts the pixel to grid data
#
# @params: (string) img_name - The name of the file to be opened
#
# @returns: (np.matrix) grid - The np.matrix containing grid data.
########################################################################
def input_grid_from_img(img_name: string) -> np.matrix:
    filename = img_name
    maze_image = img.open(filename)
    img_data = np.asarray(maze_image)
    grid = np.zeros((int(maze_image.height / 10), int(maze_image.width / 10)), dtype=int)

    for i in range(0, int(maze_image.width / 10)):
        for j in range(0, int(maze_image.height / 10)):
            if img_data[i * 10, j * 10, 0] > 127 or img_data[i * 10, j * 10, 1] > 127 or \
                    img_data[ i * 10, j * 10, 2] > 127:
                grid[i, j] = 0
            else:
                grid[i, j] = -1

    return np.matrix(grid)


if __name__ == '__main__':
    grid = input_grid_from_img("maze_64x64.png")
    setup_gui(grid)
