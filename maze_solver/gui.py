import tkinter
import numpy as np
from grid_generator import *

width = 800
height = 800
offset = 2

########################################################################
# Function: draw_grid()
# Draws the maze and returns a list with object ids to each object
# drawn.
# 0 - Corresponds to open space and is represented by a white block
# -1 - Obstacle, represented by a black block
# 7 - Start, represented by a yellow block
# 8 - End, represented by a green block
# 9 - Path point, represented by a red block
#
# @params: (tkinter.Canvas) canvas - The canvas onto which the objects
#                                    need to be drawn.
#          (np.matrix) grid - The np.matrix containing grid data.
#
# @returns: (list) A list containing the ids to each object.
########################################################################
def draw_grid(canvas:tkinter.Canvas, grid :np.matrix) -> list:
    # get the size of the side for each square. Calculated by dividing canvas width by the max of rows/columns
    square_size = 800/int(grid.shape[0] if grid.shape[0] > grid.shape[1] else grid.shape[1])
    id = []

    for i in range (0, grid.shape[0]):
        for j in range(0, grid.shape[1]):
            homogenized_x = i * square_size + offset # x cood of the (i,j)th square
            homogenized_y = j * square_size + offset# y cood of the (i,j)th square

            # open space
            if grid[i, j] == 0:
                id.append(canvas.create_polygon(homogenized_y,homogenized_x,
                                                homogenized_y,homogenized_x + square_size,
                                                homogenized_y + square_size, homogenized_x + square_size,
                                                homogenized_y + square_size, homogenized_x,
                                                fill="white", outline="black"))
            # obstacle
            elif grid[i, j] == -1:
                id.append(canvas.create_polygon(homogenized_y, homogenized_x,
                                                homogenized_y, homogenized_x + square_size,
                                                homogenized_y + square_size, homogenized_x + square_size,
                                                homogenized_y + square_size, homogenized_x,
                                                fill="black", outline="black"))
            # path point
            elif grid[i, j] == 9:
                id.append(canvas.create_polygon(homogenized_y,homogenized_x,
                                                homogenized_y,homogenized_x + square_size,
                                                homogenized_y + square_size, homogenized_x + square_size,
                                                homogenized_y + square_size, homogenized_x,
                                                fill="red", outline=""))

            # start
            elif grid[i, j] == 7:
                id.append(canvas.create_polygon(homogenized_y, homogenized_x,
                                                homogenized_y, homogenized_x + square_size,
                                                homogenized_y + square_size, homogenized_x + square_size,
                                                homogenized_y + square_size, homogenized_x,
                                                fill="yellow", outline="black"))
            # end
            elif grid[i,j] == 8:
                id.append(canvas.create_polygon(homogenized_y,homogenized_x,
                                                homogenized_y,homogenized_x + square_size,
                                                homogenized_y + square_size, homogenized_x + square_size,
                                                homogenized_y + square_size, homogenized_x,
                                                fill="green", outline="black"))

    return id


########################################################################
# Function: setup_gui
# Sets up the GUI using tkinter
#
# @params: (np.matrix) grid - The np.matrix containing grid data.
#
# @returns: None
########################################################################
def setup_gui(grid:np.matrix) -> None:
    root = tkinter.Tk()
    mainCanvas = tkinter.Canvas(root, bg="white", height=height, width=width)
    id = draw_grid(mainCanvas, grid)
    mainCanvas.pack()
    root.mainloop()

if __name__ == '__main__':
    grid = generate_grid(16,16,0.6)
    setup_gui(grid)
