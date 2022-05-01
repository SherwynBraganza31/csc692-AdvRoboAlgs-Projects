from brushfire import brushfire_with_gvd
from helpers import *
import numpy as np
import matplotlib.pyplot as plt
from aStar import AStarPlanner
from steepest_descent import *

show_animation = True

def main():
    n = 100
    grid = generate_grid(n)
    plt.title("Initial Grid")
    plt.imshow(grid)
    plt.show()
    brush, gvd, path = brushfire_with_gvd(grid, 100)
    plot_gvd(brush, gvd, path)

    #steepest descent
    path = naive_steepest_descent(grid, brush, gvd, 100)
    plot_gvd(brush, gvd, path)

    
if __name__ == '__main__':
    main()
