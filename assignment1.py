import numpy as np


class GridNode:
    def __init__(self, val, heur):
        self.val = val
        self.heur = heur


# temp function to print array of GridNodes
def print_array_grid_nodes(grid):
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            print(f"({grid[i, j].val}, {grid[i, j].heur})", end=" ")
        print()


# Node of position while searching
# possible data: value, fn (g(n)+h(n)), total cost, parent, explored (T or F), ...
class Node:
    pass


# read input.txt and ouput a 2D array of the grid
def read_input_file(input_filename):
    grid = []
    with open(input_filename, "r") as f:
        for l in f:
            t_arr = l.rstrip("\n").replace(" ", "").split(",")
            grid.append(t_arr)

    grid = np.array(grid)
    return grid


# create the heuristic based on the minimum manhattan distance between all goals and a node
def create_heuristic(grid):
    g_coords = np.transpose(np.where(grid == "G"))
    g_h = np.empty(grid.shape, dtype=GridNode)

    for i, j in np.ndindex(grid.shape):
        if grid[i, j] == 'X':
            g_h[i, j] = GridNode('X', np.nan)
            continue

        m_dist = []
        for goal in g_coords:
            m_dist.append(abs(i-goal[0])+abs(j-goal[1]))
        g_h[i, j] = GridNode(grid[i, j], np.min(m_dist))

    return g_h


def pathfinding(input_filename, optimal_path_filename, explored_list_filename):
    # input_filename contains a CSV file with the input grid
    # optimal_path_filename is the name of the file the optimal path should be written to
    # explored_list_filename is the name of the file the list of explored nodes should be written to
    grid_array = read_input_file(input_filename)
    grid = create_heuristic(grid_array)
    print_array_grid_nodes(grid)


input_filename = "Example2/input.txt"
pathfinding(input_filename, "", "")
