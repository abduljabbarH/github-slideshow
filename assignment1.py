import numpy as np
import heapq


class GridNode:
    def __init__(self, val, heur):
        self.val = val
        self.heur = heur


# temp function to print array of GridNodes
def print_array_grid_node(grid):
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            print(f"({grid[i, j].val}, {grid[i, j].heur})", end=" ")
        print()


class GeneralNode:
    def __init__(self, parent, value, fcost, cost, x, y, explored=False):
        self.parent = parent
        self.value = value
        self.fcost = fcost
        self.cost = cost
        self.explored = explored
        self.x = x
        self.y = y

    def __lt__(self, other):
        return self.fcost < other.fcost


# temp function to print array of GridNodes
def print_array_general_node(node_list):
    for node in node_list:
        print(f"{node.value}", end=" -> ")


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
            m_dist.append(abs(i - goal[0]) + abs(j - goal[1]))
        g_h[i, j] = GridNode(grid[i, j], np.min(m_dist))

    return g_h


# Traverse the grid with frontier and explored list in mind
def traverse_grid(grid):
    adjacency = np.array([[1, 0], [0, 1], [-1, 0], [0, -1]])
    # start could be anywhere
    x = 0
    y = 0
    g = grid[x, y].val
    h = grid[x, y].heur
    item = GeneralNode(None, g, h, 0, x, y)
    q = []
    heapq.heappush(q, (item.fcost, item))
    explored = []

    while q:
        current = heapq.heappop(q)
        print(current[1].value)
        if current[1].value == "G":
            explored.append(current[1])
            print(explored)
            return explored

        current[1].exlpored = True
        explored.append(current[1])

        next = adjacency + [current[1].x, current[1].y]
        for i in next:
            if i[0] < 0 or i[1] < 0:
                continue
            if i[0] > grid.shape[0] or i[1] > grid.shape[0]:
                continue
            if grid[i[0], i[1]].val == "X":
                continue
            if grid[i[0], i[1]].val == "S":
                continue
            if grid[i[0], i[1]].val == "G":
                f = int(current[1].fcost) + int(grid[i[0], i[1]].heur)
            else:
                f = int(current[1].fcost) + int(grid[i[0], i[1]].val) + int(grid[i[0], i[1]].heur)

            item = GeneralNode(current[1], grid[i[0], i[1]].val, f, grid[i[0], i[1]].heur, i[0], i[1])
            if item in explored:
                continue

            heapq.heappush(q, (item.fcost, item))


def pathfinding(input_filename, optimal_path_filename, explored_list_filename):
    # input_filename contains a CSV file with the input grid
    # optimal_path_filename is the name of the file the optimal path should be written to
    # explored_list_filename is the name of the file the list of explored nodes should be written to
    grid_array = read_input_file(input_filename)
    grid = create_heuristic(grid_array)
    print_array_grid_node(grid)

    e = traverse_grid(grid)
    print_array_general_node(e)


input_filename = "Example2/input.txt"
pathfinding(input_filename, "", "")
