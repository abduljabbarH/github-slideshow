import numpy as np
import heapq


# class GridNode:
#     def __init__(self, val, heur):
#         self.val = val
#         self.heur = heur
#
#
# # temp function to print array of GridNodes
# def print_array_grid_node(grid):
#     for i in range(grid.shape[0]):
#         for j in range(grid.shape[1]):
#             print(f"({grid[i, j].value}, {grid[i, j].heuristic})", end=" ")
#         print()
#
#
# class GeneralNode:
#     def __init__(self, parent, value, fcost, cost, x, y, explored=False):
#         self.parent = parent
#         self.value = value
#         self.fcost = fcost
#         self.cost = cost
#         self.explored = explored
#         self.x = x
#         self.y = y
#
#     def __lt__(self, other):
#         return self.fcost < other.fcost


# temp function to print array of GridNodes
def print_array_node(node_list):
    for node in node_list:
        print(f"({node.x}, {node.y})", end=" -> ")
    print()


# Node of position while searching
# possible data: value, fn (g(n)+h(n)), total cost, parent, explored (T or F), ...
class Node:
    def __init__(self, value, heuristic, x, y):
        self.value = value
        self.heuristic = heuristic
        self.x = x
        self.y = y
        self.parent = None
        self.f_cost = None
        self.g_cost = None
        self.frontier = False
        self.explored = False

    def add_to_frontier(self, parent, f_cost, g_cost):
        self.parent = parent
        self.f_cost = f_cost
        self.g_cost = g_cost
        self.frontier = True

    def add_to_explored(self):
        self.frontier = False
        self.explored = True

    def __lt__(self, other):
        return self.f_cost < other.f_cost


# read input.txt and output a 2D array of the grid
def read_input_file(input_filename):
    grid = []
    with open(input_filename, "r") as f:
        for l in f:
            t_arr = l.rstrip("\n").replace(" ", "").split(",")
            grid.append(t_arr)

    grid = np.array(grid)
    return grid


# create the heuristic based on the minimum manhattan distance between all goals and a node
# def create_heuristic(grid):
#     g_coords = np.transpose(np.where(grid == "G"))
#     g_h = np.empty(grid.shape, dtype=GridNode)
#
#     for i, j in np.ndindex(grid.shape):
#         if grid[i, j] == 'X':
#             g_h[i, j] = GridNode('X', np.nan)
#             continue
#
#         m_dist = []
#         for goal in g_coords:
#             m_dist.append(abs(i - goal[0]) + abs(j - goal[1]))
#         g_h[i, j] = GridNode(grid[i, j], np.min(m_dist))
#
#     return g_h

# create the heuristic based on the minimum manhattan distance between all goals and a node
def create_heuristic(grid):
    g_coords = np.transpose(np.where(grid == "G"))
    s_coords = np.transpose(np.where(grid == "S"))[0]

    g_h = np.empty(grid.shape, dtype=Node)

    for i, j in np.ndindex(grid.shape):
        if grid[i, j] == 'X':
            g_h[i, j] = Node('X', np.nan, i, j)
            continue

        m_dist = []
        for goal in g_coords:
            m_dist.append(abs(i - goal[0]) + abs(j - goal[1]))
        g_h[i, j] = Node(grid[i, j], np.min(m_dist), i, j)

    return g_h, s_coords

# Traverse the grid with frontier and explored list in mind
# def traverse_grid(grid):
#     adjacency = np.array([[1, 0], [0, 1], [-1, 0], [0, -1]])
#     # start could be anywhere
#     x = 0
#     y = 0
#     g = grid[x, y].val
#     h = grid[x, y].heur
#     item = GeneralNode(None, g, h, 0, x, y)
#     q = []
#     heapq.heappush(q, (item.fcost, item))
#     explored = []
#
#     while q:
#         current = heapq.heappop(q)
#         print(current[1].value)
#         if current[1].value == "G":
#             explored.append(current[1])
#             print(explored)
#             return explored
#
#         current[1].exlpored = True
#         explored.append(current[1])
#
#         next = adjacency + [current[1].x, current[1].y]
#         for i in next:
#             if i[0] < 0 or i[1] < 0:
#                 continue
#             if i[0] > grid.shape[0] or i[1] > grid.shape[0]:
#                 continue
#             if grid[i[0], i[1]].val == "X":
#                 continue
#             if grid[i[0], i[1]].val == "S":
#                 continue
#             if grid[i[0], i[1]].val == "G":
#                 f = int(current[1].fcost) + int(grid[i[0], i[1]].heur)
#             else:
#                 f = int(current[1].fcost) + int(grid[i[0], i[1]].val) + int(grid[i[0], i[1]].heur)
#
#             item = GeneralNode(current[1], grid[i[0], i[1]].val, f, grid[i[0], i[1]].heur, i[0], i[1])
#             if item in explored:
#                 continue
#
#             heapq.heappush(q, (item.fcost, item))

# Traverse the grid with frontier and explored list in mind
def traverse_grid(grid, s_coords):
    adjacency = np.array([[1, 0], [0, 1], [-1, 0], [0, -1]])

    x, y = s_coords

    grid[x, y].add_to_frontier(None, grid[x, y].heuristic, 0)

    q = []
    heapq.heappush(q, grid[x, y])
    explored = []

    while q:
        current: Node = heapq.heappop(q)
        current.add_to_explored()
        explored.append(current)

        if current.value == "G":
            return explored

        adjacent = adjacency + [current.x, current.y]
        for a in adjacent:
            a_x, a_y = a
           

            if a_x < 0 or a_y < 0:
                continue
            if a_x >= grid.shape[0] or a_y >= grid.shape[0]:
                continue
            
            adj_node = grid[a_x, a_y]

            if adj_node.value == "X":
                continue
            if adj_node.value == "S":
                continue
            if adj_node.explored:
                continue

            gn = int(current.g_cost) + (int(adj_node.value) if adj_node.value != "G" else 0)
            fn = gn + int(adj_node.heuristic)

            # if node is in the frontier and the old f_cost is smaller than the new f_cost, don't update
            if adj_node.frontier:
                if adj_node.f_cost < fn:
                    continue

            adj_node.add_to_frontier(current, fn, gn)
            heapq.heappush(q, adj_node)

def write_explored_list(explored, explored_list_filename):  
    f = open(explored_list_filename, "w")
    f.close()
    f = open(explored_list_filename, "a") 
    for node in explored:
        f.write(f"({node.x}, {node.y})\n")
    f.close()

def write_optimal_path(explored, optimal_path_filename):
    f = open(optimal_path_filename, "w")
    f.close()
    f = open(optimal_path_filename, "a")
    cost = 0
    optimal =[]
    e= explored[-1]
    while 1:
        optimal.insert(0,e)
        if (e.parent== None):
            break
       
        e = e.parent
    
    for node in optimal:
       f.write(f"({node.x}, {node.y})\n")
    f.close()
    

def pathfinding(input_filename, optimal_path_filename, explored_list_filename):
    # input_filename contains a CSV file with the input grid
    # optimal_path_filename is the name of the file the optimal path should be written to
    # explored_list_filename is the name of the file the list of explored nodes should be written to

    # grid_array = read_input_file(input_filename)
    # grid = create_heuristic(grid_array)
    # print_array_grid_node(grid)
    #
    # e = traverse_grid(grid)
    # print_array_node(e)
    # print(e[-1].cost)

    grid_array = read_input_file(input_filename)
    grid, s_coords = create_heuristic(grid_array)
    # print_array_grid_node(grid)

    e = traverse_grid(grid, s_coords)
    write_explored_list(e,explored_list_filename)
    write_optimal_path(e,optimal_path_filename)
    #print_array_node(e)
    print(e[-1].g_cost)

explore_list_filename = "Example1/explored_list_filename.txt"
optimal_path_filename = "Example1/optimal_path_filename.txt"
input_filename = "Example1/input.txt"
pathfinding(input_filename, optimal_path_filename, explore_list_filename)
