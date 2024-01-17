import queue
from math import inf, sqrt
from heapq import heappop, heappush

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """
    path = []
    boxes = {}

    # source point x and y cords
    spx = source_point[0]
    spy = source_point[1]

    # destination point x and y cords
    dpx = destination_point[0]
    dpy = destination_point[1]

    # boxes that holds source and destination cords
    src_box = None
    dst_box = None

    # identify source and destination box
    for box in mesh['boxes']:
        # kill loop if both found
        if src_box != None and dst_box != None:
            break
        # check if current box is source box
        if (box[0] <= spx and box[1] >= spx) and (box[2] <= spy and box[3] >= spy):
            src_box = box
        # check if current box is destination box
        if (box[0] <= dpx and box[1] >= dpx) and (box[2] <= dpy and box[3] >= dpy):
            dst_box = box
    
    # # invalid selection
    # if src_box == None or dst_box == None:
    #     print("invalid selection for source point or destination point")
    #     return
            
    # adding keys and adj
    try:
        boxes[src_box] = mesh['adj'][src_box]
        boxes[dst_box] = mesh['adj'][dst_box]
    except:
        print("No path!")
        return path, boxes.keys()

    # BFS complete search algo to determine if there is a valid path
    frontier = queue.Queue()
    frontier.put(src_box)
    came_from = dict()
    came_from[src_box] = None
    while not frontier.empty():
        current = frontier.get()

        # if current == dst_box:
        #     break

        for next in mesh['adj'][current]:
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current

    # check for no valid path
    if dst_box not in came_from:
        print("No path!")
        return path, boxes.keys()
    
    # cur = dst_box
    # path = []
    # path.append(destination_point)
    # while cur != src_box:
    #     path.append(dp[cur])
    #     cur = came_from[cur]

    # path.append(source_point)
    # path.reverse()

    path, dp_box = dijkstras_shortest_path(source_point, destination_point, src_box, dst_box, mesh, navigation_edges)

    # path.append(destination_point)
    for b in path:
        b = dp_box[b]
        if b not in boxes:
            boxes[b] = mesh['adj'][b]
    return path, boxes.keys()

def dijkstras_shortest_path(src_p, dest_p, initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    paths = {initial_position: []}          # maps cells to previous cells on path
    pathcosts = {initial_position: 0}       # maps cells to their pathcosts (found so far)
    queue = []
    heappush(queue, (0, initial_position))  # maintain a priority queue of cells
    dp = dict()
    dp_box = dict()
    dp[destination] = dest_p
    dp[initial_position] = src_p
    dp_box[dest_p] = destination
    dp_box[src_p] = initial_position
    
    while queue:
        priority, cell = heappop(queue)
        if cell == destination:
            paths = path_to_cell(cell, paths)
            for i, b in enumerate(paths):
                paths[i] = dp[b]
            return paths, dp_box


        
        # investigate children
        for child in graph['adj'][cell]:
            # find detail points
            ########################################

            # current detail point
            cur_point = dp[cell]
            # box 1 & 2 x ranges
            b1x = (cell[0], cell[1])
            b2x = (child[0], child[1])
            # box 1 & 2 y ranges
            b1y = (cell[2], cell[3])
            b2y = (child[2], child[3])
            # defining x & y ranges
            x_range = (max(b1x[0], b2x[0]), min(b1x[1], b2x[1]))
            y_range = (max(b1y[0], b2y[0]), min(b1y[1], b2y[1]))
            # find detail point of next box (inefficient, can fix later)
            min_dist = float('inf')
            for x in range(x_range[0], x_range[1] + 1):
                for y in range(y_range[0], y_range[1] + 1):
                    temp = (x,y)
                    dist = sqrt(((cur_point[0] - x) ** 2)+ ((cur_point[1] - y) ** 2))
                    if dist < min_dist:
                        detail_point = (x, y)
            dp[child] = detail_point
            dp_box[detail_point] = child
            #########################################
        # calculate cost along this path to child
            cost_to_child = priority + transition_cost(graph, cell, child)
            if child not in pathcosts or cost_to_child < pathcosts[child]:
                pathcosts[child] = cost_to_child # update the cost
                p = cost_to_child + heuristic(destination, child) # adding estimated distance
                paths[child] = cell                         # set the backpointer
                heappush(queue, (p, child))     # put the child on the priority queue
            
    return False

def path_to_cell(cell, paths):
    if cell == []:
        return []
    return path_to_cell(paths[cell], paths) + [cell]
    



def navigation_edges(level, cell):
    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """
    res = []
    for delta in [(x, y) for x in [-1,0,1] for y in [-1,0,1] if not (x==0 and y==0)]:
        new = (cell[0] + delta[0], cell[1] + delta[1])
        if new in level['spaces']:
            res.append((new, transition_cost(level, new, cell)))
    return res

def transition_cost(level, cell, cell2):
    distance = sqrt((cell2[0] - cell[0])**2 + (cell2[1] - cell[1])**2)
    # average_cost = (level['spaces'][cell] + level['spaces'][cell2])/2
    average_cost = 1
    return distance * average_cost

def heuristic(a, b):
    return sqrt((a[0] - b[0]) ** 2 + abs(a[1] - b[1]) ** 2)