import queue
from math import inf, sqrt
from heapq import heappop, heappush

def find_detail(cur_point, box_curr, box_next):
    # box 1 & 2 x ranges
    b1x = (box_curr[0], box_curr[1])
    b2x = (box_next[0], box_next[1])
    # box 1 & 2 y ranges
    b1y = (box_curr[2], box_curr[3])
    b2y = (box_next[2], box_next[3])
    
    # defining x & y ranges
    # The "largest" (rightmost) left bound (x1), to the "smallest" (leftmost) right bound (x2)
    x_range = (max(b1x[0], b2x[0]), min(b1x[1], b2x[1]))
    # The "largest" (lowest) upper bound (y1), to the "smallest" (highest) lower bound (y2)
    y_range = (max(b1y[0], b2y[0]), min(b1y[1], b2y[1]))
    
    # Coordinates of the next detail point (based off current detail point)
    new_x = cur_point[0][0]
    new_y = cur_point[0][1]
    
    # Recalculate the x value of the new detail point if it isn't in range
    if cur_point[0][0] not in range(x_range[0], x_range[1] + 1):
        if cur_point[0][0] < x_range[0]:
            new_x = x_range[0]
        else:
            new_x = x_range[1]
            
    # Recalculate the y value of the new detail point if it isn't in range
    if cur_point[0][1] not in range(y_range[0], y_range[1] + 1):
        if cur_point[0][1] < y_range[0]:
            new_y = y_range[0]
        else:
            new_y = y_range[1]
    
    new_cords = (new_x, new_y)
    dist = heuristic(cur_point[0], new_cords)
    return (new_cords, dist)

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
    
    if (src_box is None) or (dst_box is None):
        print("No path!")
        return path, boxes.keys()

    # adding keys and adj
    boxes[src_box] = mesh['adj'][src_box]
    boxes[dst_box] = mesh['adj'][dst_box]

    # BFS complete search algo to determine if there is a valid path
    frontier = queue.Queue()
    frontier.put(src_box)
    came_from = dict()
    came_from[src_box] = None
    while not frontier.empty():
        current = frontier.get()
        boxes[current] = current

        if current == dst_box:
            break

        for next in mesh['adj'][current]:
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current

    # check for no valid path
    if dst_box not in came_from:
        print("No path!")
        return path, boxes.keys()
    cur = dst_box
    
    # Detail points
    dp = dict()
    dp[dst_box] = destination_point, 0
    #Default dp of src_box is just the destination point for the edge case where the points are in the same box
    dp[src_box] = destination_point, 0

    # section for drawing line
    while cur != src_box:
        # next box
        box_next = came_from[cur]
        # current detail point
        cur_point = dp[cur]
        
        
        
        new_point = find_detail(cur_point, cur, box_next)
        dp[box_next] = new_point
        path.append(cur_point[0])
        if (box_next == src_box):
            last = cur # idk what this is
            break
        cur = came_from[cur]
        
    # Append the last detail point to the path
    path.append(dp[src_box][0])
    # Append the source point to complete the path
    path.append(source_point)
    # Reverse it since we built it from destination to source
    path.reverse()

    #path, dp_box = dijkstras_shortest_path(source_point, destination_point, src_box, dst_box, mesh, navigation_edges)
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
            y_range = (min(b1y[0], b2y[0]), max(b1y[1], b2y[1]))
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
    return euclidean_dist(a, b)

def euclidean_dist(a,b):
    return sqrt(abs(a[0] - b[0]) ** 2 + abs(a[1] - b[1]) ** 2)
