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
    new_x = cur_point[0]
    new_y = cur_point[1]
    
    # Recalculate the x value of the new detail point if it isn't in range
    if cur_point[0] not in range(x_range[0], x_range[1] + 1):
        if cur_point[0] < x_range[0]:
            new_x = x_range[0]
        else:
            new_x = x_range[1]
            
    # Recalculate the y value of the new detail point if it isn't in range
    if cur_point[1] not in range(y_range[0], y_range[1] + 1):
        if cur_point[1] < y_range[0]:
            new_y = y_range[0]
        else:
            new_y = y_range[1]
    
    new_cords = (new_x, new_y)
    dist = heuristic(cur_point, new_cords)
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
    
    # No path condition
    if (src_box is None) or (dst_box is None):
        print("No path!")
        return path, boxes.keys()

    """
    ####################################################################
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
            break
        cur = came_from[cur]
        
    # Append the last detail point to the path
    path.append(dp[src_box][0])
    # Append the source point to complete the path
    path.append(source_point)
    # Reverse it since we built it from destination to source
    path.reverse()

    """

    dp_path, dp_box = a_star_shortest_path(source_point, destination_point, src_box, dst_box, mesh)
    if not dp_path:
        print("No Path!")
        return [],[]
    return dp_path, list(dp_box)

def a_star_shortest_path(src_p, dest_p, src_box, dest_box, graph):
    """ Searches for a minimal cost path through a graph using the A* algorithm.

    Args:
        src_p: initial point
        dest_p: destination point
        src_box: The initial cell from which the path extends.
        dest_box: The end cell for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.

    Returns:
        If a path exits, return a list containing all cells from src_box to destination.
        Otherwise, return None.

    """
    paths = {src_box: []}          # maps cells to previous cells on path
    pathcosts = {src_box: 0}       # maps cells to their pathcosts (found so far)
    queue = []
    heappush(queue, (0, src_box))  # maintain a priority queue of cells
    dp = dict()
    dp_box = set()
    dp[src_box] = dest_p, 0
    dp[dest_box] = dest_p, 0
    dp_box.add(src_box)
    dp_box.add(dest_box)
    
    while queue:
        priority, cell = heappop(queue)
        if cell == dest_box:
            path = path_to_cell(cell, paths)
            for i, b in enumerate(path):
                path[i] = dp[b][0]
            return path, dp_box
        
        # investigate children
        for child in graph['adj'][cell]:
            # find detail point for each child
            cur_point = dp[cell][0]
            dp[child] = find_detail(cur_point, cell, child)
            
            # calculate cost along this path to child
            cost_to_child = priority + dp[child][1]
            if child not in pathcosts or cost_to_child < pathcosts[child]:
                dp_box.add(child)
                pathcosts[child] = cost_to_child # update the cost
                p = cost_to_child + heuristic(dest_box, child) # adding estimated distance
                paths[child] = cell                         # set the backpointer
                heappush(queue, (p, child))     # put the child on the priority queue
            
    return False, False

def path_to_cell(cell, paths):
    if cell == []:
        return []
    return path_to_cell(paths[cell], paths) + [cell]
    
def heuristic(a, b):
    return euclidean_dist(a, b)

def euclidean_dist(a,b):
    return sqrt(abs(a[0] - b[0]) ** 2 + abs(a[1] - b[1]) ** 2)
