import queue
import math

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
    path = []
    dp = dict()
    dp[dst_box] = destination_point
    dp[src_box] = source_point
    while cur != src_box:
        # next box
        next = came_from[cur]
        # current detail point
        cur_point = dp[cur]
        # box 1 & 2 x ranges
        b1x = (cur[0], cur[1])
        b2x = (next[0], next[1])
        # box 1 & 2 y ranges
        b1y = (cur[2], cur[3])
        b2y = (next[2], next[3])
        # defining x & y ranges
        x_range = (max(b1x[0], b2x[0]), min(b1x[1], b2x[1]))
        y_range = (max(b1y[0], b2y[0]), min(b1y[1], b2y[1]))
        # find detail point of next box (inefficient, can fix later)
        min_dist = float('inf')
        for x in range(x_range[0], x_range[1] + 1):
            for y in range(y_range[0], y_range[1] + 1):
                dist = math.sqrt(((cur_point[0] - x) ** 2)+ ((cur_point[1] - y) ** 2))
                if dist < min_dist:
                    detail_point = (x, y)
        dp[next] = detail_point
        path.append(cur_point)
        cur = came_from[cur]

    path.append(source_point)
    path.reverse()

    return path, boxes.keys()
