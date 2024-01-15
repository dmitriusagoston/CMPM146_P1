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
    
    # adding keys and adj
    boxes[src_box] = mesh['adj'][src_box]
    boxes[dst_box] = mesh['adj'][dst_box]

    return path, boxes.keys()
