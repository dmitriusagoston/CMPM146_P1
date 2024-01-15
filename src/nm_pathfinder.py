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
    print(mesh)
    path = []
    boxes = {}

    src_box = None
    dst_box = None
    for box in mesh['boxes']:
        print(box)

    return path, boxes.keys()
