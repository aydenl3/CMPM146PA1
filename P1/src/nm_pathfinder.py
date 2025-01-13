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
    start_end_boxes = []
    print(f"Source Point:{source_point}\nDestination Point:{destination_point}")

   
    for box in mesh.get("boxes"):
        if (source_point[0] < box[1] and source_point[0] > box[0]) and (source_point[1] < box[3] and source_point[1] > box[2]):
            start_end_boxes.insert(0, box)
        if (destination_point[0] < box[1] and destination_point[0] > box[0]) and (destination_point[1] < box[3] and destination_point[1] > box[2]):
            start_end_boxes.append(box)

    print(start_end_boxes)

    #simple search algorithm
    try:
        BFS(start_end_boxes[0], start_end_boxes[1], mesh)
    except: #only runs when invalid inputs(needs 2 boxes), or the other case. 
        print("No path!")

    return path, boxes.keys()

def BFS(start, goal, mesh):
    queue = [start] 
    TheSet = set([])
    TheDict = {} # {CHILD : PARENT}
    while True:
        node = queue.pop()
        if(node == goal):
            print("SUCCESS")
            path = [node]
            while path[-1] != start:
                parent = TheDict.get(path[-1])
                path.append(parent)
                if(len(path) > 500):
                    print("PATH TOO LONG")
                    break
            print(path)
            return path #Im not sure how to return the correct from start to goal. NEEDS WORK.
        else:
            TheSet.add(node)
            for x in mesh["adj"].get(node):
                if(x not in TheSet):
                    queue.append(x)
                    TheDict.update({x:node})
        if(not queue): #checks at the END of the loop if queue is empty.
            break
    print("No path!") #if no other paths exist, this runs after if(not queue)