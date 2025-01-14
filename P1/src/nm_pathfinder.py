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
        path = BFS(start_end_boxes[0], start_end_boxes[1], mesh, source_point, destination_point)
    except: #only runs when invalid inputs(needs 2 boxes), or the other case. 
        print("No path!2")

    return path, boxes.keys()

def BFS(start, goal, mesh, sources, goals):
    queue = [start] 
    TheSet = set([])
    TheDict = {} # {CHILD : PARENT}
    while True:
        node = queue.pop()
        if(node == goal):
            print("SUCCESS")
            path = [node]
            drawnpath = [goals, boxToCenterPoint(node)]
            while path[-1] != start:
                parent = TheDict.get(path[-1])
                #other = boxesToEdgePoint(parent, path[0])
                path.append(parent)
                drawnpath.append(boxToCenterPoint(parent))
                if(len(path) > 500):
                    print("PATH TOO LONG")
                    break
            drawnpath.append(sources)
            return drawnpath 
        else:
            TheSet.add(node)
            for x in mesh["adj"].get(node):
                if(x not in TheSet):
                    queue.append(x)
                    TheDict.update({x:node})
        if(not queue): #checks at the END of the loop if queue is empty.
            break
    print("No path!") #if no other paths exist, this runs after if(not queue)


def boxToCenterPoint(box): #Turns a box into a center point and returns it (x,y). 
    pointone = (box[1] + box[0]) / 2
    pointtwo = (box[2] + box[3]) / 2 
    return [pointone, pointtwo]

def boxesToEdgePoint(box, boxtwo): #Finds a range at the border of two points. Returns the average of the range (x,y).
    pointone = max(box[0], boxtwo[0])
    pointtwo = min(box[1], boxtwo[1])
    pointthree = max(box[2], boxtwo[2])
    pointfour = min(box[3], boxtwo[3])
    return [(pointthree + pointfour)/ 2, (pointone + pointtwo) / 2] 

def euclideanDistance(point, pointtwo): #finds the closest disance between two points. Returns an int.
    return [math.sqrt((point[0] + pointtwo[0])^2 + (point[1] + pointtwo[1])^2)]
