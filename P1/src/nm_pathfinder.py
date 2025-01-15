import math
from heapq import heappush, heappop
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
    """
    #simple search algorithm
    try:
        path = BFS(start_end_boxes[0], start_end_boxes[1], mesh, source_point, destination_point,boxes)
    except: #only runs when invalid inputs(needs 2 boxes), or the other case. 
        print("No path!")
    """
    #A* implemenation
    #try:
    path = AStarMonodirection(source_point, destination_point, mesh["adj"], start_end_boxes[0], start_end_boxes[1], boxes)
    #except:
    #print("No path!")
    print(path)
    return path, boxes.keys()

def BFS(start, goal, mesh, sources, goals,TheDict):
    queue = [start] 
    TheSet = set([])
    #TheDict = {} # {CHILD : PARENT}
    while True:
        node = queue.pop()
        if(node == goal):
            print("SUCCESS")
            path = [node]
            drawnpath = [goals, boxesToEdgePoint(goals,node)]
            while path[-1] != start:
                parent = TheDict.get(path[-1])
                path.append(parent)
                #print(f"PATHPOINT:{drawnpath[-1]}")
                drawnpath.append(boxesToEdgePoint(drawnpath[-1],parent))
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
    return (pointone, pointtwo)

def boxesToEdgePoint(point, box): 
    p1 = (box[0],box[2])
    p2 = (box[1],box[2])
    p3 = (box[0],box[3])
    p4 = (box[1],box[3])
    #print("1")
    dist1 = euclideanDistance(point,p1)
    dist2 = euclideanDistance(point,p2)
    dist3 = euclideanDistance(point,p3)
    dist4 = euclideanDistance(point,p4)
    #print(min(euclideanDistance(point,p1),euclideanDistance(point,p2),euclideanDistance(point,p3),euclideanDistance(point,p4)))
    if(dist1 <= dist2 and dist1 <= dist3 and dist1 <= dist4):
        #print(dist1)
        return p1
    elif(dist2 <= dist1 and dist2 <= dist3 and dist2 <= dist4):
        #print(dist2)
        return p2
    elif(dist3 <= dist1 and dist3 <= dist2 and dist3 <= dist4):
        #print(dist3)
        return p3
    elif(dist4 <= dist1 and dist4 <= dist2 and dist4 <= dist3):
        #print(dist4)
        return p4
    else:
        print("ERROR")

def euclideanDistance(point, pointtwo): #finds the closest disance between two points. Returns an int.
    return [math.sqrt((point[0] - pointtwo[0])**2 + (point[1] - pointtwo[1])**2)]

def AStarMonodirection(start, goal, adj, startbox, endbox, TheDict):
    TheDict = {startbox : []}
    pathcost = {startbox : 0}
    queue = []
    heappush(queue, (0, startbox))
    while queue:
        priority, currentbox = heappop(queue)                         #priority = (int) cell = (x1, x2, y1, y2) celledge = (x, y)                                                                      #cellcenter = (xmid, ymid)
        if(currentbox == endbox): # this checks for whether we are in last box
            TheDict[currentbox] = endbox
            print("SUCCESS")
            print(currentbox)
            print(endbox)
            npath = [startbox]
            print(f"npath:{npath}")
            drawnpath = [start, boxesToEdgePoint(start,TheDict.get(startbox))]
            print(drawnpath)
            while npath[-1] != endbox:
                child = TheDict.get(npath[-1])
                try:
                    print(child)
                except:
                    print("NO CHILD")
                if(child == None):
                    #print(TheDict)
                    print("CHILD DROPPED")
                    return drawnpath
                npath.append(child)
                print(f"PATHPOINT:{drawnpath[-1]}")
                drawnpath.append(boxesToEdgePoint(drawnpath[-1],child))
                if(len(npath) > 500):
                    print("PATH TOO LONG")
                    break
            drawnpath.append(goal)
            return drawnpath 
        else:
            for neighbor in adj[currentbox]:                                           #adj[cel] = [(x1, x2, y1, y2), (x1, x2, y1, y2)...()], x = (x1, x2, y1, y2)
                newcost = priority + euclideanDistance(boxToCenterPoint(currentbox), goal)[0]     #newcost = (int) + (int)                               #newx = [x, y]
                if(neighbor not in pathcost or newcost < pathcost[neighbor]):     #Error here
                    pathcost[neighbor] = newcost
                    TheDict[currentbox] = neighbor
                    #print(f"{TheDict.get(currentbox)}\n\n\n")
                    heappush(queue, (newcost, neighbor))

def path_to_cell(cell, path):
    print (f"{path} \n\n\n")
    if(cell == []):
        return []
    return path_to_cell(path[cell], path) + [cell]
