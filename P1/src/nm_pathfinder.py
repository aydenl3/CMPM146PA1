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
    try:
        path = AStarBidirection(source_point, destination_point, mesh["adj"], start_end_boxes[0], start_end_boxes[1], boxes)
    except:
        print("No path!")
    return path, boxes.keys()

def BFS(start, goal, mesh, sources, goals,TheDict):
    queue = [start] 
    TheSet = set([])
    while True:
        node = queue.pop()
        if(node == goal):
            print("SUCCESS")
            path = [node]
            drawnpath = [goals, boxesToEdgePoint(goals,node)]
            while path[-1] != start:
                parent = TheDict.get(path[-1])
                path.append(parent)
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
    dist1 = euclideanDistance(point,p1)
    dist2 = euclideanDistance(point,p2)
    dist3 = euclideanDistance(point,p3)
    dist4 = euclideanDistance(point,p4)
    if(dist1 <= dist2 and dist1 <= dist3 and dist1 <= dist4):
        return p1
    elif(dist2 <= dist1 and dist2 <= dist3 and dist2 <= dist4):
        return p2
    elif(dist3 <= dist1 and dist3 <= dist2 and dist3 <= dist4):
        return p3
    elif(dist4 <= dist1 and dist4 <= dist2 and dist4 <= dist3):
        return p4
    else:
        print("ERROR")

def euclideanDistance(point, pointtwo): #finds the closest disance between two points. Returns an int.
    return [math.sqrt((point[0] - pointtwo[0])**2 + (point[1] - pointtwo[1])**2)]

def AStarMonodirection(start, goal, adj, startbox, endbox, TheDict):
    TheDict.update({startbox: None}) 
    pathcost = {startbox: 0}
    queue = []
    heappush(queue, (0, startbox))

    while queue:
        priority, currentbox = heappop(queue)

        # If we reach the end box
        if currentbox == endbox:
            print("SUCCESS")
            return reconstruct_path(TheDict, startbox, endbox, start, goal)

        # Else explore neighbors
        for neighbor in adj[currentbox]:
            step_cost = euclideanDistance(boxToCenterPoint(currentbox), boxToCenterPoint(neighbor))[0]
            newcost = pathcost[currentbox] + step_cost

            if neighbor not in pathcost or newcost < pathcost[neighbor]:
                pathcost[neighbor] = newcost
                priority = newcost + euclideanDistance(boxToCenterPoint(neighbor), boxToCenterPoint(endbox))[0]
                heappush(queue, (priority, neighbor))
                TheDict[neighbor] = currentbox

    # If no path is found
    print("NO PATH FOUND")
    return []

def reconstruct_path(TheDict, startbox, endbox, start, goal):
    path = [goal]
    current = endbox
    if(startbox == endbox):
        path.append(start)
        return path
    path.append(boxesToEdgePoint(start, endbox))
    while current != startbox:
        path.append(boxesToEdgePoint(path[-1], current))
        current = TheDict[current]
    path.append(boxesToEdgePoint(goal, startbox))
    path.append(start)
    return path


def path_to_cell(cell, path):#does not work with our implementaion
    print (f"{path} \n\n\n")
    if(cell == []):
        return []
    return path_to_cell(path[cell], path) + [cell]

def AStarBidirection(start, goal, adj, startbox, endbox, TheDict):
    TheDicttwo = TheDict.copy()
    TheDict.update({startbox: None}) 
    TheDicttwo.update({endbox: None})
    pathcost = {startbox: 0}
    pathcostother = {endbox: 1}#this is one otherwise it breaks. I dont know why
    queue = []
    heappush(queue, (0, startbox, "Destination"))
    heappush(queue, (0, endbox, "Start"))

    while queue:
        priority, currentbox, direction = heappop(queue)

        if currentbox in TheDict and currentbox in TheDicttwo:
            return reconstruct_path_two(TheDict, startbox, endbox, start, goal, TheDicttwo, currentbox)


        # Else explore neighbors
        for neighbor in adj[currentbox]:
            if(direction == "Destination"):
                step_cost = euclideanDistance(boxToCenterPoint(currentbox), boxToCenterPoint(neighbor))[0]
                newcost = pathcost[currentbox] + step_cost
                if neighbor not in pathcost or newcost < pathcost[neighbor]:
                    pathcost[neighbor] = newcost
                    priority = newcost + euclideanDistance(boxToCenterPoint(neighbor), boxToCenterPoint(endbox))[0]
                    heappush(queue, (priority, neighbor, "Destination"))
                    TheDict[neighbor] = currentbox
            else:
                step_cost = euclideanDistance(boxToCenterPoint(currentbox), boxToCenterPoint(neighbor))[0]
                newcost = pathcostother[currentbox] + step_cost
                if neighbor not in pathcostother or newcost < pathcostother[neighbor]:
                    pathcostother[neighbor] = newcost
                    priority = newcost + euclideanDistance(boxToCenterPoint(neighbor), boxToCenterPoint(startbox))[0]
                    heappush(queue, (priority, neighbor, "Start"))
                    TheDicttwo[neighbor] = currentbox
            
            

    # If no path is found
    print("NO PATH FOUND")
    return []

    
def reconstruct_path_two(TheDict, startbox, endbox, start, goal, TheDicttwo, temp):
    path = [start]
    pathtwo = [boxesToEdgePoint(start, temp)]
    current = temp
    currenttwo = temp
    if(startbox == endbox):
        path.append(goal)
        return path
    while currenttwo != None:
        if(current != None):
            path.append(boxesToEdgePoint(path[-1], current))
            current = TheDict[current]
        elif(currenttwo != None):
            pathtwo.append(boxesToEdgePoint(pathtwo[-1], currenttwo))
            currenttwo = TheDicttwo[currenttwo]
    path.reverse()              #this section is because path is actually reversed.
    path = path[:-1]
    path.insert(0, start)
    path = path + pathtwo

    path.append(goal)
    return path 
