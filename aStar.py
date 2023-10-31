'''
author: sarah a ibsen
date: 2023- 11- 10 
objective: implement the A* pathfinding algorithm and to use it to find paths in a
pathfinding graph that is input to the program. 


'''
import math
import sys
trace_file = "trace.txt"
#------------------------------------------
# Utility Functions
def write_text(textfile, msg, first=False):
    mode = 'w' if first else 'a'
    with open(textfile, mode) as f:
        f.write(msg + "\n")

def num_width(x, left, right):
    return "{:.{prec}f}".format(round(x, right), prec=right).rjust(left + right + (1 if right > 0 else 0))

# Constants
INFINITY = float('inf')
UNDEFINED = 0
UNVISITED = 1
OPEN = 2
CLOSED = 3

# Node Indexes
STATUS = 3
COSTSOFAR = 4
HEURISTIC = 5
TOTAL = 6
PREVIOUS = 7
LOC_X = 8
LOC_Z = 9

# Connection Indexes
FROM_NODE = 3
TO_NODE = 4
COST = 5
COST_POS = 6
TYPE = 7

# Find shortest path from first node to last node using A* algorithm.
# This implementation uses Node Array A*, described in [Millington, 2019] section 4.3.7 pp. 228-230.

# Find the OPEN node with the lowest total cost.
# If more than one open node has the lowest total cost, the lowest numbered node is returned.
def findLowestTotalCostNode(openNodes):
    lowestCost = math.inf
    lowestCostNode = None
    for node in openNodes:
        if node.totalCost < lowestCost:
            lowestCost = node.totalCost
            lowestCostNode = node
    return lowestCostNode

# Calculate heuristic value using standard Euclidean 2D distance.
def calculateHeuristic(node1, node2):
    return math.sqrt((node1[LOC_X] - node2[LOC_X]) ** 2 + (node1[LOC_Z] - node2[LOC_Z]) ** 2)

# get all outgoing connections from the current node
def getOutgoingConnections(node, connections):
    outgoingConnections = []
    for connection in connections:
        if connection[FROM_NODE] == node:
            outgoingConnections.append(connection)
    return outgoingConnections


# display status of current iteration
def displayStatus(graph, iteration, currentNode = None):
    fill_symbols = [".", "O", "X"]
    fill_display = ["?" for _ in range(len(graph["nodes"]))]
    for idx, node in enumerate(graph):
        fill_display[idx] = fill_symbols[node[STATUS]]
    if currentNode != None:
        fill_display[currentNode - 1] = "C"
    status_line = "    {}     {}     {}     {}     {}".format(
        num_width(iteration, 2, 0),
        num_width(sum(1 for node in graph["nodes"] if node[STATUS] == UNVISITED), 2, 0),
        num_width(sum(1 for node in graph["nodes"] if node[STATUS] == OPEN), 2, 0),
        num_width(sum(1 for node in graph["nodes"] if node[STATUS] == CLOSED), 2, 0),
        "".join(fill_display)
    )
    write_text(trace_file, status_line)

# Find path from start node (first) to goal node (last) using A* 
def findPath(graph, start, goal):
    # Initialize all nodes to UNDEFINED
    for node in graph["nodes"]:
        node[STATUS] = UNDEFINED

    # initialize node array
    for node in graph["nodes"]:
        node.append(UNDEFINED) # status
        node.append(INFINITY) # cost so far
        node.append(INFINITY) # heuristic
        node.append(INFINITY) # total cost
        node.append(UNDEFINED) # previous node
    # Initialize start node
    graph["nodes"][start - 1][STATUS] = OPEN
    graph["nodes"][start - 1][COSTSOFAR] = 0
    graph["nodes"][start - 1][HEURISTIC] = calculateHeuristic(graph["nodes"][start - 1], graph["nodes"][goal - 1])
    graph["nodes"][start - 1][TOTAL] = graph["nodes"][start - 1][COSTSOFAR] + graph["nodes"][start - 1][HEURISTIC]
    graph["nodes"][start - 1][PREVIOUS] = None
    
    # Initialize iteration counter
    iteration = 0

    # Initialize open and closed node lists
    openNodes = [graph["nodes"][start - 1]]
    closedNodes = []

    # Main loop
    while len(openNodes) > 0:
        # Display status
        displayStatus(graph["nodes"], iteration, openNodes[0][0])
        iteration += 1

        # Find node with lowest total cost
        currentNode = findLowestTotalCostNode(openNodes)
        currentNode[STATUS] = CLOSED
        closedNodes.append(currentNode)
        openNodes.remove(currentNode)

        # Check if goal node has been reached
        if currentNode[0] == goal:
            break

        # Get outgoing connections
        outgoingConnections = getOutgoingConnections(currentNode[0], graph["connections"])

        # Process outgoing connections
        for connection in outgoingConnections:
            # Get to node
            # to.cost is sum of COSTSOFAR, cost from start node to to current node,  plus COST, cost of current connection.
            toNode = graph["nodes"][connection[TO_NODE] - 1]
            toNodeCost = currentNode[COSTSOFAR] + connection[COST]

             # If the path to the to node via the current node is lower cost than the previous lowest cost path to the to node,
            # then update the to node's fields to reflect the newly found lower cost path.
            if toNodeCost < toNode[COSTSOFAR]:
                toNode[COSTSOFAR] = toNodeCost
                toNode[HEURISTIC] = calculateHeuristic(toNode, graph["nodes"][goal - 1])
                toNode[TOTAL] = toNode[COSTSOFAR] + toNode[HEURISTIC]
                toNode[PREVIOUS] = currentNode[0]
            
    # Show iteration status and close current node.
        if(len(openNodes) > 0):
            displayStatus(graph["nodes"], iteration, openNodes[0][0])
        graph["nodes"][currentNode[0] - 1][STATUS] = CLOSED
        # the r code for this section is  open.nodes <- setdiff(open.nodes, current.node)
        # remove the current node from the open nodes list
        openNodes.remove(currentNode)
    return graph


#------------------------------------------
# return path nodes from first to last 
def getPath(graph, start, last):
    path = []
    currentNode = last 
    while current != start:
        path.insert(0, current)
        current = graph["nodes"][current - 1][PREVIOUS]

    if current == start:
        path.insert(0, start)
        write_text(trace_file, f"Path from {start} to {last} path= {' '.join(map(str, path))} cost= {graph['nodes'][last - 1][COSTSOFAR]}")
    else:
        path = []
        write_text(trace_file, f"Path from {start} to {last} not found")
    
    return path