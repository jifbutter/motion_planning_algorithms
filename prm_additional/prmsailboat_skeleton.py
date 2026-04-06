'''prmsailboat.py

   This is the PRM skeleton code for the 2D sailboat problem.

   PLEASE FINISH WRITING THE CODE, based on the 2D planner problem.
   See also places marked by FIXME.

   Use the PRM algorithm to find a path consistent with the wind.

'''

import bisect
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math               import inf, pi, sin, cos, sqrt, ceil, dist

from shapely.geometry   import Point, LineString, Polygon, MultiPolygon
from shapely.prepared   import prep


######################################################################
#
#   Parameters
#
#   FIXME: Define the N/K...
#
N = 15
K = 2


######################################################################
#
#   World Definitions (No Fixes Needed)
#
#   List of obstacles/objects as well as the start/goal.
#
(xmin, xmax) = (0, 10)
(ymin, ymax) = (0, 10)

# Collect all the obstackes and prepare (for faster checking).
obstacles = prep(MultiPolygon([
    Polygon([[4,0], [4,4], [6,4], [6,0]]),
    Polygon([[6,10], [8,8], [10,10]]),
    Polygon([[0,6], [0,10], [4,10]]),]))

# Define the start/goal states (x, y, theta)
(xstart, ystart) = ( 8, 4)
(xgoal,  ygoal)  = ( 2, 2)


######################################################################
#
#   Visualization Class (No Fixes Needed)
#
#   This renders the world.  In particular it provides the methods:
#     show(text = '')                   Show the current figure
#     drawNode(node,         **kwargs)  Draw a single node
#     drawEdge(node1, node2, **kwargs)  Draw an edge between nodes
#     drawPath(path,         **kwargs)  Draw a path (list of nodes)
#
class Visualization:
    def __init__(self):
        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.grid(True)
        plt.gca().axis('on')
        plt.gca().set_xlim(xmin, xmax)
        plt.gca().set_ylim(ymin, ymax)
        plt.gca().set_aspect('equal')

        # Show the triangles.
        for poly in obstacles.context.geoms:
            plt.plot(*poly.exterior.xy, 'k-', linewidth=2)

        # Show immediately.
        self.show()

    def show(self, text = ''):
        # Show the plot.
        plt.pause(0.001)
        # If text is specified, print and wait for confirmation.
        if len(text)>0:
            input(text + ' (hit return to continue)')

    def drawNode(self, node, **kwargs):
        plt.plot(node.x, node.y, **kwargs)

    def drawEdge(self, node, neighbor, **kwargs):
        plt.arrow(node.x, node.y,
                  (neighbor.x-node.x), (neighbor.y-node.y),
                  head_width=0.2, head_length=0.1,
                  length_includes_head=True, **kwargs)

    def drawPath(self, path, **kwargs):
        for i in range(len(path)-1):
            self.drawEdge(path[i], path[i+1], **kwargs)


######################################################################
#
#   A* Planning Algorithm - COPY FROM 2D PLANNER PROBLEM
#

def astar(nodes, start, goal):
    # Clear the A* search tree information.
    for node in nodes:
        node.done     = False
        node.seen     = False
        node.parent   = None
        node.creach   = 0
        node.ctogoest = inf

    # Prepare the still empty *sorted* on-deck queue.
    onDeck = []

    # Begin with the start node on-deck.
    start.done     = False
    start.seen     = True
    start.parent   = None
    start.creach   = 0
    start.ctogoest = start.costToGoEst(goal)
    bisect.insort(onDeck, start)

    # Continually expand/build the search tree.
    while True:
        # Make sure we have something pending in the on-deck queue.
        # Otherwise we were unable to find a path!
        if not (len(onDeck) > 0):
            return None

        # Grab the next node (first on deck).
        node = onDeck.pop(0)

        # Mark this node as done and check if the goal is thereby done.
        node.done = True
        if goal.done:
            break

        # Add the neighbors to the on-deck queue (or update)
        for neighbor in node.neighbors:
            # Skip if already done.
            if neighbor.done:
                continue

            # Compute the cost to reach the neighbor via this new path.
            creach = node.creach + node.costToConnect(neighbor)

            # Just add to on-deck if not yet seen (in correct order).
            if not neighbor.seen:
                neighbor.seen     = True
                neighbor.parent   = node
                neighbor.creach   = creach
                neighbor.ctogoest = neighbor.costToGoEst(goal)
                bisect.insort(onDeck, neighbor)
                continue

            # Skip if the previous path to reach (cost) was same or better!
            if neighbor.creach <= creach:
                continue

            # Update the neighbor's connection and resort the on-deck queue.
            # Note the cost-to-go estimate does not change.
            neighbor.parent = node
            neighbor.creach = creach
            onDeck.remove(neighbor)
            bisect.insort(onDeck, neighbor)

    # Build the path.
    path = [goal]
    while path[0].parent is not None:
        path.insert(0, path[0].parent)

    # Return the path.
    return path

#
#   Node Definition - COPY FROM 2D PLANNER PROBLEM
#

class Node():
    #################
    # Initialization:
    def __init__(self, x, y):
        # Define/remember the state/coordinates (x,y).
        self.x = x
        self.y = y

        # Edges = set of neighbors.  This needs to filled in.
        self.neighbors = set()

        # Clear the status, connection, and costs for the A* search tree.
        #   TRUNK:  done = True
        #   LEAF:   done = False, seen = True
        #   AIR:    done = False, seen = False
        self.done     = False
        self.seen     = False
        self.parent   = None
        self.creach   = 0               # Known/actual cost to get here
        self.ctogoest = inf             # Estimated cost to go from here

    ###############
    # A* functions:
    # Actual cost to connect to a neighbor and estimated to-go cost to
    # a distant (goal) node.
    def costToConnect(self, other):
        return self.distance(other)

    def costToGoEst(self, goal):
        return self.distance(goal)

    # Define the "less-than" to enable sorting in A*.  Use total cost estimate.
    def __lt__(self, other):
        return (self.creach + self.ctogoest) < (other.creach + other.ctogoest)

    ################
    # PRM functions:
    # Compute the relative distance to another node.
    def distance(self, other):
        return sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    # Check whether in free space.
    def inFreespace(self):
        
        point = Point(self.x, self.y)
        return obstacles.disjoint(point)

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        
        # Calculate distance using square root formula
        distance = sqrt((other.x - self.x) ** 2 + (other.y - self.y) ** 2)

        # Check how far left sailboat is
        if (other.x - self.x) / distance < cos(120 * pi / 180):
            return False 
        
        line = LineString([(self.x, self.y), (other.x, other.y)])
        return obstacles.disjoint(line)

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<Point %5.2f,%5.2f>" % (self.x, self.y))

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    def intermediate(self, other, alpha):
        return Node(self.x + alpha * (other.x - self.x),
                    self.y + alpha * (other.y - self.y))

#
#   PRM Functions - COPY FROM 2D PLANNER PROBLEM
#

def createNodes(N):
    # Add nodes sampled uniformly across the space.
    nodes = []
    while len(nodes) < N:
        node = Node(random.uniform(xmin, xmax),
                    random.uniform(ymin, ymax))
        if node.inFreespace():
            nodes.append(node)
    return nodes

# Connect to up to K nearest neighbors (classic/standard approach)
def connectNearestNeighbors(nodes, K):
    # Clear any existing neighbors, which are stored as a set.
    for node in nodes:
        node.neighbors = set()

    # Check the neighbors.
    for node in nodes:
        # Examine the K nearest neighbors (ignoring node itself).
        indicies = np.argsort(np.array([node.distance(n) for n in nodes]))
        
        # Define total connections for each node and current neighbor
        total_connect = 0
        neighbor_index = 1

        # Check if total connections has reached K neighbors for given node
        while total_connect < K and neighbor_index < len(nodes):
            neighbor = nodes[indicies[neighbor_index]]
            # Force a undirected graph, so node is also neighbor's neighbor.
            if (neighbor not in node.neighbors) and (node.connectsTo(neighbor)):
                node.neighbors.add(neighbor)
                # neighbor.neighbors.add(node)
            neighbor_index += 1     # Move to next neighbor in list

# Compute the path cost
def pathCost(path):
    cost = 0
    for i in range(1, len(path)):
        cost += path[i-1].costToConnect(path[i])
    return cost

# Post process the path
def postProcess(path):
    # FIXME: This is just a placeholder.  You should replace with
    # actual processing.
    
    i = 1
    while i < (len(path) - 1):      # Don't include the start and goal nodes
        predecessor = path[i - 1]
        successor = path[i + 1]
        if (predecessor.connectsTo(successor)):
            path.pop(i)     # Pop the node, but don't increment i
                            # since next node will fall into its place
        else:
            i += 1

    finalpath = path
    return finalpath

#
#  Main Code - COPY FROM 2D PLANNER PROBLEM
#

def main():
    # Report the parameters.
    
    print('Running with', N, 'nodes and', K, 'neighbors.')

    # Create the figure.  Some computers seem to need an additional show()?
    visual = Visualization()
    visual.show()

    # Create the start/goal nodes.
    startnode = Node(xstart, ystart)
    goalnode  = Node(xgoal,  ygoal)

    # Show the start/goal nodes.
    visual.drawNode(startnode, color='orange', marker='o')
    visual.drawNode(goalnode,  color='purple', marker='o')
    visual.show("Showing basic world")


    # Create the list of nodes.
    print("Sampling the nodes...")
    tic = time.time()
    nodes = createNodes(N)
    toc = time.time()
    print("Sampled the nodes in %fsec." % (toc-tic))

    # Show the sample nodes.
    for node in nodes:
        visual.drawNode(node, color='k', marker='x')
    visual.show("Showing the nodes")

    # Add the start/goal nodes.
    nodes.append(startnode)
    nodes.append(goalnode)


    # Connect to the nearest neighbors.
    print("Connecting the nodes...")
    tic = time.time()
    connectNearestNeighbors(nodes, K)
    toc = time.time()
    print("Connected the nodes in %fsec." % (toc-tic))

    # Show the neighbor connections.
    for (i,node) in enumerate(nodes):
        for neighbor in node.neighbors:
            if neighbor not in nodes[:i]:
                visual.drawEdge(node, neighbor, color='g', linewidth=0.5)
    visual.show("Showing the full graph")


    # Run the A* planner.
    print("Running A*...")
    tic = time.time()
    path = astar(nodes, startnode, goalnode)
    toc = time.time()
    print("Ran A* in %fsec." % (toc-tic))

    # If unable to connect, show the part explored.
    if not path:
        print("UNABLE TO FIND A PATH")
        # print(f"Number of connectsTo() calls: {connect_count}")
        for node in nodes:
            if node.done:
                visual.drawNode(node, color='r', marker='o')
        visual.show("Showing DONE nodes")
        return

    # Show the path.
    cost = pathCost(path)
    visual.drawPath(path, color='r', linewidth=2)
    visual.show("Showing the raw path (cost/length %.1f)" % cost)


    # Post process the path.
    finalpath = postProcess(path)

    # Show the post-processed path.
    cost = pathCost(finalpath)
    visual.drawPath(finalpath, color='b', linewidth=2)
    visual.show("Showing the post-processed path (cost/length %.1f)" % cost)
    # print(f"Number of connectsTo() calls: {connect_count}")

if __name__== "__main__":
    main()
