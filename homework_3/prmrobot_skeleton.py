'''prmrobot.py

   This is the PRM skeleton code for the robot planning problem.

   PLEASE FINISH WRITING THE CODE, especially where marked by FIXME.

'''

import bisect
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math               import inf, pi, sin, cos, sqrt, ceil, dist

from shapely.geometry   import Point, LineString, Polygon
from shapely.geometry   import MultiLineString, MultiPolygon
from shapely.prepared   import prep

from vandercorput       import vandercorput


######################################################################
#
#   Parameters
#
#   FIXME: Define the N/K...
#
N = 20
K = 2

# Decide whether to allow unlimited joints (360deg is same as 0deg)
UNLIMITEDJOINTS = True

# Define the robot dimensions
L1 = 1.0        # Base link
L2 = 1.0        # Middle link
L3 = 1.0        # Tip link

# Goal is 0.5 * sqrt(2) = 0.35, so distance to wall has to be < 0.35
DISTANCETOWALL = 0.15
STEPSIZE       = 0.05


######################################################################
#
#   Angle Wrapping Utilities (No Fixes Needed)
#

# Angle Wrap Utility.  Return the angle wrapped into +/- 1/2 of full range.
def wrap(angle, fullrange):
    return angle - fullrange * round(angle/fullrange)

def wrap180(angle):  return wrap(angle,   pi)
def wrap360(angle):  return wrap(angle, 2*pi)


######################################################################
#
#   World Definitions (No Fixes Needed)
#
#   List of obstacles/objects as well as the start/goal.
#
(xmin, xmax) = (-3.5, 3.5)
(ymin, ymax) = (-3.5, 3.5)

xlabels = (-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5)
ylabels = (-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5)

# Collect all the walls
walls = MultiLineString([[[ 0.5,  3.5], [0.5,  1.5], [1.0,  1.0], [3.5, 1.0]],
                         [[ 3.5, -1.0], [1.5, -1.0], [3.5, -3.0]],
                         [[-3.5,  1.0], [1.5, -3.5]]])

# Define the start/goal states (joint values)
(q1start, q2start, q3start) = ( pi/2, 0, 0)
(q1goal,  q2goal,  q3goal)  = (-pi/4, 0, 0)

# Define the drawing dimensions.
STEPSIZEDRAW = 0.5      # Joint step size when drawing the paths


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
# Visualization Class.
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
        plt.gca().set_xticks(xlabels)
        plt.gca().set_yticks(ylabels)
        plt.gca().set_aspect('equal')

        # Show the walls.
        for l in walls.geoms:
            plt.plot(*l.xy, color='k', linewidth=2)

        # Place joint 0 only once!
        plt.gca().add_artist(plt.Circle((0,0),color='k',radius=0.05))

        # Show.
        self.show()

    def show(self, text = ''):
        # Show the plot.
        plt.pause(0.001)
        # If text is specified, print and wait for confirmation.
        if len(text)>0:
            input(text + ' (hit return to continue)')

    def drawTip(self, node, **kwargs):
        plt.arrow(node.xB, node.yB,
                  0.9*(node.xC-node.xB), 0.9*(node.yC-node.yB),
                  head_width=0.1, head_length=0.1, **kwargs)

    def drawRobot(self, node, **kwargs):
        plt.plot([0, node.xA, node.xB], [0, node.yA, node.yB], **kwargs)
        self.drawTip(node, **kwargs)
        kwargs['radius']=0.05
        plt.gca().add_artist(plt.Circle((node.xA,node.yA), **kwargs))
        plt.gca().add_artist(plt.Circle((node.xB,node.yB), **kwargs))

    def drawNode(self, node, **kwargs):
        self.drawTip(node, zorder=10, **kwargs)

    def drawEdge(self, node1, node2, **kwargs):
        plt.plot([(node1.xB + node1.xC)/2, (node2.xB + node2.xC)/2],
                 [(node1.yB + node1.yC)/2, (node2.yB + node2.yC)/2], **kwargs)

    def drawPath(self, path, **kwargs):
        # Choose the larger step size to keep the drawings clean.
        step = max(STEPSIZE, STEPSIZEDRAW)
        for i in range(len(path)-1):
            n = ceil(path[i].distance(path[i+1]) / step)
            for j in range(n):
                node = path[i].intermediate(path[i+1], j/n)
                self.drawRobot(node, zorder=20, **kwargs)
                plt.pause(0.1)
        self.drawRobot(path[-1], zorder=20, **kwargs)


######################################################################
#
#   A* Planning Algorithm (No Fixes Needed)
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


######################################################################
#
#   Node Definition (to be fixed - see FIXME)
#
class Node():
    def __init__(self, q1, q2, q3):
        # Define/remember the state (joint angles).
        (self.q1, self.q2, self.q3) = (q1, q2, q3)

        # Pre-compute the link positions.
        self.xA = L1*cos(q1)
        self.yA = L1*sin(q1)
        self.xB = L1*cos(q1) + L2*cos(q1+q2)
        self.yB = L1*sin(q1) + L2*sin(q1+q2)
        self.xC = L1*cos(q1) + L2*cos(q1+q2) + L3*cos(q1+q2+q3)
        self.yC = L1*sin(q1) + L2*sin(q1+q2) + L3*sin(q1+q2+q3)
        self.links = LineString(
            [[0,0], [self.xA,self.yA], [self.xB,self.yB], [self.xC, self.yC]])

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

    def costToGoEst(self, other):
        return self.distance(other)

    # Define the "less-than" to enable sorting in A*.  Use total cost estimate.
    def __lt__(self, other):
        return (self.creach + self.ctogoest) < (other.creach + other.ctogoest)

    ################
    # PRM functions:
    # Compute the relative distance to another node.
    def distance(self, other):
        # FIXME: Compute and return the distance (used to sort).
        
        # If unlimited joints, wrap around 360 degrees
        # else, just find difference between q's
        if UNLIMITEDJOINTS:
            dist1 = wrap360(self.q1 - other.q1)
            dist2 = wrap360(self.q2 - other.q2)
            dist3 = wrap360(self.q3 - other.q3)
        else:
            dist1 = self.q1 - other.q1
            dist2 = self.q2 - other.q2
            dist3 = self.q3 - other.q3

        return sqrt(dist1 ** 2 + dist2 ** 2 + dist3 ** 2)

    # Check whether in free space.
    def inFreespace(self):
        return (walls.distance(self.links) > DISTANCETOWALL)

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        n = ceil(self.distance(other) / STEPSIZE)
        for delta in vandercorput(1/n):
            if not self.intermediate(other, delta).inFreespace():
                return False
        return True

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<Joints %6.1fdeg,%6.1fdeg,%6.1fdeg>" %
                (self.q1 * 180/pi, self.q2 * 180/pi, self.q3 * 180/pi))

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    # If allowing multiple joint rotations, this should wrap the angle
    # difference to +/-180deg.
    def intermediate(self, other, alpha):
        # FIXME: Compute and return the distance (used to sort).

        if UNLIMITEDJOINTS:
            new_q1 = self.q1 + alpha * wrap360(other.q1 - self.q1)
            new_q2 = self.q2 + alpha * wrap360(other.q2 - self.q2)
            new_q3 = self.q3 + alpha * wrap360(other.q3 - self.q3)
        else:
            new_q1 = self.q1 + alpha * (other.q1 - self.q1)
            new_q2 = self.q2 + alpha * (other.q2 - self.q2)
            new_q3 = self.q3 + alpha * (other.q3 - self.q3)

        return Node(new_q1, new_q2, new_q3)

######################################################################
#
#   PRM Functions (to be fixed - see FIXME)
#
# Create the list of nodes.
def createNodesUniform(N):
    # FIXME: create, similar to previous code

    # Add nodes sampled uniformly across the space.
    nodes = []
    while len(nodes) < N:
        node = Node(random.uniform(-pi, pi),
                    random.uniform(-pi, pi),
                    random.uniform(-pi, pi))
        if node.inFreespace():
            nodes.append(node)
    return nodes


# Connect to K neighbors (from all, testing nearest first)
def connectKNeighbors(nodes, K):
    # FIXME: copy from the previous planner code

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
                neighbor.neighbors.add(node)
                total_connect += 1      # If connection successful,
                                        # increase number of connections by 1
            neighbor_index += 1     # Move to next neighbor in list

# Compute the path cost
def pathCost(path):
    # FIXME: copy from the previous planner code

    cost = 0
    for i in range(1, len(path)):
        cost += path[i-1].costToConnect(path[i])
    return cost

# Post process the path
def postProcess(path):
    # FIXME: copy from the previous planner code

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


######################################################################
#
#  Run/Main Code (No Fixes Needed)
#
def run():
    # Report the parameters.
    print('Running with', N, 'nodes and', K, 'neighbors.')

    # Create the figure.  Some computers seem to need an additional show()?
    visual = Visualization()
    visual.show()

    # Create the start/goal nodes.
    startnode = Node(q1start, q2start, q3start)
    goalnode  = Node(q1goal,  q2goal,  q3goal)

    # Show the start/goal nodes.
    visual.drawRobot(startnode, color='orange', linewidth=2)
    visual.drawRobot(goalnode,  color='purple', linewidth=2)
    visual.show("Showing basic world")


    # Create the list of sample points.
    print("Sampling the nodes...")
    tic = time.time()
    nodes = createNodesUniform(N)
    toc = time.time()
    print("Sampled the nodes in %fsec." % (toc-tic))

    # Show the sample nodes.
    if True:
        for node in nodes:
            visual.drawNode(node, color='k', linewidth=1)
        visual.show("Showing the nodes (last link only)")

    # Add the start/goal nodes.
    nodes.append(startnode)
    nodes.append(goalnode)


    # Connect to the nearest neighbors.
    print("Connecting the nodes...")
    tic = time.time()
    connectKNeighbors(nodes, K)
    toc = time.time()
    print("Connected the nodes in %fsec." % (toc-tic))

    # Show the neighbor connections.
    if True:
        for (i,node) in enumerate(nodes):
            for neighbor in node.neighbors:
                if neighbor not in nodes[:i]:
                    visual.drawEdge(node, neighbor, color='g', linewidth=1)
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
        for node in nodes:
            if node.done:
                visual.drawNode(node, color='r')
        visual.show("Showing DONE nodes")
        return

    # Show the path.
    cost = pathCost(path)
    visual.drawPath(path, color='r', linewidth=2)
    visual.show("Showing the raw path (cost/length %.1f)" % cost)


    # Post process the path.
    finalpath = postProcess(path)


    # Unwrap: If allowing multiple revolutions, this shows the nodes
    # past +/-180deg (to display the wrapping).
    for i in range(1,len(finalpath)):
        finalpath[i] = finalpath[i-1].intermediate(finalpath[i], 1.0)

    # Report the path.
    for node in finalpath:
        print(node)


    # Show the post-processed path.
    cost = pathCost(finalpath)
    visual.drawPath(finalpath, color='b', linewidth=2)
    visual.show("Showing the post-processed path (cost/length %.1f)" % cost)


if __name__== "__main__":
    run()
