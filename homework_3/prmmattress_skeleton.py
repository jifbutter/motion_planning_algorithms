'''prmmattress.py

   This is the PRM skeleton code for the mattress moving problem.

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
N = 250  # FIXME: Select the number of nodes
K = 10  # FIXME: Select the number of nearest neighbors

# Define the mattress dimensions.
L = 7           # Legth of the mattress
W = 1           # Width of the mattress

# Step size in states for connectivity testing and plotting.  Make
# sure there is no space between the physical bounding boxes in the
# test paths.
STEPSIZE = 0.2

# FIXME: Sampling Style, change in part (c)
SAMPLEATEDGES = True

# FIXME: Include the bonus wall for part (d)
BONUSWALL = False

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
(xmin, xmax) = (0, 30)
(ymin, ymax) = (0, 20)

(xA, xB, xC, xD, xE) = ( 5, 12, 15, 18, 21)
(yA, yB, yC, yD)     = ( 5, 10, 12, 15)

xlabels = (xmin, xA, xB, xC, xD, xE, xmax)
ylabels = (ymin, yA, yB, yC, yD,     ymax)

outside = LineString([[xmin, ymin], [xmax, ymin], [xmax, ymax],
                      [xmin, ymax], [xmin, ymin]])
wall1   = LineString([[xmin, yB], [xC, yB]])
wall2   = LineString([[xD, yB], [xmax, yB]])
wall3   = LineString([[xB, yC], [xC, yC], [xC, ymax]])
bonus   = LineString([[xD, yC], [xE, yC]])

# Collect all the wall and prepare (for faster checking).
if BONUSWALL:
    walls = prep(MultiLineString([outside, wall1, wall2, wall3, bonus]))
else:
    walls = prep(MultiLineString([outside, wall1, wall2, wall3]))

# Define the start/goal states (x, y, theta)
(xstart, ystart, tstart) = (xA, yD, pi/2)
(xgoal,  ygoal,  tgoal)  = (xA, yA, 0)


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
        plt.gca().set_xticks(xlabels)
        plt.gca().set_yticks(ylabels)
        plt.gca().set_aspect('equal')

        # Show the walls.
        for l in walls.context.geoms:
            plt.plot(*l.xy, 'k', linewidth=2)
        if bonus in walls.context.geoms:
            plt.plot(*bonus.xy, 'b:', linewidth=3)

        # Show immediately.
        self.show()

    def show(self, text = ''):
        # Show the plot.
        plt.pause(0.001)
        # If text is specified, print and wait for confirmation.
        if len(text)>0:
            input(text + ' (hit return to continue)')

    def drawNode(self, node, **kwargs):
        plt.plot(*node.outline.exterior.xy, **kwargs)

    def drawEdge(self, head, tail, **kwargs):
        plt.plot([head.x, tail.x], [head.y, tail.y], **kwargs)

    def drawPath(self, path, **kwargs):
        for i in range(len(path)-1):
            n = ceil(path[i].distance(path[i+1]) / STEPSIZE)
            for j in range(n):
                node = path[i].intermediate(path[i+1], j/n)
                self.drawNode(node, **kwargs)
        self.drawNode(path[-1], **kwargs)


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
    def __init__(self, x, y, t):
        # Define/remember the state (x,y,theta).
        self.x = x
        self.y = y
        self.t = t              # Angle theta

        # Precompute the mattress box (for collision detection and drawing).
        self.outline = Polygon(
            [[x + L/2*cos(t) + W/2*sin(t), y + L/2*sin(t) - W/2*cos(t)],
             [x + L/2*cos(t) - W/2*sin(t), y + L/2*sin(t) + W/2*cos(t)],
             [x - L/2*cos(t) - W/2*sin(t), y - L/2*sin(t) + W/2*cos(t)],
             [x - L/2*cos(t) + W/2*sin(t), y - L/2*sin(t) - W/2*cos(t)]])

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
        # FIXME_FIRST: Compute and return the distance (used to sort).

        # Set the radius of mattress which is L / 2
        R = L / 2

        # Find change in angle when 0 = 180 = 360 degrees
        angle_distance = abs(sin(wrap180(self.t - other.t)))

        # Use formula to find distance between two nodes accounting for change in x, y, and theta
        distance = sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (R * angle_distance)**2)

        return distance

    # Check whether in free space.
    def inFreespace(self):
        return walls.disjoint(self.outline)

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
        return ("<Node %5.2f,%5.2f @ %5.2f>" % (self.x, self.y, self.t))

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    def intermediate(self, other, alpha):
        # Return the current Node

        new_x = self.x + alpha * (other.x - self.x)
        new_y = self.y + alpha * (other.y - self.y)
        new_t = wrap180(self.t + alpha * (wrap180(other.t - self.t)))

        return Node(new_x, new_y, new_t)

######################################################################
#
#   PRM Functions (to be fixed - see FIXME)
#
# Create the list of nodes.
def createNodesUniform(N):
    # FIXME: create, see also last week's code

    # Add nodes sampled uniformly across the space.
    nodes = []
    while len(nodes) < N:
        node = Node(random.uniform(xmin, xmax),
                    random.uniform(ymin, ymax),
                    random.uniform(-pi/2, pi/2))
        if node.inFreespace():
            nodes.append(node)
    return nodes


def createNodesAtEdges(N):
    
    # FIXME: please add in part (c)

    r = W / 2
    R = L / 2
    nodes = []

    while len(nodes) < N:
        # Reset r before generating new node
        r = W / 2
        
        # Generate a random Node
        node = Node(random.uniform(xmin, xmax),
                    random.uniform(ymin, ymax),
                    random.uniform(-pi/2, pi/2))
        
        # Continue generating new node until it collides with obstacle
        while node.inFreespace():
            node = Node(random.uniform(xmin, xmax),
                    random.uniform(ymin, ymax),
                    random.uniform(-pi/2, pi/2))
        
        # Continue loop until a node by edge is found
        while True:
            dx = random.uniform(-r, r)
            dy = random.uniform(-r, r)
            dt = random.uniform(-r/R, r/R)
            test_node = Node(node.x + dx, node.y + dy, wrap180(node.t + dt))
            
            # If non-colliding node (node by edge) is found, add to list
            # else, increase radius by 10%
            if test_node.inFreespace():
                nodes.append(test_node)
                break
            else:
                r = 1.1 * r

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
#  Run/Test/Main Code (No Fixes Needed)
#
def run():
    # Report the parameters.
    print('Running with', N, 'nodes and', K, 'neighbors.')

    # Create the figure.  Some computers seem to need an additional show()?
    visual = Visualization()
    visual.show()

    # Create the start/goal nodes.
    startnode = Node(xstart, ystart, tstart)
    goalnode  = Node(xgoal,  ygoal,  tgoal)

    # Show the start/goal nodes.
    visual.drawNode(startnode, color='orange', linewidth=3)
    visual.drawNode(goalnode,  color='purple', linewidth=3)
    visual.show("Showing basic world")


    # Create the list of nodes.
    print("Sampling the nodes...")
    tic = time.time()
    if   SAMPLEATEDGES:  nodes = createNodesAtEdges(N)
    else:                nodes = createNodesUniform(N)
    toc = time.time()
    print("Sampled the nodes in %fsec." % (toc-tic))

    # Show the sample nodes.
    # if True:
    #     for node in nodes:
    #         visual.drawNode(node, color='grey', linewidth=1)
    #     visual.show("Showing the nodes")

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
    # if True:
    #     for (i,node) in enumerate(nodes):
    #         for neighbor in node.neighbors:
    #             if neighbor not in nodes[:i]:
    #                 visual.drawEdge(node, neighbor, color='g', linewidth=1)
    #     visual.show("Showing the full graph")


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
                visual.drawNode(node, color='r', linewidth=1)
        visual.show("Showing DONE nodes")
        return

    # Show the path.
    cost = pathCost(path)
    visual.drawPath(path, color='r', linewidth=1)
    visual.show("Showing the raw path (cost/length %.1f)" % cost)


    # Post process the path.
    finalpath = postProcess(path)

    # Show the post-processed path.
    cost = pathCost(finalpath)
    visual.drawPath(finalpath, color='b', linewidth=1)
    visual.show("Showing the post-processed path (cost/length %.1f)" % cost)


def test():
    # Define the test paths.
    path1 = [Node( 5, 2, 0), Node( 5, 8,      pi)]
    path2 = [Node(15, 5, 0), Node(15, 5, 1.49*pi)]
    path3 = [Node(26, 2, 0), Node(23, 5, 0.5 *pi)]

    # Create the figure.  Some computers seem to need an additional show()?
    visual = Visualization()
    visual.show()

    # Show path with end points highlighted.
    def show(path, **kwargs):
        visual.drawNode(path[0 ], linewidth=2, **kwargs)
        visual.drawNode(path[-1], linewidth=2, **kwargs)
        visual.drawPath(path,     linewidth=1, **kwargs)
    
    show(path1, color='b')
    show(path2, color='g')
    show(path3, color='r')
    visual.show("Showing the test paths")


if __name__== "__main__":
    test()
    run()
