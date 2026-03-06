'''rrttriangles.py

   This is the RRT skeleton code for the 2D triangular problem.

   PLEASE FINISH WRITING THE CODE, especially where marked by FIXME.

   Use RRT to find a path around polygonal obstacles.

'''

import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math               import inf, pi, sin, cos, atan2, sqrt, ceil, dist

from shapely.geometry   import Point, LineString, Polygon, MultiPolygon
from shapely.prepared   import prep


######################################################################
#
#   Parameters
#
#   Define the step size.  Also set the maximum number of nodes.

DSTEP = 1.0

# Maximum number of steps (attempts) or nodes (successful steps).
SMAX = 50000
NMAX = 1500


######################################################################
#
#   World Definitions (No Fixes Needed)
#
#   List of obstacles/objects as well as the start/goal.
#
(xmin, xmax) = (0, 10)
(ymin, ymax) = (0, 12)

# Collect all the triangles and prepare (for faster checking).
obstacles = prep(MultiPolygon([
    Polygon([[7,  3], [3,  3], [3,  4], [7,  3]]),
    Polygon([[5,  5], [7,  7], [4,  6], [5,  5]]),
    Polygon([[9,  2], [8,  7], [6,  5], [9,  2]]),
    Polygon([[1, 10], [7, 10], [4,  8], [1, 10]])]))

# Define the start/goal states (x, y, theta)
(xstart, ystart) = (6, 1)
(xgoal,  ygoal)  = (5, 11)


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

    def drawEdge(self, head, tail, **kwargs):
        plt.plot([head.x, tail.x], [head.y, tail.y], **kwargs)

    def drawPath(self, path, **kwargs):
        for i in range(len(path)-1):
            self.drawEdge(path[i], path[i+1], **kwargs)


######################################################################
#
#   Node Definition (No Fixes Needed)
#
class Node:
    #################
    # Initialization:
    def __init__(self, x, y):
        # Define/remember the state/coordinates (x,y).
        self.x = x
        self.y = y

        # Define a parent (cleared for now).
        self.parent = None

    ################
    # Planner functions:
    # Compute the relative distance to another node.
    def distance(self, other):
        return sqrt((other.x - self.x)**2 + (other.y - self.y)**2)

    # Check whether in free space.
    def inFreespace(self):
        if (self.x <= xmin or self.x >= xmax or
            self.y <= ymin or self.y >= ymax):
            return False
        point = Point(self.x, self.y)
        return obstacles.disjoint(point)

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
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


######################################################################
#
#   RRT Functions (to be fixed - see FIXME)
#
def rrt(startnode, goalnode, visual=None):
    # Start the tree with the startnode (set no parent just in case).
    startnode.parent = None
    tree = [startnode]

    # Function to attach a new node to an existing node: attach the
    # parent, add to the tree, and show in the figure.
    def addtotree(oldnode, newnode):
        newnode.parent = oldnode
        tree.append(newnode)
        if visual:
            visual.drawEdge(oldnode, newnode, color='g', linewidth=1)
            visual.show()

    # Loop - keep growing the tree.
    steps = 0
    while True:

        # FIXME: Implement an RRT step!
        # Break the while loop if the tree reaches the goal.

        # Uniformly sample a target from entire space
        # Add bias towards the goal
        percent = 5
        num = random.randint(1, 100)
        if num <= percent:
            target = goalnode
        else:
            target = Node(random.uniform(xmin, xmax),
                        random.uniform(ymin, ymax))
        
        last_dist = inf

        # Loop through nodes in tree to find closest to target
        for node in tree:
            new_dist = node.distance(target)
            if new_dist < last_dist:
                last_dist = new_dist
                nearest = node

        if nearest.distance(target) == 0:
            continue
        
        # Identify the new node DSTEP away from nearest
        # If nearest node is less than DSTEP away from target, new node is target
        if DSTEP >= nearest.distance(target):
            new_node = target
        else:
            dx = target.x - nearest.x
            dy = target.y - nearest.y
            d = nearest.distance(target)
            new_node_x = nearest.x + DSTEP * (dx/d)
            new_node_y = nearest.y + DSTEP * (dy/d)
            new_node = Node(new_node_x, new_node_y)

        # Check if nearest node connects to new node OR target and add to tree
        if (new_node.inFreespace()) and (nearest.connectsTo(new_node)):
            addtotree(nearest, new_node)
            goal_dist = new_node.distance(goalnode)
            if goal_dist <= DSTEP:
                # If new node is less than DSTEP away from goal, check connection and add to tree
                if new_node.connectsTo(goalnode):
                    addtotree(new_node, goalnode)
                    break
        
        # Check whether we should abort - too many steps or nodes.
        steps += 1
        if (steps >= SMAX) or (len(tree) >= NMAX):
            print("Aborted after %d steps and the tree having %d nodes" %
                  (steps, len(tree)))
            return None

    # Build the path.
    path = [goalnode]
    while path[0].parent is not None:
        path.insert(0, path[0].parent)

    # Report and return.
    print("Finished after %d steps and the tree having %d nodes" %
          (steps, len(tree)))
    return path


# Compute the path cost
def pathCost(path):
    cost = 0
    for i in range(1, len(path)):
        cost += path[i-1].distance(path[i])
    return cost

# Post process the path
def postProcess(path):
    shortpath = [path[0]]
    for i in range(2, len(path)):
        if not shortpath[-1].connectsTo(path[i]):
            shortpath.append(path[i-1])
    shortpath.append(path[-1])
    return shortpath


######################################################################
#
#  Main Code (No Fixes Needed)
#
def main():
    # Report the parameters.
    print('Running with step size ', DSTEP, ' and up to ', NMAX, ' nodes.')

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


    # Run the RRT planner.
    print("Running RRT...")
    path = rrt(startnode, goalnode, visual)

    # If unable to connect, just note before closing.
    if not path:
        visual.show("UNABLE TO FIND A PATH")
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


if __name__== "__main__":
    main()
