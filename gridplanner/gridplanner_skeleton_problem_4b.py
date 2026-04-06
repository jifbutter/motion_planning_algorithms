'''gridplanner_skeleton.py

   This is the skeleton code for the grid planner, which will develop
   both Dijstra's and the Astar algorithms.

   PLEASE FINISH WRITING THE CODE IN THE planner() function, also
   marked by FIXME.

   When working, this will find a path in a simple 2D grid.

'''

import bisect

from math       import inf
from visualgrid import VisualGrid


#
#  Define the Grid
#
grid1 = ['####################',
         '#                  #',
         '#                  #', 
         '#   ########       #',
         '#          ##      #',
         '#   S       ##G    #',
         '#            ####  #',
         '#                  #',
         '#                  #',
         '####################']

grid2 = ['#####################',
         '#    #    #    #    #',
         '#    #    #    #G   #', 
         '#    #    #    #    #',
         '##  ## ##### ###    #',
         '#  S           #    #',
         '#                   #',
         '###### ### #####    #',
         '#       #      #    #',
         '#       #      #    #',
         '#####################']

grid = grid2


#
#   Colors
#
WHITE  = [1.000, 1.000, 1.000]
BLACK  = [0.000, 0.000, 0.000]
RED    = [1.000, 0.000, 0.000]
BARK   = [0.325, 0.192, 0.094]  # bark brown
GREEN  = [0.133, 0.545, 0.133]  # forrest green
SKY    = [0.816, 0.925, 0.992]  # light blue


#
#   Node Class
#
#   We create one node to match each valid state being the robot
#   occupying a valid space in the grid.  That is, one node per
#   unblocked grid element (with a row/column).
#
#   To encode the graph, we also note a list of accessible neighbors.
#   And, as part of the search, we store the parent (in the tree), the
#   cost to reach this node (via the tree), and the status flags.
#
class Node:
    # Initialization
    def __init__(self, row, col):
        # Save the matching state.
        self.row = row
        self.col = col

        # Clear the list of neighbors (used for the full graph).
        self.neighbors = []

        # Clear the parent (used for the search tree), as well as the
        # actual cost to reach (via the parent).
        self.parent = None      # No parent
        self.cost   = inf       # Unable to reach = infinite cost

        # State of the node during the search algorithm.
        self.seen = False
        self.done = False

        # predicted_path_cost = real distance from start to node + estimated distance from node to goal
        self.predicted_path_cost = inf


    # Define the Manhattan distance to another node.
    def distance(self, other):
        return abs(self.row - other.row) + abs(self.col - other.col)
    
    def direction_cost(self, other, direction):
        if direction == "vertical":
            return 9 * (1 + abs(self.col - 9)) * abs(self.row - other.row)
        elif direction == "horizontal":
            return 5 * (1 +abs(self.row - 5)) * abs(self.col - other.col)
        else:
            return None

    # Define the "less-than" to enable sorting by cost.
    def __lt__(self, other):
        # Queue based on total predicted path cost
        return self.predicted_path_cost < other.predicted_path_cost

    # Print (for debugging).
    def __str__(self):
        return("(%2d,%2d)" % (self.row, self.col))
    def __repr__(self):
        return("<Node %s, %7s, cost %f>" %
               (str(self),
                "done" if self.done else "seen" if self.seen else "unknown",
                self.cost))


#
#   Search/Planner Algorithm
#
#   This is the core algorithm.  It builds a search tree inside the
#   node graph, transfering nodes from air (not seen) to leaf (seen,
#   but not done) to trunk (done).
#
# Run the planner.
def planner(start, goal, show = None):
    # Use the start node to initialize the on-deck queue: it has no
    # parent (being the start), zero cost to reach, and has been seen.
    start.seen   = True
    start.cost   = 0
    start.parent = None
    x = 10
    start.predicted_path_cost = start.cost + x * start.distance(goal)
    onDeck = [start]
    path = []


    # Continually expand/build the search tree.
    print("Starting the processing...")
    
    while True:
        # Show the grid.
        if show:
            show()

        # Make sure we have something pending in the on-deck queue.
        # Otherwise we were unable to find a path!
        if not (len(onDeck) > 0):
            return None

        # Grab the next state (first on the storted on-deck list).
        node = onDeck.pop(0)
        node.done = True

        if node is goal:
            break

        for neighbor in node.neighbors:
            # Don't process neighbor if it's already been processed
            if neighbor.done:
                continue
            
            cost_to_go = x * neighbor.distance(goal)
            
            # Check if neighbor has been processed or if there's a shorter route to it
            new_cost = node.cost + 1
            if not (neighbor.seen) or (new_cost < neighbor.cost):
                # If neighbor is in onDeck and is being replaced with a lower cost, remove the previous one
                try:
                    onDeck.remove(neighbor)
                except ValueError:
                    pass

                neighbor.cost = new_cost
                neighbor.predicted_path_cost = new_cost + cost_to_go
                neighbor.seen = True
                neighbor.parent = node
                bisect.insort(onDeck, neighbor)
            else:
                pass

    # Work way backwards and go from goal to start using parent nodes, then reverse
    node = goal
    while node is not None:
        path.append(node)
        node = node.parent

    path.reverse()
                
    return path

######################################################################
#
#  Main Code
#
if __name__== "__main__":

    ####################  INITIALIZE  ####################
    # Grab the dimensions.
    rows = len(grid)
    cols = max([len(line) for line in grid])

    # Set up the visual grid.
    visual = VisualGrid(rows, cols)

    # Parse the grid to set up the nodes list, as well as start/goal.
    nodes  = []
    for row in range(rows):
        for col in range(cols):
            # Create a node per space, except only color walls black.
            if grid[row][col] == '#':
                visual.color(row, col, BLACK)
            else:
                nodes.append(Node(row, col))

    # Create the neighbors, being the edges between the nodes.
    for node in nodes:
        for (dr, dc) in [(-1,0), (1,0), (0,-1), (0,1)]:
            others = [n for n in nodes
                      if (n.row,n.col) == (node.row+dr,node.col+dc)]
            if len(others) > 0:
                node.neighbors.append(others[0])

    # Grab/mark the start/goal.
    start = [n for n in nodes if grid[n.row][n.col] in 'Ss'][0]
    goal  = [n for n in nodes if grid[n.row][n.col] in 'Gg'][0]
    visual.write(start.row, start.col, 'S')
    visual.write(goal.row,  goal.col,  'G')
    visual.show(wait="Hit return to start")


    ####################  RUN  ####################
    # Create a function to show each step.
    def show(wait=0.005):
        # Update the grid for all nodes.
        for node in nodes:
            # Choose the appropriate color.
            if   node.done: visual.color(node.row, node.col, BARK)
            elif node.seen: visual.color(node.row, node.col, GREEN)
            else:           visual.color(node.row, node.col, SKY)
        # Show.
        visual.show(wait)

    # Run.
    path = planner(goal, start, show)


    ####################  REPORT  ####################
    # Check the number of nodes.
    unknown   = len([n for n in nodes if not n.seen])
    processed = len([n for n in nodes if n.done])
    ondeck    = len(nodes) - unknown - processed
    print("Solution cost %f" % start.cost)
    print("%3d states fully processed" % processed)
    print("%3d states still pending"   % ondeck)
    print("%3d states never reached"   % unknown)

    # Show the path in red.
    if not path:
        print("UNABLE TO FIND A PATH")
    else:
        print("Marking the path")
        for node in path:
            visual.color(node.row, node.col, RED)
        visual.show()

    input("Hit return to end")