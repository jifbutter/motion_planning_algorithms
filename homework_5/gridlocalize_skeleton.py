'''gridlocalize.py

   This is the skeleton code for the grid-based localization.

   PLEASE FINISH WRITING THE CODE, especially where marked by FIXME.

'''

import numpy as np

from gridutilities import Visualization, Robot


#
#  Define the Walls
#
w = ['xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx',
     'x               xx             xx               x',
     'x                xx           xx                x',
     'x                 xx         xx                 x',
     'x        xxxx      xx       xx                  x',
     'x        x  xx      xx     xx                   x',
     'x        x   xx      xx   xx      xxxxx         x',
     'x        x    xx      xx xx     xxx   xxx       x',
     'x        x     xx      xxx     xx       xx      x',
     'x        x      xx      x      x         x      x',
     'x        x       xx           xx         xx     x',
     'x        x        x           x           x     x',
     'x        x        x           x           x     x',
     'x        x        x           x           x     x',
     'x                 xx         xx           x     x',
     'x                  x         x                  x',
     'x                  xx       xx                  x',
     'x                   xxx   xxx                   x',
     'x                     xxxxx         x           x',
     'x                                   x          xx',
     'x                                   x         xxx',
     'x            x                      x        xxxx',
     'x           xxx                     x       xxxxx',
     'x          xxxxx                    x      xxxxxx',
     'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx']

walls = np.array([[1.0*(c == 'x') for c in s] for s in w])
rows  = np.size(walls, axis=0)
cols  = np.size(walls, axis=1)


######################################################################
#
#  PREDICTION
#
#  Input:  bel          Grid of probabilities (current belief)
#          drow, dcol   Commanded movement delta in row/col
#
#  Output: prd          Grid of probabilities (prediction)
#
def computePrediction(bel, drow, dcol):
    # Prepare an empty prediction grid.
    prd = np.zeros((rows,cols))
    free_cells = 0

    # Determine the new probablities (remember to consider walls).
    # FIXME: Build up the grid of probabilities...
    
    # Loop through all of the rows and columns
    for row in range(0, rows):
        for col in range(0, cols):
            # Calculate the new position
            new_row = row + drow
            new_col = col + dcol
            
            # Check if new row and new col are within bounds and isn't a wall
            if (rows > new_row >= 0 and cols > new_col >= 0) and ((walls[new_row, new_col]) == 0):
                prd[new_row, new_col] += 0.8 * bel[row, col]
                prd[row, col] += 0.2 * bel[row, col]
            else:
                # Assign and add probabiltiy
                prd[row, col] += bel[row, col]

    # Loop through all cells to find the free cells
    for row in range(0, rows):
        for col in range(0, cols):
            if walls[row, col] == 0:
                # Keep count of number of cells that aren't walls
                free_cells += 1

    uniform = 0.01 / free_cells

    # Loop through cells, add uniform to free cells
    for row in range(0, rows):
        for col in range(0, cols):
            if walls[row, col] == 0:
                prd[row, col] = 0.99 * prd[row, col] + uniform
            else:
                prd[row, col] = 0.99 * prd[row, col]

    # Check the prediction.
    if abs(np.sum(prd) - 1.0) > 1e-12:
        print("WARNING: Prediction does not add up to 100%")

    # Return the prediction grid.
    return prd

#
#  MEASUREMENT UPDATE (CORRECTION)
#
#  Input:  prior        Grid of prior probabilities (belief)
#          probSensor   Grid of modeled probabilities that (sensor==True)
#          sensor       Actual value of sensor (True/False)
#
#  Output: post         Grid of posterior probabilities (updated belief)
#
def updateBelief(prior, probSensor, sensor):
    post = np.zeros((rows,cols))
    total = 0
    
    # Create the posterior belief.
    for row in range(0, rows):
        for col in range(0, cols):
            if sensor:
                post[row, col] = prior[row, col] * probSensor[row, col]
            else:
                post[row, col] = prior[row, col] * (1 - probSensor[row, col])
            total += post[row, col]

    # Re-normalize so total sum of probabilities adds up to 1
    for row in range(0, rows):
        for col in range(0, cols):
            post[row, col] = post[row, col] / total

    # Check the updated belief.
    if abs(np.sum(post) - 1.0) > 1e-12:
        print("WARNING: Belief does not add up to 100%")

    # Return the updated belief.
    return post


#######################################################################
#
#  Pre-compute the Modeled Sensor Probability Grid
#
#  Input:  drow, dcol   Sensor direction in row/col
#
#  Output: prob         Grid of modeled probabilities that (sensor==True)
#
def precomputeSensorProbability(drow, dcol):
    # Prepare an empty probability grid.
    prob = np.zeros((rows, cols))
    probSensor = [0.9, 0.6, 0.3]

    # Pre-compute the sensor probability on the grid.
    for row in range(0, rows):
        for col in range(0, cols):
            p = 0.0

            for d in range(1, 4):
                new_row = row + d * drow
                new_col = col + d * dcol
            
                # If new cell is out of bounds or there's a wall at that distance
                if (not (rows > new_row >= 0 and cols > new_col >= 0)) or (walls[new_row, new_col] == 1):
                    p = probSensor[d - 1]
                    break

            prob[row, col] = p

    # Return the computed grid.
    return prob


######################################################################
#
#  Main Code
#
def main():
    # Initialize the robot simulation.
    # FIXME... PICK THE "REALITY" WE SHOULD SIMULATE:
    
    # robot=Robot(walls)
    # 4a 
    # robot=Robot(walls)
    # 4b 
    # robot=Robot(walls, row=12, col=26)
    # 5  
    # robot=Robot(walls, row=12, col=26, pSensor=[0.9,0.6,0.3])
    # 6  
    # robot=Robot(walls, row=15, col=47, pSensor=[0.9,0.6,0.3], pCommand=0.8)
    # 7  
    robot=Robot(walls, row= 7, col=12, pSensor=[0.9,0.6,0.3], pCommand=0.8, kidnap=True)
    # Or to play:
    #    robot=Robot(walls, pSensor=[0.9,0.6,0.3], pCommand=0.8)


    # Initialize the figure.
    visual = Visualization(walls, robot)


    # Pre-compute the probability grids for each sensor reading.
    probUp    = precomputeSensorProbability(-1,  0)
    probRight = precomputeSensorProbability( 0,  1)
    probDown  = precomputeSensorProbability( 1,  0)
    probLeft  = precomputeSensorProbability( 0, -1)

    # Show the sensor probability maps.
    visual.show(probUp,    "Probability of up    proximal sensor reading True")
    visual.show(probRight, "Probability of right proximal sensor reading True")
    visual.show(probDown,  "Probability of down  proximal sensor reading True")
    visual.show(probLeft,  "Probability of left  proximal sensor reading True")


    # Start with a uniform belief grid.
    bel = (1.0 - walls) / np.sum(1.0 - walls)

    # Loop continually.
    while True:
        # Show the current belief.  Also show the actual position.
        visual.show(bel, markRobot=True)

        # Get the command key to determine the direction.
        while True:
            key = input("Cmd (q=quit, i=up, m=down, j=left, k=right) ?")
            if   (key == 'q'):  return
            elif (key == 'i'):  (drow, dcol) = (-1,  0) ; break # up
            elif (key == 'k'):  (drow, dcol) = ( 0,  1) ; break # right
            elif (key == 'm'):  (drow, dcol) = ( 1,  0) ; break # down
            elif (key == 'j'):  (drow, dcol) = ( 0, -1) ; break # left

        # Move the robot in the simulation.
        robot.command(drow, dcol)


        # Compute a prediction.
        prd = computePrediction(bel, drow, dcol)
        #visual.show(prd, "Showing the prediction")

        # Correct the prediction/execute the measurement update.
        bel = prd
        bel = updateBelief(bel, probUp,    robot.sensor(-1,  0))
        bel = updateBelief(bel, probRight, robot.sensor( 0,  1))
        bel = updateBelief(bel, probDown,  robot.sensor( 1,  0))
        bel = updateBelief(bel, probLeft,  robot.sensor( 0, -1))


if __name__== "__main__":
    main()
