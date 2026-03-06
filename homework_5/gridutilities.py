'''gridutilities.py

   This provides two helper classes:
     Visualization - to visualize a grid of probablilities
     Robot         - to simulate a grid-based robot

'''

import matplotlib.pyplot as plt
import numpy as np
import random


######################################################################
#
#   VISUALIZATION of the probability grid
#
#     visual = Visualization(walls, robot)
#
#     visual.show(prob)
#     visual.show(prob, msg="Message")
#     visual.show(prob, msg, markRobot=True)
#
#     walls         NumPy 2D array defining both the grid size
#                   (rows/cols) and walls (being non-zero elements).
#     robot         Robot object (see below)
#     prob          NumPy 2D array of probabilities, values 0 to 1.
#
#   visual.Show() will visualize the probabilities.  An optional
#   second argument provides a message and waits for user input.
#   An optional third argument implies the robot's actual position
#   should be overlayed with an 'x'.
#
class Visualization():
    def __init__(self, walls, robot):
        # Save the walls, robot, and determine the rows/cols:
        self.walls = walls
        self.robot = robot
        self.spots = np.sum(np.logical_not(walls))
        self.rows  = np.size(walls, axis=0)
        self.cols  = np.size(walls, axis=1)

        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.grid(False)
        plt.gca().axis('off')
        plt.gca().set_aspect('equal')
        plt.gca().set_xlim(0, self.cols)
        plt.gca().set_ylim(self.rows, 0)

        # Add the row/col numbers.
        for row in range(0, self.rows, 2):
            plt.gca().text(         -0.3, 0.6+row, '%d'%row,
                           verticalalignment='center',
                           horizontalalignment='right')
        for row in range(1, self.rows, 2):
            plt.gca().text(self.cols+0.3, 0.6+row, '%d'%row,
                           verticalalignment='center',
                           horizontalalignment='left')
        for col in range(0, self.cols, 2):
            plt.gca().text(0.5+col,          -0.3, '%d'%col,
                           verticalalignment='bottom',
                           horizontalalignment='center')
        for col in range(1, self.cols, 2):
            plt.gca().text(0.5+col, self.rows+0.3, '%d'%col,
                           verticalalignment='top',
                           horizontalalignment='center')

        # Draw the grid, zorder 1 means draw after zorder 0 elements.
        for row in range(self.rows+1):
            plt.gca().axhline(row, lw=1, color='k', zorder=1)
        for col in range(self.cols+1):
            plt.gca().axvline(col, lw=1, color='k', zorder=1)

        # Add the text.
        plt.gca().text(0, 29, "Probability: Yellow==0%")
        plt.gca().text(0, 31, "     White<=0.1%, Pink<1%, Violet<10%, Purple<50%, Blue=100%")

        # Clear the content and mark.  Then show blank field.
        self.content = None
        self.mark    = None
        self.show()

    def flush(self):
        # Show the plot.
        plt.pause(0.001)

    def updatemark(self, markRobot=True):
        # Clear/potentially remove the previous mark.
        if self.mark is not None:
            self.mark.remove()
            self.mark = None

        # If requested, add a new mark.
        if markRobot:
            # Grab the robot position and check.
            row = self.robot.row
            col = self.robot.col
            assert (row >= 0) and (row < self.rows), "Illegal robot row"
            assert (col >= 0) and (col < self.cols), "Illegal robot col"

            # Draw the mark.
            self.mark  = plt.gca().text(0.5+col, 0.5+row, 'x', color = 'green',
                                        verticalalignment='center',
                                        horizontalalignment='center',
                                        zorder=1)

    def updategrid(self, prob):
        # Check the probability grid array size.
        if prob is not None:
            assert np.size(prob, axis=0) == self.rows, "Inconsistent # of rows"
            assert np.size(prob, axis=1) == self.cols, "Inconsistent # of cols"

        # Potentially remove the previous grid/content.
        if self.content is not None:
            self.content.remove()
            self.content = None

        # Create the color range.  There are clearly more elegant ways...
        color = np.ones((self.rows, self.cols, 3))
        for row in range(self.rows):
            for col in range(self.cols):
                if self.walls[row,col]:
                    color[row,col,0:3] = np.array([0.0, 0.0, 0.0])   # Black
                elif prob is None:
                    color[row,col,0:3] = np.array([1.0, 1.0, 1.0])   # White
                else:
                    # Shades of pink/purple/blue. Yellow means impossible.
                    p    = prob[row,col]
                    pmin = 0.9 / self.spots
                    if p == 0:
                        color[row,col,0:3] = np.array([1.0, 1.0, 0.0])
                    elif p < pmin:
                        rlevel = (1.0 - p)
                        glevel = (1.0 - p)
                        color[row,col,0:3] = np.array([rlevel, glevel, 1.0])
                    else:
                        rlevel = (1.0 - p)
                        glevel = (1.0 - p) * pmin/p
                        color[row,col,0:3] = np.array([rlevel, glevel, 1.0])
    
        # Draw the boxes.
        self.content = plt.gca().imshow(color,
                                        aspect='equal',
                                        interpolation='none',
                                        extent=[0, self.cols, self.rows, 0],
                                        zorder=0)

    def show(self, prob = None, msg = None, markRobot = False):
        # Update the content.
        self.updategrid(prob)

        # Potentially add the mark.
        self.updatemark(markRobot)

        # Flush the figure.
        self.flush()

        # Optionally display a message and wait for confirmation.
        if msg:
            input(msg)


######################################################################
#
#   ROBOT SIMULATION:
#
#     robot = Robot(walls, row=0, col=0,
#                   pSensor=[1.0], pCommand=1.0, kidnap=False)
#
#     robot.command(drow, dcol)
#     True/False = robot.sensor(drow, dcol)
#
#     pSensor       List of probabilities (0 to 1).  Each element is
#                   the probability that the proximity sensor will
#                   fire at a distance of (index+1 = 1,2,3,etc).
#                   The probability will be zero at greater distances.
#     pCommand      Probability the command is executed (0 to 1).
#     kidnap        Flag - kidnap the robot (once in the simulation)
#
#     (drow, dcol)  Delta up/right/down/left: (-1,0) (0,1) (1,0) (0,-1)
#
#   Simulate a robot, to give us the sensor readings.  If the starting
#   row/col are not given, pick them randomly.  Note both the command
#   and the sensor may be configured to a random probability level.
#
class Robot():
    def __init__(self, walls, row = 0, col = 0,
                 pSensor = [1.0], pCommand = 1.0, kidnap = False):
        # Check the row/col arguments.
        assert (row >= 0) and (row < np.size(walls, axis=0)), "Illegal row"
        assert (col >= 0) and (col < np.size(walls, axis=1)), "Illegal col"

        # Report.
        if walls[row, col]:
            location = " (random location)"
        else:
            location = " (at %d, %d)" % (row, col)
        print("Starting robot with real" +
              " sensor probabilities = " + str(pSensor) +
              ", command probability = " + str(pCommand) +
              ", kidnap = " + str(kidnap) + str(location))

        # Save the walls, the initial location, and the probabilities.
        self.walls    = walls
        self.pCommand = pCommand
        self.pSensor  = pSensor
        if kidnap:
            self.countdown = random.randrange(10, 15)
        else:
            self.countdown = 0

        # Set the initial location (the default is invalid, so randomize).
        self.jumpTo(row, col)

    def jumpTo(self, row, col):
        # If invalid, randomize until we have a valid location.
        while self.walls[row, col]:
            row = random.randrange(0, np.size(self.walls, axis=0))
            col = random.randrange(0, np.size(self.walls, axis=1))
        # Set the location.
        self.row = row
        self.col = col

    def command(self, drow, dcol):
        # Check the delta.
        assert ((abs(drow+dcol) == 1) and (abs(drow-dcol) == 1)), "Bad delta"

        # Check whether to kidnap.
        if self.countdown > 0:
            self.countdown -= 1
            if self.countdown == 0:
                self.jumpTo(0,0)
                return

        # Try to move the robot the given delta.
        row = self.row + drow
        col = self.col + dcol
        if (not self.walls[row, col]) and (random.random() < self.pCommand):
            self.row = row
            self.col = col

    def sensor(self, drow, dcol): 
        # Check the delta.
        assert ((abs(drow+dcol) == 1) and (abs(drow-dcol) == 1)), "Bad delta"

        # Check the proximity in the given direction.
        for k in range(len(self.pSensor)):
            if self.walls[self.row + drow*(k+1), self.col + dcol*(k+1)]:
                return (random.random() < self.pSensor[k])
        return False
