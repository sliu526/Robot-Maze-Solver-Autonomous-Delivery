from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

from collections import deque

"""
createMazeDict()
Description: Construct a dictionary representing a maze grid. The function takes three parameters:
nXCells and nYCells, which are integers representing the number of cells in the x-axis and y-axis of the
maze, respectively, and cellDim, an integer specifying the dimension of each cell in the maze in
centimeters. The function iterates over the grid defined by nXCells and nYCells, creating a dictionary
entry for each cell. Each entry has a tuple of (i, j) as the key, where i and j are the cell's x and y coordinates
in the grid. The value associated with each key is another dictionary containing the cell's actual position
in space (position), a list of neighboring cells (neighbors), and another boolean flag to track if the cell has
been visited (visited), a cost that can be used to indicate the distance to the destination (cost). The
function returns the mazeDict, which is a comprehensive map of the maze.
"""

def createMazeDict(nXCells, nYCells, cellDim):
    mazeDict = {}
    for i in range(nXCells):
        for j in range(nYCells):
            cellsKey = (i, j)
            position = (i*cellDim, j*cellDim)
            neighbors = []
            visited = position in neighbors
            cost = 0
            mazeDict[cellsKey] = {'position': position, 'neighbors': neighbors, 'visited': visited, 'cost': cost}
    return mazeDict

"""
addAllNeighbors()
Description: This function is designed to populate the 'neighbors' field for each cell in a maze grid within
the provided mazeDict. The function iterates over all possible cell coordinates in a grid defined by nXCells
(number of cells along the x-axis) and nYCells (number of cells along the y-axis). For each cell, it
calculates and adds a list of neighboring cells (coordinates of adjacent cells) to the mazeDict. This
process involves checking all potential adjacent cells (left, front, right, and back) and adding them to the
neighbors list if they fall within the grid's boundaries. The function updates the mazeDict with this neighbor
information for each cell and returns the updated dictionary.
"""

def addAllNeighbors(mazeDict, nXCells, nYCells):
    for i in range(nXCells):
        for j in range(nYCells):
            cells = (i, j)
            neighborList = []

            if i - 1 >= 0:
                newi = i - 1
                neighborList.append((newi, j))

            if j + 1 <= len(range(nYCells))-1:
                newi = j + 1
                neighborList.append((i, newi))

            if i + 1 <= len(range(nXCells))-1:
                newi = i + 1
                neighborList.append((newi, j))

            if j - 1 >= 0:
                newi = j - 1
                neighborList.append((i, newi))
                
            mazeDict[cells]['neighbors'] += neighborList
    return mazeDict

"""
getRobotOrientation()
Description: This function converts a robot's heading, given in degrees, into the nearest cardinal
direction ("N", "W", "S", or "E"). The heading parameter is a floating-point number representing the robot's
current orientation in degrees, where 0 or 360 degrees correspond to East ("E"), 90 to North ("N"), 180
to West ("W"), and 270 to South ("S"). The function calculates the closest cardinal direction to this heading
and returns it as a string.
"""

def getRobotOrientation(heading):
    E = (0, 360)
    N = 90
    W = 180
    S = 270
    if (abs(heading - E[1]) < abs(N- heading) and abs(heading - E[1])<abs(heading - W) and abs(heading - E[1])<abs(S - heading)) or ((heading < abs(N- heading) and heading<abs(heading - W) and heading<abs(S - heading))):
        return "E"
    elif abs(N- heading) < abs(heading - W) and abs(N- heading) < abs(S - heading):
        return "N"
    elif abs(W - heading) < abs(S - heading):
        return "W"
    else:
        return "S"

"""
getPotentialNeighbors()
Description: This function calculates the indices of the potential neighboring cells relative to a given cell
in a grid, based on the robot's current orientation. The currentCell parameter is a tuple representing the
(i, j) coordinates of the robot's current position in the grid. The orientation parameter is a string that
indicates the robot's current facing direction, with possible values "N", "S", "E", or "W" (representing North,
South, East, or West, respectively). The function returns a list containing four tuples, each representing
the grid index of the cell directly to the left, in front, to the right and behind the currentCell, given the
specified orientation.
"""

def getPotentialNeighbors(currentCell, orientation):
    neighborList = []
    if orientation == "S":
        leftNeighbor = (currentCell[0]+1, currentCell[1])
        backNeighbor = (currentCell[0], currentCell[1]+1)
        frontNeighbor = (currentCell[0], currentCell[1]-1)
        rightNeighbor = (currentCell[0]-1, currentCell[1])

    if orientation == "N":
        leftNeighbor = (currentCell[0]-1, currentCell[1])
        backNeighbor = (currentCell[0], currentCell[1]-1)
        frontNeighbor = (currentCell[0], currentCell[1]+1)
        rightNeighbor = (currentCell[0]+1, currentCell[1])

    if orientation == "E":
        leftNeighbor = (currentCell[0], currentCell[1]+1)
        backNeighbor= (currentCell[0]-1, currentCell[1])
        frontNeighbor= (currentCell[0]+1, currentCell[1])
        rightNeighbor = (currentCell[0], currentCell[1]-1)

    if orientation == "W":
        leftNeighbor = (currentCell[0], currentCell[1]-1)
        backNeighbor= (currentCell[0]+1, currentCell[1])
        frontNeighbor= (currentCell[0]-1, currentCell[1])
        rightNeighbor = (currentCell[0], currentCell[1]+1)

    neighborList = [leftNeighbor, frontNeighbor, rightNeighbor, backNeighbor]
    return neighborList

"""
isValidCell()
Description: The isValidCell function determines whether a given cell, identified by its coordinates
(cellIndices), falls within the bounds of a specified grid. The grid is defined by its dimensions nXCells (the
number of cells along the x-axis) and nYCells (the number of cells along the y-axis). The cellIndices
parameter is a tuple consisting of the x and y coordinates of the cell in question (e.g., (x, y)). The function
checks if the x-coordinate is between 0 and nXCells (exclusive) and if the y-coordinate is between 0 and
nYCells (exclusive). It returns True if both conditions are met, indicating that the cell is within the grid's
boundaries; otherwise, it returns False.
"""

def isValidCell(cellIndices, nXCells, nYCells):
    (current_x, current_y) = cellIndices
    inX = False
    inY = False
    if current_x >= 0 and current_x < nXCells:
        inX = True
    if current_y >= 0 and current_y < nYCells:
        inY = True
    if inX and inY:
        return True
    return False

"""
getWallConfiguration()
Description: This function assesses the presence of walls around the robot using infrared (IR) sensor
readings. It specifically utilizes readings from three IR sensors – IR0, IR3, and IR6, which are expected
to represent the left, front, and right IR sensor readings, respectively. The threshold parameter is an
integer that acts as a benchmark for determining the presence of a wall. The function processes each IR
sensor reading using the proximity formula 4095/(IRx+1), where IRx is the reading from one of the IR
sensors and compares the result to the threshold. If the calculated value is less than or equal to the
threshold, it infers the presence of a wall in that direction. The function returns a tuple of booleans
indicating the presence (True) or absence (False) of walls on the left, in front, and on the right of the
robot, respectively.
"""

def getWallConfiguration(IR0,IR3,IR6,threshold):
    proxList = []
    wallList= []
    leftProx = 4095/(IR0 + 1)
    frontProx = 4095/(IR3 + 1)
    rightProx = 4095/(IR6 + 1)
    proxList.append(leftProx)
    proxList.append(frontProx)
    proxList.append(rightProx)
    for prox in proxList:
        if prox <= threshold:
            wallList.append(True)
        else:
            wallList.append(False)
    return wallList

"""
getNavigableNeighbors()
Description: This function determines the accessible neighboring cells of a given cell within a grid, taking
into account the presence of walls, the grid's dimensions, and the previously visited cell. The function
evaluates which of the potential neighboring cells, denoted by their coordinates in potentialNeighbors,
are accessible from the current cell. This evaluation is based on the presence of walls around the current
cell (wallsAroundCell), the grid's dimensions (nXCells and nYCells), and the previously visited cell
(prevCell). The function returns a list of tuples (navNeighbors), each representing the coordinates of a
navigable neighboring cell.
"""

def getNavigableNeighbors(wallsAroundCell, potentialNeighbors, prevCell, nXCells, nYCells):
    navNeighbors = []
    if prevCell != None:
        navNeighbors.append(prevCell)
    for i in range(len(wallsAroundCell)):
        if wallsAroundCell[i] == True:
            continue
        elif isValidCell(potentialNeighbors[i], nXCells, nYCells):
            navNeighbors.append(potentialNeighbors[i])
    return navNeighbors

"""
updateMazeNeighbors()
Description: This function updates the neighbor information concerning a specific cell within a maze. It
takes as input the mazeDict and the currentCell parameters. The function first iterates through the current
neighbors of currentCell in mazeDict and removes currentCell from their neighbors if they are not in the
navNeighbors list. Then, it sets the 'neighbors' field of currentCell in mazeDict to the new navNeighbors
list. This process ensures that the bidirectional relationship between cells and their neighbors is correctly
maintained. After updating, the function returns the modified mazeDict, which now reflects the current
navigable paths from currentCell, including any changes in the maze's layout or the cell's accessibility.
"""

def updateMazeNeighbors(mazeDict, currentCell, navNeighbors):
    for coordinate in mazeDict:
        if currentCell in mazeDict[coordinate]["neighbors"]:
            if coordinate not in navNeighbors:
                mazeDict[coordinate]["neighbors"].remove(currentCell)
    mazeDict[currentCell]["neighbors"] = navNeighbors
    return mazeDict

"""
getNextCell()
Description: The getNextCell function is designed to determine the next best move (or cell to navigate
to) from the current cell in a maze, based on a cost metric and visitation status. The mazeDict is a
dictionary representing the maze, where each key is a cell's coordinates, and the value is a dictionary
containing various properties of the cell, such as cost and whether it has been visited. The currentCell
parameter specifies the current position in the maze. The function evaluates all neighbors of the
currentCell (retrieved from mazeDict) to find the one with the lowest cost that has not been visited yet. If
all neighboring cells have been visited, the function will choose the neighbor with the lowest cost
regardless of its visitation status. The function returns the coordinates (nextMove) of the chosen
neighboring cell. If no valid move is available, it returns None
"""

def getNextCell(mazeDict, currentCell):
    neighborList = mazeDict[currentCell]["neighbors"]
    notVisited = []
    haveVisited = []
    for coordinateTup in neighborList:
        if mazeDict[coordinateTup]["visited"] == False:
            notVisited.append(coordinateTup)
        else:
            haveVisited.append(coordinateTup)
    if len(notVisited) != 0:
        minCost = 0
        index = 0
        for i in range(len(notVisited)):
            if mazeDict[notVisited[i]]["cost"] < minCost:
                minCost = mazeDict[notVisited[i]]["cost"]
                index = i
        return notVisited[i]
    else:
        minCost = 0
        index = 0
        for i in range(len(haveVisited)):
            if mazeDict[haveVisited[i]]["cost"] < minCost:
                minCost = mazeDict[haveVisited[i]]["cost"]
                index = i
        return haveVisited[i]

"""
checkCellArrived()
Description: The checkCellArrived function assesses whether a given cell, represented by currentCell,
matches the destination cell in a grid-based environment. Both currentCell and destination are tuples
containing the x and y coordinates of their respective cells (e.g., (x, y)). The function compares these two
tuples and returns True if they are identical, indicating that the current cell is indeed the destination.
Otherwise, it returns False, showing that the destination has not been reached.
"""

def checkCellArrived(currentCell, destination):
    (current_x, current_y) = currentCell
    (desired_x, desired_y) = destination
    if current_x == desired_x and current_y == desired_y:
        return True
    return False

def updateMazeCost(mazeDict, start, goal):
    for (i,j) in mazeDict.keys():
        mazeDict[(i,j)]["flooded"] = False
    queue = deque([goal])
    mazeDict[goal]['cost'] = 0
    mazeDict[goal]['flooded'] = True
    while queue:
        current = queue.popleft()
        current_cost = mazeDict[current]['cost']
        for neighbor in mazeDict[current]['neighbors']:
            if not mazeDict[neighbor]['flooded']:
                mazeDict[neighbor]['flooded'] = True
                mazeDict[neighbor]['cost'] = current_cost + 1
                queue.append(neighbor)
    return mazeDict


def printMazeGrid(mazeDict, nXCells, nYCells, attribute):
    for y in range(nYCells - 1, -1, -1):
        row = '| '
        for x in range(nXCells):
            cell_value = mazeDict[(x, y)][attribute]
            row += '{} | '.format(cell_value)
        print(row[:-1])

# === CREATE ROBOT OBJECT
robot = Create3(Bluetooth("XJ-9"))

# === FLAG VARIABLES
HAS_COLLIDED = False
HAS_ARRIVED = False

# === BUILD MAZE DICTIONARY
N_X_CELLS = 3
N_Y_CELLS = 3
CELL_DIM = 50
MAZE_DICT = createMazeDict(N_X_CELLS, N_Y_CELLS, CELL_DIM)
MAZE_DICT = addAllNeighbors(MAZE_DICT, N_Y_CELLS, N_Y_CELLS)

# === DEFINING ORIGIN AND DESTINATION
PREV_CELL = (0,1)
START = (0,2)
CURR_CELL = START
DESTINATION = (2,2)
MAZE_DICT[CURR_CELL]["visited"] = True

# === PROXIMITY TOLERANCES
WALL_THRESHOLD = 120


# ==========================================================
# FAIL SAFE MECHANISMS

# EITHER BUTTON
@event(robot.when_touched, [True, True])  # User buttons: [(.), (..)]
async def when_either_button_touched(robot):
    global HAS_COLLIDED
    HAS_COLLIDED = True
    for i in range(50):
        await robot.set_wheel_speeds(0, 0)
        await robot.set_lights(Robot.LIGHT_ON,Color(255,0,0))


# EITHER BUMPER
@event(robot.when_bumped, [True, True])  # [left, right]
async def when_either_bumped(robot):
    global HAS_COLLIDED
    HAS_COLLIDED = True
    for i in range(50):
        await robot.set_wheel_speeds(0, 0)
        await robot.set_lights(Robot.LIGHT_ON,Color(255,0,0))


# ==========================================================
# MAZE NAVIGATION AND EXPLORATION

# === NAVIGATE TO CELL
async def navigateToNextCell(robot, nextCell, orientation):
    global MAZE_DICT, PREV_CELL, CURR_CELL, CELL_DIM
    neighborList = getPotentialNeighbors(CURR_CELL, orientation)
    i = 0
    for (index, coordinate) in enumerate(neighborList):
        if nextCell == coordinate:
            i = index
    if i == 0:
        await robot.turn_left(90)
        await robot.move(CELL_DIM)
    elif i == 1:
        await robot.move(CELL_DIM)
    elif i == 2:
        await robot.turn_right(90)
        await robot.move(CELL_DIM)
    else:
        await robot.turn_left(180)
        await robot.move(CELL_DIM)
    MAZE_DICT[CURR_CELL]["visited"] = True
    PREV_CELL = CURR_CELL
    CURR_CELL = nextCell


# === EXPLORE MAZE
@event(robot.when_play)
async def navigateMaze(robot):
    global HAS_COLLIDED, HAS_ARRIVED
    global PREV_CELL, CURR_CELL, START, DESTINATION
    global MAZE_DICT, N_X_CELLS, N_Y_CELLS, CELL_DIM, WALL_THRESHOLD
    while HAS_COLLIDED == False and HAS_ARRIVED == False:
        ir_reading = (await robot.get_ir_proximity()).sensors
        pos = await robot.get_position()
        if checkCellArrived(CURR_CELL, DESTINATION) == True:
            await robot.set_wheel_speeds(0, 0)
            await robot.set_lights(Robot.LIGHT_SPIN, Color(0,255,0))
            break
        orientation = getRobotOrientation(pos.heading)
        wallsAroundCell = getWallConfiguration(ir_reading[0],ir_reading[3],ir_reading[6],WALL_THRESHOLD)
        potentialNeighbors = getPotentialNeighbors(CURR_CELL, orientation)
        neighbors = getNavigableNeighbors(wallsAroundCell, potentialNeighbors, PREV_CELL, N_X_CELLS, N_Y_CELLS)
        MAZE_DICT = updateMazeNeighbors(MAZE_DICT, CURR_CELL, neighbors)
        MAZE_DICT = updateMazeCost(MAZE_DICT, START, DESTINATION)
        nextCell = getNextCell(MAZE_DICT, CURR_CELL)
        await navigateToNextCell(robot, nextCell, orientation)

# start the robot
robot.play()
