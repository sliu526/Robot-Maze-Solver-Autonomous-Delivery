from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note


import math as m

"""
getCorrectionAngle()
Description: Calculate the angle required for the robot to orient itself with the vertical axis by rotating to
the right. The returned value should be an integer and no rounding should take place, meaning that float
values should have their decimals truncated. The input heading will be a float corresponding to the current
robot heading in degrees. The output value should be the angle in degrees by which the robot should
turn to the right in order to be oriented vertically.
"""

def getCorrectionAngle(heading):
    integer = int(heading)
    correctionAngle = integer - 90
    return correctionAngle

"""
getAngleToDestination()
Description: Calculate the angle required for the robot to face the destination assuming it is currently
oriented along the vertical axis, i.e. it has a heading of 90 degrees. The returned value should be an
integer and no rounding should take place, meaning that float values should have their decimals
truncated. The input currentPosition will be a tuple containing the current x and y robot coordinates in
centimeters. The input destination tuple will contain the x and y goal in centimeters. The output value
should be the angle in degrees by which the robot should turn to the right to face the destination,
assuming it is currently oriented along the vertical axis.
"""

def getAngleToDestination(current_position,destination):
    (input_x, input_y) = current_position
    (desired_x, desired_y) = destination
    if desired_y < input_y:
        opposite = input_y - desired_y
        if input_x > desired_x:
            adjacent = input_x - desired_x
            angle = (-180 + m.degrees(m.atan(opposite/adjacent)))
        else:
            adjacent = desired_x - input_x
            angle = 90 + m.degrees(m.atan(opposite/adjacent))
        return int(angle)
    if desired_y > input_y:
        adjacent = desired_y - input_y
        opposite = input_x - desired_x
        angle = -(m.degrees(m.atan(opposite/adjacent)))
    return int(angle)

"""
getMinProxApproachAngle()
Description: Determine the minimum proximity and corresponding approach angle to an obstacle based
on sensor readings. The input readings will be the list containing the measurements taken from the array
of IR proximity sensors. The angles will be a list of the same length, corresponding to the angles of each
IR sensor with respect to its central forward axis. The return value should be a tuple containing the
minimum proximity to an obstacle (float) and the corresponding IR sensor angle (float)
"""

def getMinProxApproachAngle(readings, angles):
    proxList = []
    for i in range(len(angles)):
        proximity = 4095/(readings[i] + 1)
        aTup = (proximity, i)
        proxList.append(aTup)
    minProxTup = min(proxList)
    minProx = minProxTup[0]
    minIndex = minProxTup[1]
    correspondingAngle = angles[minIndex]
    finalTup = (minProx, correspondingAngle)
    return finalTup

"""
checkPositionArrived()
Description: Check if the robot has reached the destination, by calculating the distance between the
current position and the destination and checking whether that is below the input threshold. The input
currentPosition will be a tuple containing the current x and y robot coordinates in centimeters. The input
destination tuple will contain the x and y goal in centimeters. The threshold will be a distance also in
centimeters.
"""

def checkPositionArrived(current_position, destination, threshold):
    (current_x, current_y) = current_position
    (new_x, new_y) = destination
    distance = m.sqrt((current_x - new_x)**2 + (current_y - new_y)**2)
    if distance <= threshold:
        arrivedToDestination = True
        return arrivedToDestination
    else:
        arrivedToDestination = False
        return arrivedToDestination


# === CREATE ROBOT OBJECT
robot = Create3(Bluetooth("XJ-9"))

# === FLAG VARIABLES
HAS_COLLIDED = False
HAS_REALIGNED = False
HAS_FOUND_OBSTACLE = False
HAS_ARRIVED = False

# === OTHER NAVIGATION VARIABLES
SENSOR2CHECK = 0
IR_ANGLES = [-65.3, -38.0, -20.0, -3.0, 14.25, 34.0, 65.3]
DESTINATION = (0, 300)
ARRIVAL_THRESHOLD = 5


# ==========================================================
# FAIL SAFE MECHANISMS

# EITHER BUTTON
@event(robot.when_touched, [True, True])  # User buttons: [(.), (..)]
async def when_either_button_touched(robot):
    for i in range(50):
        await robot.set_wheel_speeds(0, 0)
        await robot.set_lights(Robot.LIGHT_ON,Color(255,0,0))


# EITHER BUMPER
@event(robot.when_bumped, [True, True])  # [left, right]
async def when_either_bumped(robot):
    for i in range(50):
        await robot.set_wheel_speeds(0, 0)
        await robot.set_lights(Robot.LIGHT_ON,Color(255,0,0))

# ==========================================================

# === REALIGNMENT BEHAVIOR
async def realignRobot(robot):
    global DESTINATION
    global HAS_REALIGNED
    await robot.set_wheel_speeds(0, 0)
    pos = await robot.get_position()
    current_position = (pos.x, pos.y)
    theta = getCorrectionAngle(pos.heading)
    await robot.turn_right(theta)
    angle = getAngleToDestination(current_position, DESTINATION)
    await robot.turn_right(angle)
    HAS_REALIGNED = True
    await robot.set_wheel_speeds(5, 5)


# === MOVE TO GOAL
async def moveTowardGoal(robot): 
    global HAS_ARRIVED, HAS_FOUND_OBSTACLE, IR_ANGLES, SENSOR2CHECK
    ir_reading = (await robot.get_ir_proximity()).sensors
    (proximity, angle) = getMinProxApproachAngle(ir_reading, IR_ANGLES)
    if proximity < 20:
        HAS_FOUND_OBSTACLE = True
        await robot.set_wheel_speeds(0, 0)
        if angle >= 0:
            await robot.turn_left(90 - angle)
            SENSOR2CHECK = 6
            await robot.set_wheel_speeds(5, 5)
        elif angle < 0:
            await robot.turn_right(90 + angle)
            SENSOR2CHECK = 0
            await robot.set_wheel_speeds(5, 5)


# === FOLLOW OBSTACLE
async def followObstacle(robot): 
    global HAS_FOUND_OBSTACLE, HAS_REALIGNED, SENSOR2CHECK
    await robot.set_wheel_speeds(5, 5)
    ir_reading = (await robot.get_ir_proximity()).sensors
    proximity = 4095/(ir_reading[SENSOR2CHECK] + 1)
    if proximity < 20:
        HAS_FOUND_OBSTACLE = True
        if SENSOR2CHECK == 0:
            await robot.turn_right(3)
            await robot.set_wheel_speeds(5, 5)
        elif SENSOR2CHECK == 6:
            await robot.turn_left(3)
            await robot.set_wheel_speeds(5, 5)
    elif proximity >= 100:
        await robot.set_wheel_speeds(0, 0)
        await robot.move(40)
        HAS_REALIGNED = False
        HAS_FOUND_OBSTACLE = False
        await robot.set_wheel_speeds(5, 5)


# === NAVIGATION TO DELIVERY
@event(robot.when_play)
async def makeDelivery(robot):
    global HAS_ARRIVED, HAS_COLLIDED, HAS_REALIGNED, HAS_FOUND_OBSTACLE
    global DESTINATION, ARRIVAL_THRESHOLD, IR_ANGLES, SENSOR2CHECK
    
    while HAS_COLLIDED == False:

        pos2 = await robot.get_position()
        current = (pos2.x, pos2.y)
        HAS_ARRIVED = checkPositionArrived(current, DESTINATION, ARRIVAL_THRESHOLD)
        
        if HAS_ARRIVED == True:
            await robot.set_lights(Robot.LIGHT_SPIN, Color(0,255,0))
            await robot.set_wheel_speeds(0, 0)
            break
            
        if HAS_REALIGNED == False:
            await realignRobot(robot)

        if HAS_ARRIVED == False and HAS_FOUND_OBSTACLE == False and HAS_REALIGNED == True:
            await moveTowardGoal(robot)

        if HAS_FOUND_OBSTACLE == True:
            await followObstacle(robot)

# start the robot
robot.play()
