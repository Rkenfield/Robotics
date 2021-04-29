"""combinedcontroller."""


from controller import *
import csv
import numpy

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

def CLAMP(value, low, high):
    return max(min(value, high), low)

def SIGN(x):
    return numpy.sign(x)


TAKEOFF_THRESHOLD_VELOCITY = 163

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Get propeller motors and set to velocity mode
frontLeftMotor = robot.getDevice('front left propeller')
frontRightMotor = robot.getDevice('front right propeller')
backLeftMotor = robot.getDevice('rear left propeller')
backRightMotor = robot.getDevice('rear right propeller')

frontLeftMotor.setPosition(float('inf'))
frontRightMotor.setPosition(float('inf'))
backLeftMotor.setPosition(float('inf'))
backRightMotor.setPosition(float('inf'))

frontLeftMotor.setVelocity(1.0)
frontRightMotor.setVelocity(1.0)
backLeftMotor.setVelocity(1.0)
backRightMotor.setVelocity(1.0)



# Get and enable devices:
camera = Camera("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)
gps1 = GPS("gps")
gps1.enable(timestep)
imu = InertialUnit("inertial unit")
imu.enable(timestep)
compass = Compass("compass")
compass.enable(timestep)
gyro = Gyro("gyro")
gyro.enable(timestep)
camera_roll_motor = robot.getDevice("camera roll")
camera_pitch_motor = robot.getDevice("camera pitch")

print("Starting drone...")

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if (robot.getTime() > 1.0):
        break

k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3.0
k_roll_p = 50.0
k_pitch_p = 30.0

target_altitude = 1.0 # can be changed based off needs

import math
import time
import random
import copy
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d
from controller import Robot, Motor, DistanceSensor

LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_BINS = 21 # 21 Bins to cover the angular range of the lidar, centered at 10
LIDAR_ANGLE_RANGE = 1.5708 # 90 degrees, 1.5708 radians

#Robot starts in the middle of a 2 by 2 map
pose_x = 1
pose_y = 1
pose_theta = 0

#Controls the state machine for the epuck
state = "Explore"

#code should automatically switch between states
#state = "Path_finder"
#state = "Path_follower"

#needs to be set by the supervisor
#state = "caught"
#set from the caught state
#state = "flee"



# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
MAX_SPEED = 6.28

# create the Robot instance.
robot=Robot()

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

led = robot.getDevice('led0')

receiver = robot.getDevice('receiver')

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())


# Initialize the Display
display = robot.getDevice("display")


# get and enable lidar
lidar = robot.getDevice("LDS-01")
lidar.enable(SIM_TIMESTEP)
lidar.enablePointCloud()



#set up lidar
lidar_sensor_readings = lidar.getRangeImage()
lOffsets = []
lAngle = LIDAR_ANGLE_RANGE / LIDAR_ANGLE_BINS
count = -10

#set up offsets for lidar
for i in range (21):
    if i == 10:
       lOffsets.append(0)
    else:
       lOffsets.append((count + i) * lAngle)


#set up the gps
gps2 = robot.getDevice("gps")
gps2.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)

receiver.enable(SIM_TIMESTEP)

# Ground Sensor Initialization
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)
for i in range(10): robot.step(SIM_TIMESTEP)

#path planner function A*
class pathNode:
    def __init__(self, parent = None, position = None):
        self.parent = parent
        self.position = position
        #distance from start
        self.d = 0
        #remaining heuristic/ distance from end
        self.h = 0
        #total travel/ d + h
        self.f = 0
    #overide == operator to make comparisons easier
    def __eq__(self, other):
        return self.position == other.position

def path_planner(map, start, end):


    if map[start[0]][start[1]] == 1 or map[end[0]][end[1]] == 1:

        return []


    startN = pathNode(None, start)
    endN = pathNode(None, end)

    openL = []
    closedL = []

    openL.append(startN)

    while (len(openL) > 0):
        #print("in loop")
        cIndex = 0
        cNode = openL[cIndex]

        for i in range(len(openL)):
            if (openL[i].f < cNode.f):
                cIndex = i
                cNode = openL[i]


        closedL.append(cNode)

        openL.pop(cIndex)

        if cNode == endN:

            path = []
            current = cNode

            # when end node is found work back to the start node and save the path
            while current is not None:
                path.append(current.position)
                current = current.parent
            # path is reversed since it works backwards so flip when returning
            return list(reversed(path))

        # get surrounding indeces from map
        adjacent = []
        for i in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]:

            nPos = (cNode.position[0] + i[0], cNode.position[1] + i[1])
            # make sure indeces are inside the bounds of 360
            if (nPos[0] < 360 and nPos[0] >= 0 and nPos[1] < 360 and nPos[1] >= 0):
                # check if path is open
                if (map[nPos[0]][nPos[1]] == 0):
                    adjacentNode = pathNode(cNode, nPos)
                    adjacent.append(adjacentNode)
                else:
                    continue

        # check the possible next positions
        for neighbor in adjacent:

            # make sure the node hasnt been visited
            if neighbor in closedL:
                continue


            # otherwise set the f value for the node/path

            # Add root 2 to distance from source for diagonal children since moving diagonal is root 2 not 1
            if (neighbor.position[0] != neighbor.parent.position[0] and neighbor.position[1] != neighbor.parent.position[1]):
                neighbor.d = cNode.d + np.sqrt(2)
            else:
                neighbor.d = cNode.d + 1


            neighbor.h = math.sqrt(
                ((endN.position[0] - neighbor.position[0]) ** 2) + ((endN.position[1] - neighbor.position[1]) ** 2))

            neighbor.f = neighbor.d + neighbor.h

            # double check that current node has lowest f value ensuring the minimum path is taken
            for i in openL:
                if neighbor == i:
                    if neighbor.d > i.d:
                         break
                    else:

                        i.d = neighbor.d
                        i.f = i.d + i.h
                        i.parent = neighbor.parent
                        break
            else:
                openL.append(neighbor)

    return []



#variables for looping through different behaviors
obs_dist = .1
tick = 0
#set map of 360 by 360 to all 0's
map = np.zeros((360,360))
count = 0

goalPoints = [(.38,1.65),(1,1)]
sCount = 0
timer = 0
f = 0
a = 0
ncount = 0
caughtTimer = 0

previousaa = 0
aa = 0
c = 0
Bearing_Error = 0

#Variable used to keep track of number of tasks achieved
task = 0

while robot.step(timestep) != -1:

    time = robot.getTime()

    # Obtain ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # Retrieve robot position using sensors
    roll = InertialUnit.getRollPitchYaw(imu)[0] + numpy.pi/2.0
    pitch = InertialUnit.getRollPitchYaw(imu)[1]
    altitude = GPS.getValues(gps)[1]
    roll_acceleration = Gyro.getValues(gyro)[0]
    pitch_acceleration = Gyro.getValues(gyro)[1]

    Motor.setPosition(camera_roll_motor, -0.115 * roll_acceleration)
    Motor.setPosition(camera_pitch_motor, -0.1 * pitch_acceleration)

    #Computing roll, pitch, yaw, and vertical inputs
    roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration
    pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) - pitch_acceleration
    #yaw_input = yaw_disturbance
    clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
    vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0)

    #Actuate motors, taking into consideration all computed inputs ^^
    front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input
    front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input
    rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input
    rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input
    frontLeftMotor.setVelocity(front_left_motor_input);
    frontRightMotor.setVelocity(-front_right_motor_input);
    backLeftMotor.setVelocity(-rear_left_motor_input);
    backRightMotor.setVelocity(rear_right_motor_input);

    # Read the sensors:
    lidar_sensor_readings = lidar.getRangeImage()
    #draws where epuck is in the display
    display.setColor(0XFF0000)
    display.drawPixel(360-int(pose_x*180),360-int(pose_y*180))

    #use gps to track position of epuck
    pose_y = gps2.getValues()[2]
    pose_x = gps2.getValues()[0]
    n = compass.getValues()
    #This one is for the bearing error
    rad = math.atan2(n[0], n[2])
    #this one is for the display and mapping
    rads = -((math.atan2(n[0], n[2]))-1.5708)
    #for beaeing error
    pose_theta = -rad - 1.5708

    #for drawing on display and mapping
    worldtheta = rads + 1.5708
    # Process sensor data here.

    # print("Pose X: ", pose_x)
    # print("Pose Y: ",pose_y)
    if(receiver.getQueueLength() > 0 and receiver.getQueueLength() < 2):
        if(c != 0):
            msg = receiver.getData()
            state = "caught"
            c = 0
            #print("caught")
            receiver.nextPacket()
    else:
        if(c != 1):
            #print("No connection")
            c = 1
        receiver.nextPacket()




    #mapping mode for the epuck
    if(state == "Explore"):
        #Timer for how long the explore function of the robot works for
        if(timer < 20000):
            #Reads through the lidar readings
            for i, rho in enumerate(lidar_sensor_readings):
                alpha = lOffsets[i]

                if rho > LIDAR_SENSOR_MAX_RANGE:
                    rho = LIDAR_SENSOR_MAX_RANGE


                #Get the world position for the end of the lidar beam
                xr = math.cos(alpha)* rho
                yr = math.sin(alpha)* rho

                wx = abs((xr*math.cos(worldtheta)) + (yr*math.sin(worldtheta)) + pose_x)
                wy = abs((yr*math.cos(worldtheta)) - (xr*math.sin(worldtheta)) + pose_y)

                if(wx > 2):
                    wx = 2
                if(wy > 2):
                    wy = 2

                # if(i == 10):

                    # print("X: ",360-int(wx*180))
                    # print("y: ",360-int(wy*180) )






                #Tracks when obstacles get within a certain range to the epuck
                if rho < 0.15*LIDAR_SENSOR_MAX_RANGE:

                    #display.setColor(0X0000FF)
                    #display.drawPixel(360-int(wy*30),int(wx*30))
                    #display.drawPixel(int((wx + 1)*180),int((wy + 1)*180))

                    #ensures the measurement of the lidar is within the bounds of the map
                    if(abs(int(wx*180) ) < 360 and abs(int(wy*180) ) < 360):
                        #if the spot in the map is not already at max increment the value
                        if( map[int(wx*180) ][(int(wy*180)) ] < 1):

                            map[int(wx*180) ][(int(wy*180)) ] = map[int(wx*180) ][(int(wy*180)) ] + .01
                            #print( map[int((wx + 1)*180) - 1 ][int((wy + 1)*180) - 1])

                        #if the value at the index is greater than the max make sure to cap the max value
                        else:
                            map[int(wx*180) ][(int(wy*180))] = 1
                        #if the value at the index in the map is incremented to a certain point than set it to max confirming it is an obstacle
                        if(map[int(wx*180)  ][(int(wy*180))] > .2):

                            map[int(wx*180)][(int(wy*180))] = 1

                        #Draw the Obstacles in the display
                        display.setColor( int((((map[int(wx*180) ][(int(wy*180)) ]*256)**2) + (map[int(wx*180) ][(int(wy*180))]*256) + map[int(wx*180) ][(int(wy*180)) ])*255))

                        display.drawPixel(360-int(wx*180),360-int(wy*180))





            #This handles the basic driving around the map:
            #When the front sensor is within a certain distance the epuck needs to turn
            if(lidar_sensor_readings[10] <= .3):

                #If the left side has more room than the right and and the difference between the two are greater than a certain gain then turn left
                if(lidar_sensor_readings[0] > lidar_sensor_readings[20] and lidar_sensor_readings[0] - lidar_sensor_readings[20] >= .05 and count == 0):
                    vL = (-MAX_SPEED/4)
                    vR = (MAX_SPEED/4)
                    #print("left")
                # if the right side has more room and the difference between the two are greater than a certain gain then turn right
                elif(lidar_sensor_readings[0] < lidar_sensor_readings[20] and lidar_sensor_readings[20] - lidar_sensor_readings[0] >= .05 and count == 0):
                    vL = (MAX_SPEED/4)
                    vR = (-MAX_SPEED/4)
                    #print("right")
                #if the sensors are equal or not greater than the gain then choose one side randomly and turn that direction
                else:
                    # The count makes sure that it doesnt choose a random direction back and forth but chooses one and then continues to turn that direction
                    if(count == 0):
                        x = random.randint(0,1)

                    if(x % 2 == 0):
                        vL = (MAX_SPEED/4)
                        vR = (-MAX_SPEED/4)
                        #print("right")
                    else:
                        vL = (-MAX_SPEED/4)
                        vR = (MAX_SPEED/4)
                        #print("left")

                    count = count + 1
                    #After turning a certain amount reset
                    if(count >= 100):
                        count = 0
            #If the forward sensor isnt facing the wall but one of the side sensors are too close to the wall the epuck needs to adjust
            else:

                #If the left sensor is too close to an obstacle turn right
                if(lidar_sensor_readings[0] <= obs_dist):
                    vL = (MAX_SPEED/4)
                    vR = (-MAX_SPEED/4)
                    #print("right")
                #If the right sensor is too close to an obstacle turn left
                elif(lidar_sensor_readings[20] <= obs_dist ):
                    vL = (-MAX_SPEED/4)
                    vR = (MAX_SPEED/4)
                    #print("left")
                #Otherwise just drive straight
                else:
                    vL = MAX_SPEED/2
                    vR = MAX_SPEED/2

            leftMotor.setVelocity(vL)
            rightMotor.setVelocity(vR)

            #Increment timer for how long the epuck should be in mapping/ exploring mode
            timer += 1

            #print(timer)
        #When the time runs out for the epuck discovery mode then the epuck switches to using the map it has created to drive through the level rather than obstacle sensors
        else:

            state = "Path_finder"
            timer = 0


    #When the explorer state has mapped out a bit of the map it switches to using said map to find paths to points
    elif(state == "Path_finder"):

        #Makes sure each goal point has a path after one is reached
        if(sCount < len(goalPoints)):

            #print(sCount)
            #if its the first time into the path finder
            if(sCount == 0):

                #sets up the convolved map gotten from explorer state
                map[map>.2] = 1
                map[map<=.2] = 0

                filter = np.ones((10,10))
                cmap = convolve2d(map,filter,mode = "same")
                cmap[cmap>=1] = 1

                plt.imshow(cmap)
                plt.show()

            #gets the start and end in the world for the robot and goal point
            start_w = (pose_x,pose_y)
            end_w = goalPoints[sCount]

            #changes those robot and end points coordinates into map based coordinates
            start = ((int(start_w[0]*180)) ,(int((start_w[1]*180))))
            end = (int(end_w[0]*180), int(end_w[1]*180) )

            # print("Start_w: ", start_w)
            # print("End_w: ", end_w)

            # print("start: ", start)
            # print("end: ", end)

            #calls the path planner to get the path from the robots current position to the target point using A*
            path = path_planner(cmap, start, end)

            PathPoints = []

            #takes the path created and changes the points in the path from map based coordinates into world based coordinates
            for i in range(len(path)):
                x = float(path[i][0])
                y = float(path[i][1])
                #print(x)
                #print(y)
                PathPoint = (((x)/180),((y)/180))
                PathPoints.append(PathPoint)
                #final = len(PathPoints)

            #increments the number of paths
            sCount = sCount + 1
            #sets up a counter variable for the number of points in the path reached
            pc = 0
            #print(len(path))
            #with the gotten path enter the next state being the follower
            state = "Path_follower"


        else:
            #if all the points have been reached then exit the state machine epuck has finished task
            state = "none"


    elif(state == "Path_follower"):

        #if it is the first time entering the follower for the current path
        if(f == 0):
            for i in PathPoints:
                display.setColor(0XFFFFFF)
                display.drawPixel(360-int((i[0]*180)),int(360 - (i[1]*180)))
                #print(i)
            # #plt.scatter(path[:,1],path[:,0])

            # #plt.show()

            #print("Path Follower")

            gain = .05
            f = f+1

        #Supervisor logic for when the drone catches the epuck (ie. the drone and epuck are in the same location)
        #If the drone (gps1) and the epuck (gps 2) have the same x and y positions, the controller switches to "caught" (ie. the logic
        #used to change the epuck's path )
        if((gps1.getValues()[2] == gps2.getValues()[2]) &&  (gps1.getValues()[0] == gps2.getValues()[0])):
            state = "caught"

        #Supervisor logic for when a task is complete
        if((gsr[0] || gsr[1]) || gsr[2] || gsr[3] || gsr[4]) || gsr[5] || gsr[6] || gsr[7]) > 800):
            task++

        if(pc < len(PathPoints)):

            #get the distance from the robot to the next point
            Dist_Error = math.sqrt(((pose_x - PathPoints[pc][0])**2) + ((pose_y - (PathPoints[pc][1]))**2))
            Bearing_Error = math.atan2((PathPoints[pc][1] - pose_y) , (PathPoints[pc][0] - pose_x))

            if(abs(previousaa) - abs(aa) < .1):
                aa = previousaa
            else:
                aa = Bearing_Error

            previousaa = aa
            if(abs(previousaa - aa) > math.pi):
                aa = -aa


            xR = Dist_Error
            #thetaR = aa

            # if(pose_theta > math.pi):
                # pose_theta -= 2*math.pi


            #get the difference between the robots angle and the position of the next point

            thetaR = aa + pose_theta + math.pi/2
            # #if the robot is a certain distance from the next point
            if(Dist_Error > gain):

                if(abs(thetaR)>.5):
                    new = 0
                    dtheta = -10*thetaR
                else:
                    new = 5*Dist_Error
                    dtheta = -2*thetaR



                vL = (new - (dtheta*(EPUCK_AXLE_DIAMETER/2)))
                vR = (new + (dtheta*(EPUCK_AXLE_DIAMETER/2)))


                if(vL > MAX_SPEED/4):
                    vL = MAX_SPEED/4
                elif(vL < -MAX_SPEED/4):
                    vL = -MAX_SPEED/4

                if(vR > MAX_SPEED/4):
                    vR = MAX_SPEED/4
                elif(vL < -MAX_SPEED/4):
                    vR = -MAX_SPEED/4

                print("WorldTheta: ", pose_theta,"Bearing_Error: ",Bearing_Error," ThetaR: ", thetaR," New: ", new," dtheta: ", dtheta , " VL: ", vL, "VR: ", vR)




            #if the robot has reached the desired point then set speed to 0 and get the next point skipping a few points to speed up driving
            else:
                vL = 0
                vR = 0
                pc = pc + 3
                #print(pc)


            leftMotor.setVelocity(vL)
            rightMotor.setVelocity(vR)
        #if the final point has been reached then we can switch back to the path finder to get the next goal point's path
        else:
           vL = 0
           vR = 0
           state = "Path_finder"


        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)


    #the supervisor is supposed to trigger this state telling the Epuck when it has been caught
    elif(state == "caught"):
        #In caught state the epucks lights will turn on and it will spin in place for a certain amount of time
        if(caughtTimer == 0):
            #gets a random direction for the Epuck to turn
            x = random.randint(0,1)
            #turns on the lights for the epuck
            led.set(1)
            caughtTimer = caughtTimer + 1

        if(caughtTimer < 500):
            #turns said random direction
            if(x % 2 == 0):
                vL = (MAX_SPEED/2)
                vR = (-MAX_SPEED/2)

            else:
                vL = (-MAX_SPEED/2)
                vR = (MAX_SPEED/2)
            leftMotor.setVelocity(vL)
            rightMotor.setVelocity(vR)
            caughtTimer = caughtTimer + 1

        else:
            #after the set amount of time it turns off the light and enters the flee state
            led.set(0)
            #saves the area it was caught for the flee state to use
            caughtPose = (pose_x,pose_y)
            caughtTimer = 0
            state = "flee"


    elif(state == "flee"):
        #gets the distance between the Epuck and where it was caught
        if(math.sqrt(((pose_x - caughtPose[0])**2) + ((pose_y - (caughtPose[1]))**2)) < 1):
            #similar to the Explorer state the Epuck drives using its sensors to avoid obstacles until it is a certain distance from where it was caught
            #checks if obstacle is directly in front of robot
            if(lidar_sensor_readings[10] <= .3):
                # print("Left Sensor: ", lidar_sensor_readings[0])
                # print("Right Sensor: ", lidar_sensor_readings[20])

                #turns left depending on how far obstacle is from left sensor
                if(lidar_sensor_readings[0] > lidar_sensor_readings[20] and lidar_sensor_readings[0] - lidar_sensor_readings[20] >= .05 and count == 0):
                    vL = (-MAX_SPEED/4)
                    vR = (MAX_SPEED/4)
                    #print("left")
                #turns right depending on how far obstacle is from right sensor
                elif(lidar_sensor_readings[0] < lidar_sensor_readings[20] and lidar_sensor_readings[20] - lidar_sensor_readings[0] >= .05 and count == 0):
                    vL = (MAX_SPEED/4)
                    vR = (-MAX_SPEED/4)
                    #print("right")
                #if neither left or right is acceptable or both are acceptable then just turn a random direction for a certain amount
                else:
                    if(count == 0):
                        x = random.randint(0,1)

                    if(x % 2 == 0):
                        vL = (MAX_SPEED/4)
                        vR = (-MAX_SPEED/4)
                        #print("right")
                    else:
                        vL = (-MAX_SPEED/4)
                        vR = (MAX_SPEED/4)
                        #print("left")

                    count = count + 1

                    if(count >= 100):
                        count = 0
            #if an obstacle is too far from the front sensor but close enough to the left and right then the epuck turns the needed direction
            else:
                #turns right if left sensor is too close to the wall
                if(lidar_sensor_readings[0] <= obs_dist):
                    vL = (MAX_SPEED/4)
                    vR = (-MAX_SPEED/4)
                    #print("right")
                #turns left if the right sensor is too close to the wall
                elif(lidar_sensor_readings[20] <= obs_dist ):
                    vL = (-MAX_SPEED/4)
                    vR = (MAX_SPEED/4)
                    #print("left")
                #drives straight otherwise
                else:
                    vL = MAX_SPEED/2
                    vR = MAX_SPEED/2

            leftMotor.setVelocity(vL)
            rightMotor.setVelocity(vR)

        else:
            #counter variable for the
            c = 1
            #after done fleeing then re enter the path finder state to find a path from the epucks current position to the next goal point
            state = "Explore"

    #Supervisor logic for when the epuck has won the game (ie. managed to avoid getting caught)
    else:
        #Stop the epuck
        vL = 0
        vR = 0
        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)
        #Stop the drone
        frontLeftMotor.setVelocity(front_left_motor_input);
        frontRightMotor.setVelocity(-front_right_motor_input);
        backLeftMotor.setVelocity(-rear_left_motor_input);
        backRightMotor.setVelocity(rear_right_motor_input);
        if(task == 4):
            print("Epuck wins!")


# Enter here exit cleanup code.
