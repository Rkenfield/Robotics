"""HiderController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

#Created by Ryan Kenfield

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
gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)

receiver.enable(SIM_TIMESTEP)


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

goalPoints = [(.38,1.65)]
#numerous counter variables for different timers on behaviours 
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
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(SIM_TIMESTEP) != -1:
    # Read the sensors:
    lidar_sensor_readings = lidar.getRangeImage()
    #draws where epuck is in the display
    display.setColor(0XFF0000)
    display.drawPixel(360-int(pose_x*180),360-int(pose_y*180))
    
    #use gps to track position of epuck
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]
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
    
    #receives the signal from the seeker Epuck when in a certain range to the seeker
    #if there is a message from the seeker telling the hider it has gotten too close the hider will go to its caught and flee state
    if(receiver.getQueueLength() > 0 and receiver.getQueueLength() < 2):
        #flag to ensure single trigger per overlap
        if(c != 0):
            msg = receiver.getData()
            state = "caught"
            c = 0
            
            #empties out the duplicate messages from the seeker to the hider
            receiver.nextPacket()
    else:
        #flag to ensure single trigger per exit of overlap
        if(c != 1):
            #print("No connection")
            c = 1
        #empties out the duplicate messages from the seeker to the hider
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
                
                #cap the end of the lidar lazers to the end of the map
                if(wx > 2):
                    wx = 2
                if(wy > 2):
                    wy = 2        
               
    
                
                
                
                #Tracks when obstacles get within a certain range to the epuck
                if rho < 0.15*LIDAR_SENSOR_MAX_RANGE:
       
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
            #draw the path in the display
            for i in PathPoints:
                display.setColor(0XFFFFFF)
                display.drawPixel(360-int((i[0]*180)),int(360 - (i[1]*180)))
           
            
            gain = .05
            f = f+1 

        #counter for each point in path
        if(pc < len(PathPoints)):
        
           
            
            #get the distance from the robot to the next point
            Dist_Error = math.sqrt(((pose_x - PathPoints[pc][0])**2) + ((pose_y - (PathPoints[pc][1]))**2))
            #get error in angle
            Bearing_Error = math.atan2((PathPoints[pc][1] - pose_y) , (PathPoints[pc][0] - pose_x)) 
            
            #attempt to ensure the bearing error doesn't flip back and forth between positive and negative reading of the same value
            if(abs(previousaa) - abs(aa) < .1):
                aa = previousaa
            else:
                aa = Bearing_Error
           
               
            previousaa = aa
            #attempt to ensure that bearing flips to opposite reading if it spins too far
            if(abs(previousaa - aa) > math.pi):
                aa = -aa

 
            xR = Dist_Error
            #thetaR = aa
            #if rotation is passed pi then get the difference 
            if(pose_theta > math.pi):
                pose_theta -= 2*math.pi
            
               
            
            #attempt to get the correct difference between epuck and point
            thetaR = aa + pose_theta + math.pi/2 
            # #if the robot is a certain distance from the next point
            if(Dist_Error > gain):
                #acceptance value for the rotation error tells whether to turn left or right
                #code below gotten with help from Anuj and Shivendra in order to fix Compass issues in Webots
                if(abs(thetaR)>.5):
                    new = 0
                    dtheta = -10*thetaR
                else:
                    new = 5*Dist_Error
                    dtheta = -2*thetaR
                    
                
                #set vL and vR based on above set issues    
                vL = (new - (dtheta*(EPUCK_AXLE_DIAMETER/2)))
                vR = (new + (dtheta*(EPUCK_AXLE_DIAMETER/2)))
                
                #cap max speed
                if(vL > MAX_SPEED/4):
                    vL = MAX_SPEED/4
                elif(vL < -MAX_SPEED/4):
                    vL = -MAX_SPEED/4
                    
                if(vR > MAX_SPEED/4):
                    vR = MAX_SPEED/4
                elif(vL < -MAX_SPEED/4):
                    vR = -MAX_SPEED/4
                
                #angle debug line
                #print("WorldTheta: ", pose_theta,"Bearing_Error: ",Bearing_Error," ThetaR: ", thetaR," New: ", new," dtheta: ", dtheta , " VL: ", vL, "VR: ", vR)
                  
                
                 

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
    
    
    #when seeker tells hider that it has gotten too close caught state is entered
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
    #when the epuck has completed all goals then the system is done              
    else:
        vL = 0
        vR = 0
        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)   
        print("Hider has exited")
    
    
    



# Enter here exit cleanup code.
