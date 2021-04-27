"""HiderController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
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

# These are your pose values that you will update by solving the odometry equations
pose_x = 1
pose_y = 1
pose_theta = -1.5

#Controls the state machine for the epuck
state = "Explore"
#state = "Path_finder"

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


gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)



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




for i in range (21):
    if i == 10:
       lOffsets.append(0)
    else:
       lOffsets.append((count + i) * lAngle)
        
obs_dist = .1
tick = 0
map = np.zeros((360,360))
count = 0
#goalPoints = [(.57,.7),(-.38,.8),(.66,.33),(.74,-.53)]
goalPoints = [(.38,1.65)]
sCount = 0
timer = 0
f = 0

caughtTimer = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(SIM_TIMESTEP) != -1:
    # Read the sensors:
    lidar_sensor_readings = lidar.getRangeImage()
    
    display.setColor(0XFF0000)
    display.drawPixel(360-int(pose_x*180),360-int(pose_y*180))
    
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]
    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    # Process sensor data here.
    
    # print("Pose X: ", pose_x)
    # print("Pose Y: ",pose_y)
    
    #mapping mode for the epuck 
    if(state == "Explore"):
        #Timer for how long the explore function of the robot works for
        if(timer < 15000):  
            #Reads through the lidar readings  
            for i, rho in enumerate(lidar_sensor_readings):
                alpha = lOffsets[i]
    
                if rho > LIDAR_SENSOR_MAX_RANGE:
                    rho = LIDAR_SENSOR_MAX_RANGE
    
                worldtheta = pose_theta + 1.57
                #Get the world position for the end of the lidar beam             
                xr = math.cos(alpha)* rho
                yr = math.sin(alpha)* rho
                        
                wx = abs((xr*math.cos(worldtheta)) + (yr*math.sin(worldtheta)) + pose_x)
                wy = abs((yr*math.cos(worldtheta)) - (xr*math.sin(worldtheta)) + pose_y)
        
                if(wx > 2):
                    wx = 2
                if(wy > 2):
                    wy = 2        
               
                if(i == 10):
                
                    print("X: ",360-int(wx*180))
                    print("y: ",360-int(wy*180) )
                
                
    
                
                
                
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
                        if(map[int(wx*180)  ][(int(wy*180))] > .5):
                           
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
        
        
            
    elif(state == "Path_finder"):
        
        if(sCount < len(goalPoints)):  
        
            #print(sCount)
            if(sCount == 0):
            
                map[map>.1] = 1
                map[map<=.1] = 0
        
                filter = np.ones((10,10))
                cmap = convolve2d(map,filter,mode = "same")
                cmap[cmap>=1] = 1
                
                plt.imshow(cmap)
                plt.show()
                
                
            start_w = (pose_x,pose_y)
            end_w = goalPoints[sCount]  
            
            start = ((int(start_w[0]*180)) ,(int((start_w[1]*180))))
            end = (int(end_w[0]*180), int(end_w[1]*180) )
        
            print("Start_w: ", start_w)
            print("End_w: ", end_w)
            
            print("start: ", start)
            print("end: ", end)    
                
            path = path_planner(cmap, start, end)
        
            PathPoints = []
            
            for i in range(len(path)):
                x = float(path[i][0])
                y = float(path[i][1])
                #print(x)
                #print(y)
                PathPoint = (((x)/180),((y)/180))
                PathPoints.append(PathPoint)
                #final = len(PathPoints)
            
            sCount = sCount + 1
            
            pc = 0
            
            state = "Path_follower"
        
        
        else:
        
            state = "none"   
            
            
    elif(state == "Path_follower"):

        for i in PathPoints:
            # display.setColor(0X45b6fe)
            # display.drawPixel(i[0],i[1])
            print(i)
        # #plt.scatter(path[:,1],path[:,0])   
            
        # #plt.show()
        
        gain = .02

        if(f == 0):
            for i in PathPoints:
                display.setColor(0XFFFFFF)
                display.drawPixel(int((i[0]*180)-1),int(360 - (i[1]*180)-1))
                #print(i)
            # #plt.scatter(path[:,1],path[:,0])   
                
            # #plt.show()
            
            #print("Path Follower")
            
            gain = .1
            f = f+1 

        
        if(pc < len(PathPoints)):
        
            Dist_Error = math.sqrt(((pose_x - PathPoints[pc][0])**2) + ((pose_y - (PathPoints[pc][1]))**2))
       

            Bearing_Error = math.atan2((PathPoints[pc][1] - pose_y) , (PathPoints[pc][0] - pose_x)) + pose_theta
        

            Bearing_Error = math.atan2((PathPoints[pc][0] - pose_y) , (PathPoints[pc][1] - pose_x)) + pose_theta
            
            print("Dist Error: ", Dist_Error)
            
            print("Bearing_Error: ", Bearing_Error)

    
            #STEP 2: Controller
            xR = Dist_Error 
            thetaR = Bearing_Error 
           
            
            
            #STEP 3: Compute wheelspeeds
            if(Dist_Error > gain):
        

                if(Bearing_Error < -gain):
                    
                
                    if(abs(thetaR * MAX_SPEED * xR) > MAX_SPEED):
                        vR = MAX_SPEED/4
                    else:
                        vR = abs(thetaR * MAX_SPEED * xR)/4
                    vL = -vR
                      
                elif(Bearing_Error > gain):
                    
            
                    if(abs(thetaR * MAX_SPEED * xR) > MAX_SPEED):
                        vL = MAX_SPEED/4
                    else:
                        vL = abs(thetaR * MAX_SPEED * xR)/4

                if(Bearing_Error < -.3):
                    
                
                    if(abs(thetaR * MAX_SPEED * xR) > MAX_SPEED):
                        vR = MAX_SPEED
                    else:
                        vR = abs(thetaR * MAX_SPEED * xR)
                    vL = -vR
                      
                elif(Bearing_Error > .3):
                    
            
                    if(abs(thetaR * MAX_SPEED * xR) > MAX_SPEED):
                        vL = MAX_SPEED
                    else:
                        vL = abs(thetaR * MAX_SPEED * xR)

                    
                    vR = -vL
                
                
                else:    
                    if(xR*MAX_SPEED > MAX_SPEED):

                        vL = MAX_SPEED/2
                        vR = MAX_SPEED/2
                    else:
                        vL = xR*MAX_SPEED/2
                        vR = xR*MAX_SPEED/2

                        vL = MAX_SPEED
                        vR = MAX_SPEED
                    # else:
                        # vL = xR*MAX_SPEED
                        # vR = xR*MAX_SPEED

            else:
                vL = 0
                vR = 0
                pc = pc + 1
        
               
                
        else:
           vL = 0
           vR = 0 
           state = "Path_finder" 
        
        
        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)                 
    
    

    elif(state == "caught"):
    
        if(caughtTimer == 0):
            x = random.randint(0,1)
            led.set(1)
            caughtTimer = caughtTimer + 1
        
        if(caughtTimer < 500):
           
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
            led.set(0)
            caughtPose = (pose_x,pose_y)
            caughtTimer = 0
            state = "flee"       

    
    elif(state == "flee"):
        if(math.sqrt(((pose_x - caughtPose[0])**2) + ((pose_y - (caughtPose[1]))**2)) < 1):
            if(lidar_sensor_readings[10] <= .3):
                # print("Left Sensor: ", lidar_sensor_readings[0])
                # print("Right Sensor: ", lidar_sensor_readings[20])
                
            
                if(lidar_sensor_readings[0] > lidar_sensor_readings[20] and lidar_sensor_readings[0] - lidar_sensor_readings[20] >= .05 and count == 0):
                    vL = (-MAX_SPEED/4)
                    vR = (MAX_SPEED/4)
                    #print("left")
                elif(lidar_sensor_readings[0] < lidar_sensor_readings[20] and lidar_sensor_readings[20] - lidar_sensor_readings[0] >= .05 and count == 0):
                    vL = (MAX_SPEED/4)
                    vR = (-MAX_SPEED/4)
                    #print("right") 
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
            else:
            
                if(lidar_sensor_readings[0] <= obs_dist):
                    vL = (MAX_SPEED/4)
                    vR = (-MAX_SPEED/4)
                    #print("right") 
                elif(lidar_sensor_readings[20] <= obs_dist ):
                    vL = (-MAX_SPEED/4)
                    vR = (MAX_SPEED/4)
                    #print("left")
                else:
                    vL = MAX_SPEED/2
                    vR = MAX_SPEED/2    
        
            leftMotor.setVelocity(vL)
            rightMotor.setVelocity(vR)   
        
        else:
            state = "Path_finder"
                  
    else:
        vL = 0
        vR = 0
        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)   
    
    
    



# Enter here exit cleanup code.
