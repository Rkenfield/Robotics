"""HiderController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import time
import random
import copy
import numpy as np
from matplotlib import pyplot as plt
from controller import Robot, Motor, DistanceSensor

LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_BINS = 21 # 21 Bins to cover the angular range of the lidar, centered at 10
LIDAR_ANGLE_RANGE = 1.5708 # 90 degrees, 1.5708 radians

# These are your pose values that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0


state = "basic_mode"


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

map = np.zeros((360,360))
count = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(SIM_TIMESTEP) != -1:
    # Read the sensors:
    lidar_sensor_readings = lidar.getRangeImage()
   
    # Process sensor data here.
    if(state == "basic_mode"):
    
        for i, rho in enumerate(lidar_sensor_readings):
            alpha = -lOffsets[i]

            if rho > LIDAR_SENSOR_MAX_RANGE:
                rho = LIDAR_SENSOR_MAX_RANGE

          
                        
            xr = math.sin(alpha)* rho
            yr = math.cos(alpha)* rho
                    
            wx = (xr*math.cos(pose_theta)) + (yr*math.sin(pose_theta)) + pose_x
            wy = (yr*math.cos(pose_theta)) - (xr*math.sin(pose_theta)) + pose_y
               
            
            if rho < 0.5*LIDAR_SENSOR_MAX_RANGE:
    
                #display.setColor(0X0000FF)
                #display.drawPixel(360-int(wy*30),int(wx*30))
                #display.drawPixel(int((wx + 1)*180),int((wy + 1)*180))
                
                if(int((wx + 1)*180) - 1 < 360 and int((wy + 1)*180) -1 < 360):
                
                    if( map[int((wx + 1)*180) - 1 ][int((wy + 1)*180) - 1] < 1):
                        
                        map[int((wx + 1)*180) - 1 ][int((wy + 1)*180) - 1] = (map[int((wx + 1)*180) - 1 ][int((wy + 1)*180) - 1] + .01)
                        #print( map[int((wx + 1)*180) - 1 ][int((wy + 1)*180) - 1])    
                    
                    else:
                        map[int((wx + 1)*180) - 1 ][int((wy + 1)*180) - 1] = 1
                        
                    if(map[int((wx + 1)*180) - 1 ][int((wy + 1)*180) - 1] > .1):
                       
                        map[int(wx*30)-1][int(wy*30)-1] = 1
                    
                        display.setColor( int((((map[int((wx + 1)*180) - 1 ][int((wy + 1)*180) - 1]*256)**2) + (map[int((wx + 1)*180) - 1 ][int((wy + 1)*180) - 1]*256) + map[int((wx + 1)*180) - 1 ][int((wy + 1)*180) - 1])*255))
                        
                        display.drawPixel(int((wx + 1)*180),int((wy + 1)*180))
    
        display.setColor(0XFF0000)
        display.drawPixel(int((pose_x + 1)*180),int((pose_y + 1)*180))
                        
                        
        #display.drawPixel(int(pose_x*30), 360-int(pose_y*30))
        
        
       
        # Enter here functions to send actuator commands, like:
        # motor.setPosition(10.0)
        
    
        if(lidar_sensor_readings[10] <= .3):
            print("Left Sensor: ", lidar_sensor_readings[0])
            print("Right Sensor: ", lidar_sensor_readings[20])
            print(count)
        
            if(lidar_sensor_readings[0] > lidar_sensor_readings[20] and lidar_sensor_readings[0] - lidar_sensor_readings[20] >= .05 and count == 0):
                vL = (-MAX_SPEED/4)
                vR = (MAX_SPEED/4)
                print("left")
            elif(lidar_sensor_readings[0] < lidar_sensor_readings[20] and lidar_sensor_readings[20] - lidar_sensor_readings[0] >= .05 and count == 0):
                vL = (MAX_SPEED/4)
                vR = (-MAX_SPEED/4)
                print("right") 
            else:
                if(count == 0):
                    x = random.randint(0,1)
                
                if(x % 2 == 0):
                    vL = (MAX_SPEED/4)
                    vR = (-MAX_SPEED/4)
                    print("right") 
                else:
                    vL = (-MAX_SPEED/4)
                    vR = (MAX_SPEED/4)
                    print("left")
                
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
            
    elif(state == "Path_finder"):
        
        start_w = (pose_x,pose_y)
        end_w = (.57,.7)
        
        start = (int((start_w[0] + 1)*180) -1,int((start_w[1] + 1)*180) -1)
        end = (int((end_w[0] + 1)*180) - 1,int((end_w[1] + 1)*180) - 1)
        
        map[map>.1] = 1
        map[map<=.1] = 0
    
        filter = np.ones((10,10))
        cmap = convolve2d(map,filter,mode = "same")
        cmap[cmap>=1] = 1
        
        path = path_planner(cmap, start, end)
    
        goalPoints = []
        for i in range(len(path)):
            x = float(path[i][0])
            y = float(path[i][1])
            #print(x)
            #print(y)
            goalPoint = (((x+1)/180)-1,((y+1)/180)-1)
            goalPoints.append(goalPoint)
            final = len(goalPoints)
    
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
    
    
    
    
    #####################################################
    #                    Odometry                       #
    #####################################################
    
    EPUCK_MAX_WHEEL_SPEED = 0.11695*SIM_TIMESTEP/1000.0 
    dsr=vR/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED
    dsl=vL/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED
    ds=(dsr+dsl)/2.0
    
    pose_y += ds*math.cos(pose_theta)
    pose_x += ds*math.sin(pose_theta)
    pose_theta += (dsr-dsl)/EPUCK_AXLE_DIAMETER

# Enter here exit cleanup code.
