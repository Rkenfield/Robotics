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
pose_theta = 0


state = "caught"


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
    display.drawPixel(int(pose_y*180),360-int(pose_x*180))
    
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]
    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    # Process sensor data here.
    
    # print("Pose X: ", pose_x)
    # print("Pose Y: ",pose_y)
    
    if(state == "basic_mode"):
    
        if(timer < 15000):  
              
            for i, rho in enumerate(lidar_sensor_readings):
                alpha = -lOffsets[i]
    
                if rho > LIDAR_SENSOR_MAX_RANGE:
                    rho = LIDAR_SENSOR_MAX_RANGE
    
              
                            
                xr = math.cos(alpha)* rho
                yr = -math.sin(alpha)* rho
                
                        
                wx =  math.cos(pose_theta)*xr - math.sin(pose_theta)*yr + pose_x
                wy =  -(math.sin(pose_theta)*xr + math.cos(pose_theta)*yr) + pose_y
                
                # xr = math.sin(alpha)* rho
                # yr = math.cos(alpha)* rho        
                    
                # wx = (xr*math.cos(pose_theta)) + (yr*math.sin(pose_theta)) + pose_x
                # wy = (yr*math.cos(pose_theta)) - (xr*math.sin(pose_theta)) + pose_y
    

                print("WX: ", wx)
                print("WY: ", wy)
                
                if rho < 0.5*LIDAR_SENSOR_MAX_RANGE:
        
                    #display.setColor(0X0000FF)
                    #display.drawPixel(360-int(wy*30),int(wx*30))
                    #display.drawPixel(int((wx + 1)*180),int((wy + 1)*180))
                    
                    if(abs(int((wx + 1)*180) - 1) < 360 and abs(int((wy + 1)*180) -1) < 360):
                    
                        if( map[int(wx*180) - 1 ][(int(wy*180)) - 1] < 1):
                            
                            map[int(wx*180) - 1 ][(int(wy*180)) - 1] = map[int(wx*180) - 1 ][(int(wy*180)) - 1] + .01
                            #print( map[int((wx + 1)*180) - 1 ][int((wy + 1)*180) - 1])    
                        
                        else:
                            map[int(wx*180) - 1 ][(int(wy*180)) - 1] = 1
                            
                        if(map[int(wx*180) - 1 ][(int(wy*180)) - 1] > .1):
                           
                            map[int(wx*180) - 1 ][(int(wy*180)) - 1] = 1
                        
                        display.setColor( int((((map[int(wx*180) - 1 ][(int(wy*180)) - 1]*256)**2) + (map[int(wx*180) - 1 ][(int(wy*180)) - 1]*256) + map[int(wx*180) - 1 ][(int(wy*180)) - 1])*255))
                        
                        display.drawPixel(int(wy*180),360-int(wx*180))
              
                            
                            
            #display.drawPixel(int(pose_x*30), 360-int(pose_y*30))
            
            
           
            # Enter here functions to send actuator commands, like:
            # motor.setPosition(10.0)
            
        
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
            timer += 1
            #print(timer)
            
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
            
            start = ((int(start_w[0]*180)-1) ,(int((start_w[1]*180) -1)))
            end = (int(end_w[0]*180) - 1, int(end_w[1]*180) - 1)
        
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
                PathPoint = (((x+1)/180),((y+1)/180))
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
                display.setColor(0X45b6fe)
                display.drawPixel(int((i[1]*180)-1),int(360 - (i[0]*180)-1))
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
