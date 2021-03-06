"""lab5 controller."""
# -*- coding: utf-8 -*-
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5

# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
# camera.enable(timestep)
# camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")

# Odometry
#pose_x     = 2.58
#pose_y     = 8.9
#pose_theta = 0

#based on the given maps in the lab description
pose_x     = 4.49
pose_y     = 4.8
pose_theta = 0

#pose_x = 4.266
#pose_y = 8.033
    
    

vL = 0
vR = 0

#variable initialization for controller
Dist_Error = 0
Bearing_Error = 0
gain = .2
xR = 0
thetaR = 0
final = 0

##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
#mode = 'manual' # Part 1.1: manual mode
mode = 'planner'
#mode = 'autonomous'

lidar_sensor_readings = []
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # remove blocked sensor rays

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

###################
#
# Planner
#
###################
if mode == 'planner':
# Part 2.3: Provide start and end in world coordinate frame and convert it to map's frame
    
    start_w = (pose_x,pose_y) # (Pose_X, Pose_Z) in meters
    #start_w = (math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x,-(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y)
    
    end_w = (10,7) # (Pose_X, Pose_Z) in meters
    #end_w = (7.6,2.5) 
    
    
    
    
    # Convert the start_w and end_W from webot's coordinate frame to map's
    start = (int(start_w[0]*30)-1,(int(start_w[1]*30)-1)) # (x, y) in 360x360 map
    end = (int(end_w[0]*30)-1,(int(end_w[1]*30)-1)) # (x, y) in 360x360 map
   
    
    
  
    
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
        
# Part 2.3: Implement A* or Dijkstra's
    def path_planner(map, start, end):
        # print(map[start[1],start[0]])
        #:param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
        #:param start: A tuple of indices representing the start cell in the map
        #:param end: A tuple of indices representing the end cell in the map
        #:return: A list of tuples as a path from the given start to the given end in the given maze
        if map[start[0]][start[1]] == 1 or map[end[0]][end[1]] == 1:
            
            return []
        
       
        startN = pathNode(None, start)
        endN = pathNode(None, end)
        
        openL = []
        closedL = []
        
        openL.append(startN)
        # count = 0
        # times = 0
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
    
             
# Part 2.1: Load map (map.npy) from disk and visualize it
    map = np.load("map.npy")
    
 
    map[map>.7] = 1
    map[map<=.7] = 0
    
 
# Part 2.2: Compute an approximation of the "configuration space"
   
    
    
    filter = np.ones((16,16))
    cmap = convolve2d(map,filter,mode = "same")
    cmap[cmap>=1] = 1
     
    
    # plt.imshow(cmap)
    
    # plt.show()

# Part 2.3 continuation: Call path_planner
    path = path_planner(cmap,start,end)
   
# Part 2.4: Turn paths into goal points and save on disk as path.npy and visualize it
    
    goalPoints = []
    for i in range(len(path)):
        x = float(path[i][0])
        y = float(path[i][1])
        #print(x)
        #print(y)
        goalPoint = ((x/30),(y/30))
        goalPoints.append(goalPoint)
        final = len(goalPoints)

    np.save("path.npy", goalPoints)
    #np.save("goalPath.npy",goalPoints)


# Part 1.2: Map Initialization
map = np.zeros((360,360))

# Initialize your map data structure here as a 2D floating point array
if mode == 'manual':
    map = np.zeros((360,360)) # Replace None by a numpy 2D floating point array


if mode == 'autonomous':
# Part 3.1: Load path from disk and visualize it (Make sure its properly indented)
    #path = np.load("goalPath.npy")

    path = np.load("path.npy")
    
    
    map = np.load("map.npy")
    
 
    map[map>.7] = 1
    map[map<=.7] = 0
    
 
# Part 2.2: Compute an approximation of the "configuration space"
    
    # filter = np.ones((16,16))
    # cmap = convolve2d(map,filter,mode = "same")
    # cmap[cmap>=1] = 1
    # #plt.scatter(path[:,1],path[:,0])   
    # plt.imshow(cmap)
    # plt.show()
    
   
    #plt.show()
    
    
    for i in range(len(path)):
    
        display.setColor(int(0xFFFFFF))
        display.drawPixel(360-int(path[i][1]*30),int(path[i][0]*30))
 
    state = 0 # use this to iterate through your path

while robot.step(timestep) != -1 and mode != 'planner':

###################
#
# Sensing
#
###################
    # Ground truth pose
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            rho = LIDAR_SENSOR_MAX_RANGE

        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y

        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))

        if rho < 0.5*LIDAR_SENSOR_MAX_RANGE:
# Part 1.3: visualize map gray values.

            # You will eventually REPLACE the following 2 lines with a more robust version of map
            # and gray drawing that has more levels than just 0 and 1.
            
            #display.setColor(0xFFFFFF)
            #display.drawPixel(360-int(wy*30),int(wx*30))
            
            if(int(wx*30)-1 < 360 and int(wy*30)-1 < 360):
                
            
                if(map[int(wx*30)-1][int(wy*30)-1] < 1):
            
                    map[int(wx*30)-1][int(wy*30)-1] =  (map[int(wx*30)-1][int(wy*30)-1] + 0.01)
                      
                elif (map[int(wx*30)-1][int(wy*30)-1] > 1):
                    
                    map[int(wx*30)-1][int(wy*30)-1] = 1
            
                if(map[int(wx*30)-1][int(wy*30)-1] >.7):
                   
                    
                    map[int(wx*30)-1][int(wy*30)-1] = 1
                    
                    display.setColor( int((((map[int(wx*30)-1][int(wy*30)-1]*256)**2) + (map[int(wx*30)-1][int(wy*30)-1]*256) + map[int(wx*30)-1][int(wy*30)-1])*255))
                   
                    display.drawPixel(360-int(wy*30),int(wx*30))
                    
              
                    
            display.setColor(int(0xFF0000))
            display.drawPixel(360-int(pose_y*30),int(pose_x*30))
            
    

###################
#
# Controller
#
###################
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
# Part 1.4: Save map to disc
            np.save('map.npy', map)
            print("Map file saved")
        elif key == ord('L'):
            # You will not use this portion but here's an example for loading saved a numpy array
            map = np.load("map.npy")
            print("Map loaded")
        else: # slow down
            vL *= 0.75
            vR *= 0.75
    else: # not manual mode
       
# Part 3.2: Feedback controller
        if(state < len(path)):
            #STEP 1: Calculate the error    
        
            Dist_Error = math.sqrt(((pose_x - path[state][0])**2) + ((pose_y - (path[state][1]))**2))
       
            Bearing_Error = math.atan2((path[state][1] - pose_y) , (path[state][0] - pose_x)) + pose_theta
        
    
            #STEP 2: Controller
            xR = Dist_Error 
            thetaR = Bearing_Error 
           
            
            
            #STEP 3: Compute wheelspeeds
            if(Dist_Error > gain):
        
                if(Bearing_Error < -gain):
                    
                
                    if(abs(thetaR * MAX_SPEED * xR) > MAX_SPEED):
                        vR = MAX_SPEED
                    else:
                        vR = abs(thetaR * MAX_SPEED * xR)
                    vL = -vR
                      
                elif(Bearing_Error > gain):
                    
            
                    if(abs(thetaR * MAX_SPEED * xR) > MAX_SPEED):
                        vL = MAX_SPEED
                    else:
                        vL = abs(thetaR * MAX_SPEED * xR)
                    
                    vR = -vL
                
                
                else:    
                    if(xR*MAX_SPEED > MAX_SPEED):
                        vL = MAX_SPEED
                        vR = MAX_SPEED
                    else:
                        vL = xR*MAX_SPEED
                        vR = xR*MAX_SPEED
            else:
                vL = 0
                vR = 0
        
                state+=1
            
        else:
           vL = 0
           vR = 0  
    
        # Normalize wheelspeed
        # Keep the max speed a bit less to minimize the jerk in motion
        if(vL >= MAX_SPEED):
            vL = MAX_SPEED/2
            vR = vR/2
        if(vR >= MAX_SPEED):
            vR = MAX_SPEED/2
            vL = vL/2
            
        elif( vL == vR):
            vL *= 0.75
            vR *= 0.75
    


    # Odometry code. Don't change speeds after this
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    #print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta)) #/3.1415*180))

    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)