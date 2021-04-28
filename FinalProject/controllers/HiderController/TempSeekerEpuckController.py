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

#Robot starts in the middle of a 2 by 2 map
pose_x = 1
pose_y = 1
pose_theta = 0

#Controls the state machine for the epuck
state = "Seek"




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
    
    #mapping mode for the epuck 
    if(state == "seek"):
        #Timer for how long the explore function of the robot works for
        
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
        
            #print(timer)
        #When the time runs out for the epuck discovery mode then the epuck switches to using the map it has created to drive through the level rather than obstacle sensors  
       
        
        
         
    else:
        vL = 0
        vR = 0
        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)   
    
    
    



# Enter here exit cleanup code.
