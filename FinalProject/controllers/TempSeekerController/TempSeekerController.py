import math
import time
import random
import copy
import numpy as np
import struct
from controller import Robot, Motor, DistanceSensor, Emitter

LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_BINS = 21 # 21 Bins to cover the angular range of the lidar, centered at 10
LIDAR_ANGLE_RANGE = 1.5708 # 90 degrees, 1.5708 radians

#Robot starts in the middle of a 2 by 2 map
pose_x = 1
pose_y = 1
pose_theta = 0

#Controls the state machine for the epuck
state = "seek"


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


emitter = robot.getDevice('emitter1')
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
        




#variables for looping through different behaviors 
obs_dist = .1
count = 0
#set map of 360 by 360 to all 0's

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(SIM_TIMESTEP) != -1:
    # Read the sensors:
   
    emitter.send(str.encode("close"))
    lidar_sensor_readings = lidar.getRangeImage()
    #draws where epuck is in the display
   
    #use gps to track position of epuck
   
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
                    if(count >= 500):
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
                    vL = MAX_SPEED/4
                    vR = MAX_SPEED/4   
        
            leftMotor.setVelocity(vL)
            rightMotor.setVelocity(vR)    
            
           
        
        
         
    else:
        vL = 0
        vR = 0
        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)   
    
    
    



# Enter here exit cleanup code.
