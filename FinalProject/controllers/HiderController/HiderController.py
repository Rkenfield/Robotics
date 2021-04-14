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


for i in range (21):
    if i == 10:
       lOffsets.append(0)
    else:
       lOffsets.append((count + i) * lAngle)
        
obs_dist = .2

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
    
                display.setColor(0X0000FF)
                #display.drawPixel(360-int(wy*30),int(wx*30))
                display.drawPixel(int((wx + 1)*180),int((wy + 1)*180))
                
                #print(rho)
                
                # display.setColor(0XFFFFFF)
                # display.drawLine(int((pose_x + 1)*180),int((pose_y + 1)*180),int((wx + 1)*180),int((wy + 1)*180))  
            
            
        display.setColor(0XFF0000)
        display.drawPixel(int((pose_x + 1)*180),int((pose_y + 1)*180))
        #display.drawPixel(int(pose_x*30), 360-int(pose_y*30))
        
        
       
    
    # Enter here functions to send actuator commands, like:
    # motor.setPosition(10.0)
    if(lidar_sensor_readings[10] <= obs_dist):
        x = random.randint(0,1)
        if(x % 2 == 0):
            vL = (MAX_SPEED/4)
            vR = (-MAX_SPEED/4)
            print("right") 
        else:
            vL = (-MAX_SPEED/4)
            vR = (MAX_SPEED/4)
            print("left")
            
        
    
    elif(lidar_sensor_readings[5] <= obs_dist  or lidar_sensor_readings[6] <= obs_dist or lidar_sensor_readings[7] <= obs_dist or lidar_sensor_readings[8] <= obs_dist  or lidar_sensor_readings[9] <= obs_dist):
       
        vL = (MAX_SPEED/4)
        vR = (-MAX_SPEED/4)
        print("right") 
            
    elif(lidar_sensor_readings[11] <= obs_dist  or lidar_sensor_readings[12] <= obs_dist or lidar_sensor_readings[13] <= obs_dist  or lidar_sensor_readings[14] <= obs_dist  or lidar_sensor_readings[15] <= obs_dist):
        vL = (-MAX_SPEED/4)
        vR = (MAX_SPEED/4)
        print("left")
    else:
        vL = MAX_SPEED/2
        vR = MAX_SPEED/2
    # vL = MAX_SPEED/2
    # vR = MAX_SPEED/2
    
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
