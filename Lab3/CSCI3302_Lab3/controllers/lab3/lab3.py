"""lab3 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar
import math

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.5725 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10 # Left wheel index
MOTOR_RIGHT = 11 # Right wheel index
N_PARTS = 12 # Total joints

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
# Note that positions for the joints are in radians.
target_pos = (0.0, 0.0, 0.09, 0.07, 0.26, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts = []




for i in range(N_PARTS):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))
        robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

##### Note that the following sensors will not be used for Lab 3.######

# We want to just show that these rich sensors are available with Tiago 
# some of which will be used later.Feel free to play with them. 
# The Tiago robot has a couple more sensors than the e-Puck.
# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
#camera.enable(timestep)
#camera.recognitionEnable(timestep)
# lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
#lidar.enable(timestep)
#######################################################################

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

gain = .25
bearingGain = 0

goal_pos = [[1.8,-1],[1.2,-2],[1,-2.5],[-.6,-2.6],[-.9,-3.5]]

current = 0
final = len(goal_pos)


while robot.step(timestep) != -1:

    ## webots offer builtin object detectors which will not be used in Lab 3 ###
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Webots simulates the equivalent of a Mask R-CNN deep learning-based object
    # recognition framework. We print out the names of the objects Tiago sees. You
    # can plan with this data structure eventually. 
    #objects = camera.getRecognitionObjects()
    #for i in objects:
    #    print(i.get_model()) 
    ##########################################################################
    
    if(current  < final):
    
    #STEP 1: Calculate the error
    
        Dist_Error = math.sqrt(((pose_x - goal_pos[current][0])**2) + ((pose_y - (goal_pos[current][1]))**2))
   
        Bearing_Error = math.atan2((goal_pos[current][1] - pose_y) , (goal_pos[current][0] - pose_x)) - pose_theta 
    
   
    #print(Bearing_Error) 
    
    #print(Dist_Error)
    #STEP 2: Controller (with gains)
        xR = Dist_Error * gain 
        thetaR = (Bearing_Error * gain) 
  
  
    #print(thetaR)  
    #STEP 3: Compute wheelspeeds
    
        if(Dist_Error > gain):
            if(Bearing_Error < -.1):
                vL = 0
            
            
                if(thetaR * MAX_SPEED > MAX_SPEED):
                    vR = MAX_SPEED
                else:
                    vR = abs(thetaR * MAX_SPEED)
                
                
                  
            elif(Bearing_Error > .1):
                vR = 0
        
                if(thetaR * MAX_SPEED > MAX_SPEED):
                    vL = MAX_SPEED
                else:
                    vL = abs(thetaR * MAX_SPEED)
                
            
            
            
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
        
            current+=1
        
    else:
       vL = 0
       vR = 0  
   

    #STEP 4: Normalize wheelspeed
   
    
    ################# Do not modify this block of the code ########################
    # Odometry code. Don't change speeds after this
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vL-vR)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0
    print("X: %f Y: %f Theta: %f" % (pose_x, pose_y, pose_theta)) # Comment out this if you want
    ##############################################################################

    # Enter here functions to send actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)  
    
   
        
        
    