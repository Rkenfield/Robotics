"""Lab2_StateController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot,DistanceSensor,Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 64

MAX_SPEED = 6.28
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
ps = []
psNames = [ 'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7']

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)
    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
count = 0
state = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    forward_obstacle = psValues[6] > 80 or psValues[7] > 80 or psValues[0] > 80 or psValues[1] > 80
    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    if(state == 0 ):
         leftSpeed = 1 * MAX_SPEED
         rightSpeed = 1 * MAX_SPEED
       
    
    if(forward_obstacle == True and state == 0 or state == 1):
        #Handles the turn 180 state
        state = 1
        if(count <= 24):
            leftSpeed = 0.443 * MAX_SPEED
            rightSpeed = -0.443 * MAX_SPEED
        
            count+=1
   
        else:
            leftSpeed = 0
            rightSpeed = 0
            state = 2
            count = 0
     
    if(state == 2):
        leftSpeed = 1 * MAX_SPEED
        rightSpeed = 1 * MAX_SPEED
        if(forward_obstacle):
            #rotate clockwise until sensor ps5 reads > than 80
            #when the left sensor reads > 80 set state to 3
            #if(forward_obstacle < 80):
                #wb_motor_set_velocity(left_motor, MAX_SPEED)
                #wb_motor_set_velocity(right_motor, -MAX_SPEED)
            leftSpeed = 1 * MAX_SPEED
            rightSpeed = -1 * MAX_SPEED
            
                
            #when the left sensor reads > 80 set state to 3
            if (psValues[5] > 60):
                leftSpeed = 0
                rightSpeed = 0
                state = 3
                
        
    if(state == 3):
        #implement drive until the left sensor no longer reads > 80 then set state to state 4        
        if(psValues[5] > 60):
            leftSpeed = 1 * MAX_SPEED
            rightSpeed = 1 * MAX_SPEED
        else: 
            state = 4
             
            
    if(state == 4):
        leftSpeed = 0
        rightSpeed = 0
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
   
    pass

# Enter here exit cleanup code.
