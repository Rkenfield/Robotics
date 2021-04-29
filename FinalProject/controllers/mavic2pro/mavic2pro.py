"""mavic2pro controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import *
import csv
import numpy

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

def CLAMP(value, low, high):
    return max(min(value, high), low)
    
def SIGN(x):
    return numpy.sign(x)

        
TAKEOFF_THRESHOLD_VELOCITY = 163

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Get propeller motors and set to velocity mode
frontLeftMotor = robot.getDevice('front left propeller')
frontRightMotor = robot.getDevice('front right propeller')
backLeftMotor = robot.getDevice('rear left propeller')
backRightMotor = robot.getDevice('rear right propeller')

frontLeftMotor.setPosition(float('inf'))
frontRightMotor.setPosition(float('inf'))
backLeftMotor.setPosition(float('inf'))
backRightMotor.setPosition(float('inf'))

frontLeftMotor.setVelocity(1.0)
frontRightMotor.setVelocity(1.0)
backLeftMotor.setVelocity(1.0)
backRightMotor.setVelocity(1.0)



# Get and enable devices:
camera = Camera("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)
gps = GPS("gps")
gps.enable(timestep)
imu = InertialUnit("inertial unit")
imu.enable(timestep)
compass = Compass("compass")
compass.enable(timestep)
gyro = Gyro("gyro")
gyro.enable(timestep)
camera_roll_motor = robot.getDevice("camera roll")
camera_pitch_motor = robot.getDevice("camera pitch")
keyboard = Keyboard()
keyboard.enable(timestep)

print("Starting drone...")

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if (robot.getTime() > 1.0):
        break
        
 #Display control menu
print("Use these keys to control the drone: ")
print("- 'up': move forward.")
print("- 'down': move backward.")
print("- 'right': turn right.")
print("- 'left': turn left.")
print("- 'shift + up': increase the target altitude.")
print("- 'shift + down': decrease the target altitude.")
print("- 'shift + right': strafe right.")
print("- 'shift + left': strafe left.")
        
k_vertical_thrust = 68.5 
k_vertical_offset = 0.6
k_vertical_p = 3.0
k_roll_p = 50.0
k_pitch_p = 30.0

target_altitude = 1.0 # can be changed based off needs

while robot.step(timestep) != -1:
    time = robot.getTime()
    
    # Retrieve robot position using sensors
    roll = InertialUnit.getRollPitchYaw(imu)[0] + numpy.pi/2.0
    pitch = InertialUnit.getRollPitchYaw(imu)[1]
    altitude = GPS.getValues(gps)[1]
    roll_acceleration = Gyro.getValues(gyro)[0]
    pitch_acceleration = Gyro.getValues(gyro)[1]
    
    Motor.setPosition(camera_roll_motor, -0.115 * roll_acceleration)
    Motor.setPosition(camera_pitch_motor, -0.1 * pitch_acceleration)
    
    roll_disturbance = 0.0
    pitch_disturbance = 0.0
    yaw_disturbance = 0.0
    key = Keyboard.getKey(keyboard)
    while (key > 0):
        if key == keyboard.UP:
            pitch_disturbance = 2.0

        elif key == keyboard.DOWN:
            pitch_disturbance = -2.0
 
        elif key == keyboard.RIGHT:
            yaw_disturbance = 1.3
      
        elif key == keyboard.LEFT:
            yaw_disturbance = -1.3
     
        elif keyboard.SHIFT + keyboard.RIGHT:
            roll_disturbance = -1.0
          
        elif keyboard.SHIFT + keyboard.LEFT:
            roll_disturbance = 1.0
         
        elif keyboard.SHIFT + keyboard.UP:
            target_altitude += 0.05
            print("target altitude: ", target_altitude)

        elif keyboard.SHIFT + keyboard.DOWN:
            target_altitude -= 0.05
            print("target altitude: ", target_altitude)
  
        key = Keyboard.getKey(keyboard)
    
    #Computing roll, pitch, yaw, and vertical inputs
    roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration
    pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) - pitch_acceleration
    yaw_input = yaw_disturbance
    clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
    vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0)
    
    #Actuate motors, taking into consideration all computed inputs ^^
    front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
    frontLeftMotor.setVelocity(front_left_motor_input);
    frontRightMotor.setVelocity(-front_right_motor_input);
    backLeftMotor.setVelocity(-rear_left_motor_input);
    backRightMotor.setVelocity(rear_right_motor_input);
    


# Enter here exit cleanup code.


#Recognition node: segmentation = True -> gives a segmentation ground
#truth image generated based on SolidrecognitionColors field value.