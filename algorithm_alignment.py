"""
TO DO: 
- how large should margin be?
- Ensure that the camera is preferably no longer needed in image processing -> its very annoying because then it has to be included in every function
- Calibration of poor quality -> should we redo it next time then?
- The script must be executed when the wavelength changes and periodically.
- If one of the two (x or y) is already within the margin, it should not move anymore -> it often only makes things worse.
- Where in the script should I correct for the slack? The location must be measured to know which direction to move and thus how to 
    correct for slack, but afterward, the position must be measured again before moving.
    
This function measures the pointing instability of the laser beam on a camera and realigns it by tipping/tilting a motorized mirror.
"""
import time
import clr
import math
from scipy.stats import linregress
from functions import *

# Initialize camera and motor libraries
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.InertialMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *

# Constants
try:
    with open("target_pixels_cam1.txt") as f:
        x1_target, y1_target = map(int, f.read().strip().split(","))
    with open("target_pixels_cam2.txt") as f:
        x2_target, y2_target = map(int, f.read().strip().split(","))
except FileNotFoundError:
    print("Align setup and run find_middle_after_alignment")
    exit()

margin = 5
steprate = 500
max_attempts = 5

# Setup camera and motor
cam = camera_controller(exposuretime=1000)  # Adjust exposuretime as needed
# cam2 = camera_controller(exposuretime=1000)
motor = PiezoMotor(serial_number="97251304",)

# Ensure flip mirror is in upright position
flipmirror(1)

# Check if laser is already aligned on both cameras
image1 = cam.capture_image(1)
current_x, current_y = localize_beam_center(image1)
print(current_x,current_y)
   
# check current location of second camera
image2 = cam.capture_image(2)
current_x2,current_y2 = localize_beam_center(image2)

if (x1_target - margin <= current_x <= x1_target + margin and 
    y1_target - margin <= current_y <= y1_target + margin and x2_target - margin <= current_x2 <= x2_target + margin and 
    y2_target - margin <= current_y2 <= y2_target + margin)  :
    print("Laser already correctly aligned")
    flipmirror(2)
    motor.shutdown()
    exit()

#variables of length of the setup in mm
A = 1
B = 1
C = 1
pixelsize = 0.0055 #mm
stepsize = 0.005 #radians, stepsize of motorized mirrors

# Alignment loop
attempt = 0
while attempt < max_attempts:
    steprate = 500
    image1 = cam.capture_image(1)
    middle_x1, middle_y1 = localize_beam_center(image1)
    if middle_x1 is None or middle_y1 is None:
        print("Error: Could not determine beam center at camera 1.")
        break

    dx1_pixel = x1_target - middle_x1
    dy1_pixel = y1_target - middle_y1
    dx1_height = dx1_pixel*pixelsize
    dy1_height = dy1_pixel * pixelsize
    
    image2 = cam.capture_image(2)
    middle_x2,middle_y2 = localize_beam_center(image2)
    if middle_x2 is None or middle_y2 is None:
        print("Error: Could not determine beam center at camera 2.")
    
    dx2_pixel = x2_target-middle_x2
    dy2_pixel = y2_target-middle_y2
    dx2_height = dx2_pixel*pixelsize
    dy2_height = dy2_pixel*pixelsize
    
        # Check if both axes and both mirrors are within margin
    if abs(dx1_pixel) <= margin and abs(dy1_pixel) <= margin and abs(dx2_pixel)<= margin and abs(dy2_pixel)<= margin:
        print(f"Success! Target reached within margin. final deviation: dx1 = {dx1_pixel}, dy1 = {dy1_pixel}, dx2 = {dx2_pixel}, dy2 = {dy2_pixel}")
        flipmirror(2)
        break
    
    # theoretical calculation of movement of mirrors y-direction
    alpha_y = math.atan((dy2_height - dy1_height)/C) #angle in radians
    h_y = (-A+B+C)*math.tan(alpha_y)+dy2_height # height in mm
    MM1_y = alpha_y+math.tan(h_y/A) #movement in radians
    MM2_y = -math.tan(h_y/A)
    print(f"movement in radians needed for MM1: {MM1_y}. Movement in radians needed for MM2:{MM2_y} ")
    
    # theoretical calculation of movement of mirrors y-direction   
    alpha_x = math.atan((dx2_height - dx1_height)/C) #angle in radians
    h_x = (-A+B+C)*math.tan(alpha_x)+dx2_height # height in mm
    MM1_x = alpha_x+math.tan(h_x/A) #movement in radians
    MM2_x = -math.tan(h_x/A)
    print(f"movement in radians needed for MM1: {MM1_x}. Movement in radians needed for MM2:{MM2_x} ")
    

    # Only move axes that are outside the margin
    dy1_motor = int(MM1_y/stepsize) if abs(dx1_pixel) > margin else 0
    dy2_motor = int(MM2_y/stepsize) if abs(dy2_pixel) > margin else 0
    dx1_motor = int(MM1_x/stepsize) if abs(dx1_pixel) > margin else 0
    dx2_motor = int(MM2_x/stepsize) if abs(dx2_pixel) > margin else 0

    if dx1_motor == 0 and dy1_motor == 0 and dx2_motor == 0 and dy2_motor == 0:
        print("No movement needed; both axes within margin or no valid correction.")
        break
    
    motor.move_steps(dx1_motor, dy1_motor, dx2_motor, dy2_motor,steprate)
    attempt += 1
    

else:
    print("Failed to reach target within margin after maximum attempts.")

flipmirror(2)
motor.shutdown()