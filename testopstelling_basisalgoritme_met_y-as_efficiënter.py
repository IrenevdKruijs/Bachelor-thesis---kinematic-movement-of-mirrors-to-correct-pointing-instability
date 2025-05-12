"""
TO DO: 
- moet marge eruit? hoe groot moet marge zijn?
- oplossen dat calibration nu iedere keer opnieuw moet worden gedaan -> kun je dit eens in de zoveel tijd doen en dan ergens opslaan?

"""
import numpy as np
import time
import clr
from pypylon import pylon
from scipy.stats import linregress
from functions import *
##Piezo inladen
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.InertialMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *

## camera, info van:https://pythonforthelab.com/blog/getting-started-with-basler-cameras/ 
# camera verbinden
cam = camera_setup()
im = image(cam)
middle_x,middle_y = coordinates(im,0,0,0,0)
print(middle_x)

## loop for finding back the middle pixel
#initializing loop
x_target = 1035 #set the target pixel the middle of the laser is after perfect alignment
y_target = 561
margin = 100 #set the margin of the system
maximum_tries=0

## Run calibration to obtain slopes
calibration_data = calibrate_mirror1_2D(4, 100, 1, "up", 200,cam)

## Extract slopes from calibration data
# Dit is beetje dubbelop met het plot_calibration script maar weet even niet hoe ik dat nu beter kan maken
all_steps, all_delta_x, all_delta_y = [], [], []
for run in calibration_data:
    steps, delta_x, delta_y = zip(*run)
    all_steps.extend(steps)
    all_delta_x.extend(delta_x)
    all_delta_y.extend(delta_y)

# Compute linear regression to get slopes
slope_x, intercept_x, r_value_x, *_ = linregress(all_steps, all_delta_x)
slope_y, intercept_y, r_value_y, *_ = linregress(all_steps, all_delta_y)
print(f"Calibration slopes: slope_x = {slope_x:.4f} pixels/step (R² = {r_value_x**2:.4f}), slope_y = {slope_y:.4f} pixels/step (R² = {r_value_y**2:.4f})")

## Check calibration quality
if r_value_x**2 < 0.8 or r_value_y**2 < 0.8:
    print("Warning: Calibration fit quality is low (R² < 0.8). Results may be unreliable.")

## Direct movement to target
if middle_x is None or middle_y is None:
    print("Error: middle_x or middle_y is None (invalid image or coordinates)")
else:
    # Calculate pixel differences
    delta_x = x_target - middle_x  # required pixel shift in x
    delta_y = y_target - middle_y  # required pixel shift in y

    # Calculate required motor steps using calibration slopes
    dx = int(delta_x / slope_x) if slope_x != 0 else 0  # motor steps for x
    dy = int(delta_y / slope_y) if slope_y != 0 else 0  # motor steps for y

    print(f"Required movement: dx = {dx} steps, dy = {dy} steps")

    # Move the piezo motor
    if dx != 0 or dy != 0:
        middle_x, middle_y = piezomotor(dx, dy, 0, 0, 100,cam)  # Use same steprate as calibration
        time.sleep(0.1)  # Small delay to ensure movement completes
    else:
        print("No movement required: already at target.")

    # Verify final position
    print(f"Final coordinates: middle_x = {middle_x}, middle_y = {middle_y}")
    if abs(middle_x - x_target) <= margin and abs(middle_y - y_target) <= margin:
        print("Success! Target reached within margin.")
    else:
        print(f"Failed! Target not reached within {margin} pixels.")
        print(f"Final deviations: x = {abs(middle_x - x_target)}, y = {abs(middle_y - y_target)}")