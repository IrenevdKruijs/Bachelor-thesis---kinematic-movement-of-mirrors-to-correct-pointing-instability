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
    with open("alignment_result.txt") as f:
        x_target, y_target = map(int, f.read().strip().split(","))
except FileNotFoundError:
    print("Align setup and run find_middle_after_alignment")
    
margin = 10
steprate = 250
max_attempts = 5

# Setup camera
cam = camera_controller
im = cam.capture_image
#make sure the flip mirror is in upright position
flipmirror(1)

# make sure the script only runs if the position of the laser is incorrect
current_x,current_y = localize_beam_center(im)
if x_target - margin <= current_x <= x_target + margin and y_target - margin <= current_y <= y_target + margin:
    print("laser already correctly aligned")
    flipmirror(2)
    exit()
#if there is no calibration data yet with the stepsize, the calibration is done. Else, a saved calibration will be used
calibration_data = get_cached_calibration(5, stepsize=100, repeats=2, steprate=steprate)

# Extract calibration data
all_steps, all_delta_x, all_delta_y = [], [], []
for run in calibration_data:
    steps = [item[0] for item in run]
    delta_x = [item[1] for item in run]
    delta_y = [item[2] for item in run]
    all_steps.extend(steps)
    all_delta_x.extend(delta_x)
    all_delta_y.extend(delta_y)

# Compute slopes
slope_x, intercept_x, r_value_x, *_ = linregress(all_steps, all_delta_x)
slope_y, intercept_y, r_value_y, *_ = linregress(all_steps, all_delta_y)

print(f"Calibration slopes: slope_x = {slope_x:.4f} pixels/step (R² = {r_value_x**2:.4f}), "
      f"slope_y = {slope_y:.4f} pixels/step (R² = {r_value_y**2:.4f})")

if r_value_x**2 < 0.8 or r_value_y**2 < 0.8:
    print("Warning: Calibration fit quality is low (R² < 0.8). Results may be unreliable.")

# Alignment loop
attempt = 0
while attempt < max_attempts:
    im = cam.capture_image()
    middle_x, middle_y = localize_beam_center(im)
    if middle_x is None or middle_y is None:
        print("Error: Could not determine beam center.")
        break

    dx_pixel = x_target - middle_x
    dy_pixel = y_target - middle_y

    print(f"Attempt {attempt + 1}: dx = {dx_pixel}, dy = {dy_pixel} (pixels)")

    if abs(dx_pixel) <= margin and abs(dy_pixel) <= margin:
        print("Success! Target reached within margin.")
        flipmirror(2)
        break

    dx_motor = int(dx_pixel / slope_x) if slope_x != 0 else 0
    dy_motor = int(dy_pixel / slope_y) if slope_y != 0 else 0

    print(f"Moving motor: dx = {dx_motor} steps, dy = {dy_motor} steps")
    piezomotor(dx_motor, dy_motor, 0, 0, steprate)
    time.sleep(0.1)
    attempt += 1

else:
    print("Failed to reach target within margin after maximum attempts.")
    
