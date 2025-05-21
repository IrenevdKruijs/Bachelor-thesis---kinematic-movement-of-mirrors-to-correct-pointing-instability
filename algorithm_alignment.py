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
    with open("target_pixels_cam1.txt") as f:
        x_target, y_target = map(int, f.read().strip().split(","))
except FileNotFoundError:
    print("Align setup and run find_middle_after_alignment")
    exit()

margin = 5
steprate = 250
max_attempts = 5

# Setup camera and motor
cam = camera_controller(exposuretime=1000)  # Adjust exposuretime as needed
# cam2 = camera_controller(exposuretime=1000)
motor = PiezoMotor(serial_number="97251304",)

# Ensure flip mirror is in upright position
flipmirror(1)

# Check if laser is already aligned on both cameras
image = cam.capture_image()
current_x, current_y = localize_beam_center(image)
print(current_x,current_y)
if (x_target - margin <= current_x <= x_target + margin and 
    y_target - margin <= current_y <= y_target + margin):
    print("Laser already correctly aligned")
    flipmirror(2)
    motor.shutdown()
    exit()
    
# implement this when using second camera
# image2 = cam2.capture_image()
# current_x2,current_y2 = localize_beam_center(image2)
# if (x_target - margin <= current_x <= x_target + margin and 
#     y_target - margin <= current_y <= y_target + margin):
#     print("Laser already correctly aligned")
#     flipmirror(2)
#     motor.shutdown()
#     exit()

# Get calibration data
calibration_data = get_cached_calibration(5, stepsize=100, repeats=2,steprate=500,motor=motor)

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
    print("Warning: Calibration fit quality is low (R² < 0.8). Recalibrating...")
    calibration_data = calibrate_mirror1_2D(5, stepsize=100, repeats=2, motor=motor)
    # Recompute slopes
    all_steps, all_delta_x, all_delta_y = [], [], []
    for run in calibration_data:
        steps = [item[0] for item in run]
        delta_x = [item[1] for item in run]
        delta_y = [item[2] for item in run]
        all_steps.extend(steps)
        all_delta_x.extend(delta_x)
        all_delta_y.extend(delta_y)
    slope_x, intercept_x, r_value_x, *_ = linregress(all_steps, all_delta_x)
    slope_y, intercept_y, r_value_y, *_ = linregress(all_steps, all_delta_y)
    print(f"New calibration slopes: slope_x = {slope_x:.4f} pixels/step (R² = {r_value_x**2:.4f}), "
          f"slope_y = {slope_y:.4f} pixels/step (R² = {r_value_y**2:.4f})")

# Alignment loop
attempt = 0
while attempt < max_attempts:
    steprate = 500
    image = cam.capture_image()
    middle_x, middle_y = localize_beam_center(image)
    if middle_x is None or middle_y is None:
        print("Error: Could not determine beam center.")
        break

    dx_pixel = x_target - middle_x
    dy_pixel = y_target - middle_y

    # Check if both axes are within margin
    if abs(dx_pixel) <= margin and abs(dy_pixel) <= margin:
        print(f"Success! Target reached within margin. final deviation: dx = {dx_pixel}, dy = {dy_pixel}")
        flipmirror(2)
        break

    # Only move axes that are outside the margin
    dx_motor = int(dx_pixel / slope_x) if slope_x != 0 and abs(dx_pixel) > margin else 0
    dy_motor = int(dy_pixel / slope_y) if slope_y != 0 and abs(dy_pixel) > margin else 0

    if dx_motor == 0 and dy_motor == 0:
        print("No movement needed; both axes within margin or no valid correction.")
        break

    motor.correct_backlash(dx_motor,dy_motor,0,0,steprate)
    print("correcting backlash")
    img=cam.capture_image()
    middle_x,middle_y = localize_beam_center(img)

    dx_pixel = x_target - middle_x
    dy_pixel = y_target - middle_y

    print(f"Attempt {attempt + 1}: dx = {dx_pixel}, dy = {dy_pixel} (pixels)")

    # Only move axes that are outside the margin
    dx_motor = int(dx_pixel / slope_x) if slope_x != 0 and abs(dx_pixel) > margin else 0
    dy_motor = int(dy_pixel / slope_y) if slope_y != 0 and abs(dy_pixel) > margin else 0

    if dx_motor == 0 and dy_motor == 0:
        print("No movement needed; both axes within margin or no valid correction.")
        break
    
    motor.move_steps(dx_motor, dy_motor, 0, 0,steprate)
    img = cam.capture_image()
    middle_x,middle_y = localize_beam_center(img)
    if middle_x is None or middle_y is None:
        print("Error: Failed to capture beam position after movement.")
        break

    attempt += 1
    

else:
    print("Failed to reach target within margin after maximum attempts.")

flipmirror(2)
motor.shutdown()