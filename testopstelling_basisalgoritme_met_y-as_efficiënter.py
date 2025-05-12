"""
TO DO: 
- hoe groot moet marge zijn?
- zorgen dat camera het liefst niet meer nodig is in image-> is heel irritant want dan moet het in iedere functie
- kalibratie van slechte kwaliteit-> volgende keer dan wel opnieuw doen?
- script moet worden uitgevoerd als golflengte verandert en eens in de zoveel tijd

"""
import time
import clr
from scipy.stats import linregress
from functions import *
from calibration_cache import get_cached_calibration

# Initialize camera and motor libraries
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.InertialMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *

# Constants
x_target = 1035
y_target = 561
margin = 10
steprate = 250
max_attempts = 5

# Setup camera
cam = camera_setup()

#if there is no calibration data yet with the stepsize, the calibration is done. Else, a saved calibration will be used
calibration_data = get_cached_calibration(5, stepsize=100, repeats=2, steprate=steprate, camera=cam)

# Extract calibration data
all_steps, all_delta_x, all_delta_y = [], [], []
for run in calibration_data:
    steps, delta_x, delta_y = zip(*run)
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
    im = image(cam)
    middle_x, middle_y = coordinates(im, 0, 0, 0, 0)
    if middle_x is None or middle_y is None:
        print("Error: Could not determine beam center.")
        break

    dx_pixel = x_target - middle_x
    dy_pixel = y_target - middle_y

    print(f"Attempt {attempt + 1}: dx = {dx_pixel}, dy = {dy_pixel} (pixels)")

    if abs(dx_pixel) <= margin and abs(dy_pixel) <= margin:
        print("Success! Target reached within margin.")
        break

    dx_motor = int(dx_pixel / slope_x) if slope_x != 0 else 0
    dy_motor = int(dy_pixel / slope_y) if slope_y != 0 else 0

    print(f"Moving motor: dx = {dx_motor} steps, dy = {dy_motor} steps")

    piezomotor(dx_motor, dy_motor, 0, 0, steprate, cam)
    time.sleep(5)
    attempt += 1

else:
    print("Failed to reach target within margin after maximum attempts.")
