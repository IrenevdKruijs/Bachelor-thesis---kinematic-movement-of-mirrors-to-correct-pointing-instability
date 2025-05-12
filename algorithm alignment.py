"""
TO DO: 
- hoe groot moet marge zijn?
- zorgen dat camera het liefst niet meer nodig is in image-> is heel irritant want dan moet het in iedere functie
- kalibratie van slechte kwaliteit-> volgende keer dan wel opnieuw doen?
- script moet worden uitgevoerd als golflengte verandert en eens in de zoveel tijd
- als 1 vd 2 (x of y) al binnen de marge zit moet die niet meer bewegen-> verpest het vaak alleen maar
- waar in het script moet ik corrigeren voor de slack? locatie moet worden gemeten om te weten welke kant je op moet en 
    hoe je dus slack moet corrigeren, maar daarna moet je de positie opnieuw meten voordat je gaat bewegen

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

#This will be changed to get wavelength from system instead of manual input
# wavelength = input("Give wavelength used: ").strip()
# print(f"Input wavelength (repr): {repr(wavelength)}")

# try:
#     with open("last_wavelength.txt", "r") as f:
#         stored_wavelength = f.read().strip()
#         # Debug: Print the stored wavelength to inspect it
#         print(f"Stored wavelength (repr): {repr(stored_wavelength)}")
#         if stored_wavelength == wavelength:
#             print("Wavelength unchanged. Alignment complete.Do you still want to calibrate?")
            
# except FileNotFoundError:
#     pass

# with open("last_wavelength.txt", "w") as f:
#     f.write(wavelength)                      
# Constants
x_target = 1035
y_target = 561
margin = 10
steprate = 250
max_attempts = 5

# Setup camera
cam = camera_setup()
im=image(cam)
#make sure the flip mirror is in upright position
flipmirror(1)

# make sure the script only runs if the position of the laser is incorrect
current_x,current_y = coordinates(im,0,0,0,0)
if x_target - margin <= current_x <= x_target + margin and y_target - margin <= current_y <= y_target + margin:
    print("laser already correctly aligned")
    flipmirror(2)
    exit()
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
        flipmirror(2)
        break

    dx_motor = int(dx_pixel / slope_x) if slope_x != 0 else 0
    dy_motor = int(dy_pixel / slope_y) if slope_y != 0 else 0

    print(f"Moving motor: dx = {dx_motor} steps, dy = {dy_motor} steps")
    #hoeft eigenlijk alleen als richting verandert 
    piezomotor(dx_motor, dy_motor, 0, 0, steprate, cam)
    time.sleep(0.1)
    attempt += 1

else:
    print("Failed to reach target within margin after maximum attempts.")
    
