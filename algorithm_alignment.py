import time
import clr
import math
import json
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

margin = 5  # ~27.5µm
max_attempts = 1
pixelsize = 5.5e-6  # meters
A, B, C = 0.260, 0.440, 0.350  # meters
max_steps = 5000  # Cap to prevent excessive movements

# Load slope lookup table
try:
    with open("slope_lookup.json", 'r') as f:
        slopes = json.load(f)
except FileNotFoundError:
    print("Error: slope_lookup.json not found. Run create_slope_lookup.py first.")
    exit()

# Verify slopes and print for debugging
for channel in [1, 2, 3, 4]:
    for direction in [1, -1]:
        key = f"chan{channel}_dir{direction}"
        if key not in slopes:
            print(f"Error: No slope data for {key} in lookup table.")
            exit()
        slope = slopes[key]["slope"]
        print(f"Slope for {key}: {slope:.4f} pixels/step (R² = {slopes[key]['r_squared']:.4f})")
        if slope < 0.05:
            print(f"Warning: Slope for {key} is very small ({slope:.4f}). Consider re-running check_backlash.")

# Setup camera and motor
cam = camera_controller(exposuretime=10000)
motor = PiezoMotor(serial_number="97251304")

# Ensure flip mirror is in upright position
flipmirror(1)

# Check if laser is already aligned
image1 = cam.capture_image(1)
current_x, current_y = localize_beam_center(image1)
print(f"Camera 1: x={current_x}, y={current_y}")

image2 = cam.capture_image(2)
current_x2, current_y2 = localize_beam_center(image2)
print(f"Camera 2: x={current_x2}, y={current_y2}")

if (x1_target - margin <= current_x <= x1_target + margin and 
    y1_target - margin <= current_y <= y1_target + margin and 
    x2_target - margin <= current_x2 <= x2_target + margin and 
    y2_target - margin <= current_y2 <= y2_target + margin):
    print("Laser already correctly aligned")
    flipmirror(2)
    motor.shutdown()
    exit()

# Alignment loop
attempt = 0
while attempt < max_attempts:
    # Capture current positions
    image1 = cam.capture_image(1)
    middle_x1, middle_y1 = localize_beam_center(image1)
    if middle_x1 is None or middle_y1 is None:
        print("Error: Could not determine beam center at camera 1.")
        break

    image2 = cam.capture_image(2)
    middle_x2, middle_y2 = localize_beam_center(image2)
    if middle_x2 is None or middle_y2 is None:
        print("Error: Could not determine beam center at camera 2.")
        break

    # Calculate desired pixel movements
    dx1_pixel = x1_target - middle_x1
    dy1_pixel = y1_target - middle_y1
    dx2_pixel = x2_target - middle_x2
    dy2_pixel = y2_target - middle_y2
    
    #test numbers
    dx1_pixel = 100
    dx2_pixel = 125
    dy1_pixel = 0
    dy2_pixel = 0
    print("dx and dy pixels are manipulated!")
    print(f"Pixel deviations: dx1={dx1_pixel:.2f}, dy1={dy1_pixel:.2f}, dx2={dx2_pixel:.2f}, dy2={dy2_pixel:.2f}")

    # Check if all axes are within margin
    if abs(dx1_pixel) <= margin and abs(dy1_pixel) <= margin and abs(dx2_pixel) <= margin and abs(dy2_pixel) <= margin:
        print(f"Success! Target reached within margin. Final deviation: dx1={dx1_pixel}, dy1={dy1_pixel}, dx2={dx2_pixel}, dy2={dy2_pixel}")
        flipmirror(2)
        break

    # Convert pixel movements to meters
    dx1_height = dx1_pixel * pixelsize
    dy1_height = dy1_pixel * pixelsize
    dx2_height = dx2_pixel * pixelsize
    dy2_height = dy2_pixel * pixelsize

    # Calculate required mirror movements (in radians)
    alpha_y = math.atan((dy2_height - dy1_height) / C)
    h_y = -(A + B + C) * math.tan(alpha_y) + dy2_height
    MM1_y = alpha_y + math.tan(h_y / A)
    MM2_y = -math.tan(h_y / A)

    alpha_x = math.atan((dx2_height - dx1_height) / C)
    h_x = -(A + B + C) * math.tan(alpha_x) + dx2_height
    MM1_x = alpha_x + math.tan(h_x / A)
    MM2_x = -math.tan(h_x / A)
    print(f"Angles (radians): MM1_x={MM1_x:.6f}, MM1_y={MM1_y:.6f}, MM2_x={MM2_x:.6f}, MM2_y={MM2_y:.6f}")
    


    # Convert angular movements to pixel movements
    pixels_per_radian_1 = (B+C) / pixelsize # pixels/radian for mirror 1
    pixels_per_radian_2 = (A+B + C) / pixelsize  # pixels/radian for mirror 2
    MM2_x_pixels = MM2_x * pixels_per_radian_2
    MM2_y_pixels = MM2_y * pixels_per_radian_2
    MM1_x_pixels = MM1_x * pixels_per_radian_1
    MM1_y_pixels = MM1_y * pixels_per_radian_1
    print(f"Pixel movements: MM1_x={MM1_x_pixels:.2f}, MM1_y={MM1_y_pixels:.2f}, MM2_x={MM2_x_pixels:.2f}, MM2_y={MM2_y_pixels:.2f}")

    # Determine directions
    dx1_dir = 1 if MM1_x_pixels > 0 else -1
    dy1_dir = 1 if MM1_y_pixels > 0 else -1
    dx2_dir = 1 if MM2_x_pixels > 0 else -1
    dy2_dir = 1 if MM2_y_pixels > 0 else -1


    # Calculate steps using slopes, with cap
    dx1_steps = int(abs(MM1_x_pixels)/ slopes[f"chan1_dir{dx1_dir}"]["slope"]) * dx1_dir if abs(dx1_pixel) > margin else 0
    print(dx1_steps)
    dy1_steps = int(abs(MM1_y_pixels)/slopes[f"chan2_dir{dy1_dir}"]["slope"]) * dy1_dir if abs(dy1_pixel) > margin else 0
    dx2_steps = int(abs(MM2_x_pixels) / slopes[f"chan3_dir{dx2_dir}"]["slope"]) * dx2_dir if abs(dx2_pixel) > margin else 0
    dy2_steps = int(abs(MM2_y_pixels)/ slopes[f"chan4_dir{dy2_dir}"]["slope"]) * dy2_dir if abs(dy2_pixel) > margin else 0
    print(f"Steps: dx1 steps={dx1_steps}, dy1 steps={dy1_steps}, dx2 steps={dx2_steps}, dy2 steps={dy2_steps}")
    
    # Cap steps to prevent excessive movement
    # dx1_steps = max(min(dx1_steps, max_steps), -max_steps) if dx1_steps != 0 else 0
    # dy1_steps = max(min(dy1_steps, max_steps), -max_steps) if dy1_steps != 0 else 0
    # dx2_steps = max(min(dx2_steps, max_steps), -max_steps) if dx2_steps != 0 else 0
    # dy2_steps = max(min(dy2_steps, max_steps), -max_steps) if dy2_steps != 0 else 0

    print(f"Steps: dx1 steps={dx1_steps}, dy1 steps={dy1_steps}, dx2 steps={dx2_steps}, dy2 steps={dy2_steps}")

    if dx1_steps == 0 and dy1_steps == 0 and dx2_steps == 0 and dy2_steps == 0:
        print("No movement needed; all axes within margin or no valid correction.")
        break

    # Perform movement
    #tijdelijk, weghalen als ook met spiegel 2 werken

    dy2_steps=0
    dy1_steps = 0
    motor.move_steps(dx1_steps, dy1_steps, dx2_steps, dy2_steps, backlash_correction=False)
    attempt += 1

else:
    print("Failed to reach target within margin after maximum attempts.")

flipmirror(2)
motor.shutdown()