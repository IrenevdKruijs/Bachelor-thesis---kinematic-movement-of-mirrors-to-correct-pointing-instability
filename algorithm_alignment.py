import time
import clr
import math
import json
from functions import *
import cv2

# Initialize camera and motor libraries
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.InertialMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *

# Constants
try:
    target_img_cam1 = cv2.imread("target_pixels_cam1.png")
    target_img_cam2 = cv2.imread("target_pixels_cam2.png")
except FileNotFoundError:
    print("Align setup and run calibrate_after_alignment")
    exit()

margin = 5  # ~27.5µm
max_attempts = 2
pixelsize = 5.5*(10**-6)  # meters
A, B, C = 0.255, 0.445, 0.345  # meters
max_steps = 5000  # Cap to prevent excessive movements
deviations = []
# Load slope lookup table
# fix the except statement, probably not correct like this
try:
    with open("slope_lookup.json", 'r') as f:
        slopes = json.load(f)
except FileNotFoundError:
    create_slope_lookup([1,2,3,4],[1,-1])
    

# Verify slopes and print for debugging
for channel in [1, 2, 3, 4]:
    for direction in [1, -1]:
        key = f"chan{channel}_dir{direction}"
        if key not in slopes:
            print(f"Error: No slope data for {key} in lookup table.")
            exit()
        
        # Determine whether to use slope_x or slope_y based on channel
        slope_key = "slope_x" if channel in [1, 3] else "slope_y"
        r_squared_key = "r_squared_x" if channel in [1, 3] else "r_squared_y"
        
        slope = slopes[key][slope_key]
        r_squared = slopes[key][r_squared_key]
        
        print(f"Slope for {key}: {slope:.4f} pixels/step (R² = {r_squared:.4f})")
        if abs(slope) < 0.05:  # Use abs() to handle negative slopes
            print(f"Warning: Slope for {key} is below 0.05 pixels/step")
# Setup camera and motor
cam = camera_controller(exposuretime=50)
motor = PiezoMotor(serial_number="97251304")

# Ensure flip mirror is in upright position
flipmirror(1)

# Check if laser is already aligned
image1 = cam.capture_image(1)
image2 = cam.capture_image(2)

correlated_img1,x1_dev,y1_dev = localize_beam_center(target_img_cam1,image1)
correlated_img2,x2_dev,y2_dev = localize_beam_center(target_img_cam2,image2)

plt.imshow(correlated_img1)
plt.show()
plt.imshow(correlated_img2)
plt.show
if (abs(x1_dev) <= margin and 
    abs(y1_dev) and 
    abs(x2_dev) <= margin and 
    abs(y2_dev) <= margin):
    print("Laser already correctly aligned")
    flipmirror(2)
    motor.shutdown()
    exit()

# Alignment loop
attempt = 0
while attempt < max_attempts:
    # Capture current positions
    image1 = cam.capture_image(1)
    _,dx1_pixel,dy1_pixel = localize_beam_center(target_img_cam1,image1)
    if dx1_pixel is None or dy1_pixel is None:
        print("Error: Could not determine beam center at camera 1.")
        break
    #debugging
    print(f"dx1_pixel = {dx1_pixel}")
    plt.imshow(image1)
    plt.show()
    
    image2 = cam.capture_image(2)
    _,dx2_pixel,dy2_pixel = localize_beam_center(target_img_cam2,image2)
    if dx2_pixel is None or dy2_pixel is None:
        print("Error: Could not determine beam center at camera 2.")
        break
    plt.imshow(image2)
    plt.show
    dx1_pixel = -1* dx1_pixel
    print(f"dx1_pixel = {dx1_pixel}, dx2_pixel = {dx2_pixel}, dy1_pixel = {dy1_pixel}, dy2_pixel = {dy2_pixel}")
    print(f"Pixel deviations: dx1={dx1_pixel:.2f}, dy1={dy1_pixel:.2f}, dx2={dx2_pixel:.2f}, dy2={dy2_pixel:.2f}")

    # Check if all axes are within margin
    if abs(dx1_pixel) <= margin and abs(dy1_pixel) <= margin and abs(dx2_pixel) <= margin and abs(dy2_pixel) <= margin:
        print(f"Success! Target reached within margin. Final deviation: dx1={dx1_pixel}, dy1={dy1_pixel}, dx2={dx2_pixel}, dy2={dy2_pixel}")
        flipmirror(2)
        break

    # Store initial deviations for this attempt
    deviations.append({
        'attempt': attempt + 1,
        'dx1_before': dx1_pixel,
        'dy1_before': dy1_pixel,
        'dx2_before': dx2_pixel,
        'dy2_before': dy2_pixel
    })

    # Convert pixel movements to meters
    dx1_height = dx1_pixel * pixelsize
    dy1_height = dy1_pixel * pixelsize
    dx2_height = dx2_pixel * pixelsize
    dy2_height = dy2_pixel * pixelsize

    print(f"dx1_height = {dx1_height}, dx2_height = {dx2_height}")
    # Calculate required mirror movements (in radians)
    # required mirror movement is half of required beam movement, because when you turn the mirror with an angle alpha,
    # the beam turns with angle 2*alpha. We call the optical angle alpha_o and the mechanical alpha alpha_m, which is 
    # half
    
    alpha_y = math.atan((dy2_height - dy1_height) / C)
    h_y = -((A + B + C) * math.tan(alpha_y) - dy2_height)
    beta_y = math.atan(h_y/A)
    MM1_y = -(alpha_y + beta_y)
    MM2_y = (beta_y)

    alpha_x = math.atan((dx2_height - dx1_height) / C)
    h_x = -((A + B + C) * math.tan(alpha_x) - dx2_height)
    beta_x = math.atan(h_x/A)
    #if h is positive, beta moet negatief zijn, 
    MM1_x = -(alpha_x + beta_x)
    #we need to correct the opposite of the output so minus
    MM2_x = (beta_x)
    print(f"Angles (radians): MM1_x={MM1_x:.6f}, MM1_y={MM1_y:.6f}, MM2_x={MM2_x:.6f}, MM2_y={MM2_y:.6f}")
    
    # Convert angular movements to pixel movements
    MM2_x_pixels = (MM2_x * (B+C))/pixelsize
    MM2_y_pixels = (MM2_y *(B+C))/pixelsize
    MM1_x_pixels = (MM1_x *(A+B+C))/pixelsize
    MM1_y_pixels = (MM1_y *(A+B+C))/pixelsize
    print(f"Pixel movements: MM1_x={MM1_x_pixels:.2f}, MM1_y={MM1_y_pixels:.2f}, MM2_x={MM2_x_pixels:.2f}, MM2_y={MM2_y_pixels:.2f}")

    # Determine directions
    dx1_dir = 1 if MM1_x_pixels > 0 else -1
    dy1_dir = 1 if MM1_y_pixels > 0 else -1
    dx2_dir = 1 if MM2_x_pixels > 0 else -1
    dy2_dir = 1 if MM2_y_pixels > 0 else -1

    # Calculate steps using slopes, with cap
    #All steps need a factor of -1 to send them in the right direction.
    dx1_steps = int(MM1_x_pixels / slopes[f"chan1_dir{dx1_dir}"]["slope_x"]) if abs(MM1_x_pixels) > margin else 0
    print(dx1_steps)
    # dx1_steps = int(MM1_x_pixels/slopes[f"chan1_dir{dx1_dir}"]["slope_x"])if abs(MM1_x_pixels) > margin else 0
    dy1_steps = int(MM1_y_pixels/slopes[f"chan2_dir{dy1_dir}"]["slope_y"])  if abs(dy1_pixel) > margin else 0
    dx2_steps = int(MM2_x_pixels/ slopes[f"chan3_dir{dx2_dir}"]["slope_x"]) if abs(dx2_pixel) > margin else 0
    dy2_steps = int(MM2_y_pixels/ slopes[f"chan4_dir{dy2_dir}"]["slope_y"]) if abs(dy2_pixel) > margin else 0
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
    # dy2_steps=0
    # dy1_steps = 0
    
    
    motor.move_steps(dx1_steps, dy1_steps, dx2_steps, dy2_steps, backlash_correction=False)
    attempt += 1
    
    
    # Capture current positions
    image1 = cam.capture_image(1)
    _,dx1_pixel_after,dy1_pixel_after = localize_beam_center(target_img_cam1,image1)
    if dx1_pixel_after is None or dy1_pixel_after is None:
        print("Error: Could not determine beam center at camera 1.")
        break
    #debugging
    plt.imshow(image1)
    plt.show()
    
    image2 = cam.capture_image(2)
    _,dx2_pixel_after,dy2_pixel_after = localize_beam_center(target_img_cam2,image2)
    if dx2_pixel is None or dy2_pixel is None:
        print("Error: Could not determine beam center at camera 2.")
        break
    plt.imshow(image2)
    plt.show
    
        # Store final deviations for this attempt
    deviations[-1]['dx1_after'] = dx1_pixel_after
    deviations[-1]['dy1_after'] = dy1_pixel_after
    deviations[-1]['dx2_after'] = dx2_pixel_after
    deviations[-1]['dy2_after'] = dy2_pixel_after
    
    print(f"Final deviations: dx1_pixel = {dx1_pixel_after}, dx2_pixel = {dx2_pixel_after}, dy1_pixel = {dy1_pixel_after}, dy2_pixel = {dy2_pixel_after}")
    print(f"improvement: dx1: {dx1_pixel} -> {dx1_pixel_after}, dx2: {dx2_pixel}-> {dx2_pixel_after}, dy1: {dy1_pixel}-> {dy1_pixel_after},dy2: {dy2_pixel}->{dy2_pixel_after}")
    # Check if all axes are within margin
    if abs(dx1_pixel_after) <= margin and abs(dy1_pixel_after) <= margin and abs(dx2_pixel_after) <= margin and abs(dy2_pixel_after) <= margin:
        print(f"Success! Target reached within margin. Final deviation: dx1={dx1_pixel_after}, dy1={dy1_pixel_after}, dx2={dx2_pixel_after}, dy2={dy2_pixel_after}")
        flipmirror(2)
        break
else:
    print("Failed to reach target within margin after maximum attempts.")


plt.figure(figsize=(8, 6))
for dev in deviations:
    attempt_num = dev['attempt']
    plt.scatter(dev['dx1_before'], dev['dy1_before'], label=f'Attempt {attempt_num} Before', marker='o')
    plt.scatter(dev['dx1_after'], dev['dy1_after'], label=f'Attempt {attempt_num} After', marker='x')
plt.xlabel('X Pixel Deviation')
plt.xlabel('Pixels')
plt.ylabel('Pixels')
plt.title(f'Deviation from target before and after using algorithm ')
plt.grid(True)
plt.legend()
plt.savefig("deviation from target after algorithm2.png")
plt.close()


flipmirror(2)
motor.shutdown()