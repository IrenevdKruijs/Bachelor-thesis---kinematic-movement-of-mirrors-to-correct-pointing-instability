import clr
import math
import json
from functions import *
import cv2
from datetime import datetime
# Initialize camera and motor libraries
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.InertialMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *


# Load images of target position of the beam
try:
    target_img_cam1 = cv2.imread("target_pixels_cam1.png")
    target_img_cam2 = cv2.imread("target_pixels_cam2.png")
except FileNotFoundError:
    print("Align setup and run calibrate_after_alignment")
    exit()

#set constants
wavelength = int(input("what wavelength are you using?:"))
margin = 5  # ~27.5µm
max_attempts = 3
pixelsize = 5.5*(10**-6)  # meters, 
A, B, C, D = 0.255, 0.260, 0.345, 0.87# meters, A is distance between mirror 1 and mirror 2, B = distance between mirror 2 and cam 1, C = distance between cam1 and cam2
deviations = [] #initialize deviations to save the deviations from the target position of the beam


# Get current datetime
current_datetime = datetime.now()
# Output: YYYY-MM-DD HH:MM:SS.microseconds (e.g., 2025-06-18 20:13:45.123456)

# Format time only
current_time = current_datetime.strftime("%H_%M_%S")
 # Output: HH:MM:SS (e.g., 20:13:45)

# Load slope lookup table
# TO DO: fix the except statement, probably not correct like this
try:
    with open("slope_lookup.json", 'r') as f:
        slopes = json.load(f)
except FileNotFoundError:
    create_slope_lookup([1,2,3,4],[1,-1])


# Verify slopes
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
            
# Initialize camera and motor
cam = camera_controller(wavelength,exposuretime=50)
motor = PiezoMotor(serial_number="97251304")

# Ensure flip mirror is in upright position
flipmirror(1)

# Check if laser is already aligned
#taking images of the current position of the beam
image1 = cam.capture_image(1)
image2 = cam.capture_image(2)

# Do a cross correlation of the target image and the current image of the beam to find the shift
correlated_img1,x1_dev,y1_dev = localize_beam_center(target_img_cam1,image1)
correlated_img2,x2_dev,y2_dev = localize_beam_center(target_img_cam2,image2)

# Check if laser is already correctly aligned 
if (abs(x1_dev) <= margin and 
    abs(y1_dev) <= margin and 
    abs(x2_dev) <= margin and 
    abs(y2_dev) <= margin):
    print("Laser already correctly aligned")
    flipmirror(2)
    motor.shutdown()
    exit()

# Alignment loop
attempt = 0
# Stop alignment after a maximum number of attempts
while attempt < max_attempts:
    
    # Capture picture of current position of beam for camera 1 and 2 and calculate shift by doing cross correlation
    image1 = cam.capture_image(1)
    _,dev_x1_pixel,dev_y1_pixel = localize_beam_center(target_img_cam1,image1)
    if dev_x1_pixel is None or dev_y1_pixel is None:
        print("Error: Could not determine beam center at camera 1.")
        break
    
    image2 = cam.capture_image(2)
    _,dev_x2_pixel,dev_y2_pixel = localize_beam_center(target_img_cam2,image2)
    if dev_x2_pixel is None or dev_y2_pixel is None:
        print("Error: Could not determine beam center at camera 2.")
        break


    # The beam splitter swaps the direction of the deviation in the x-direction for camera 1 but not for camera 2.
    # That is why it is needed to swap the sign of the dx1_pixel. 
    # dev_x1_pixel = -1* dev_x1_pixel #Als het goed is is dit niet meer nodig want cam2 heeft nu ook een spiegel en is meegenomen in kalibratie
    # dev_x2_pixel = -1*dev_x2_pixel
    print(f"dx1_pixel = {dev_x1_pixel}, dx2_pixel = {dev_x2_pixel}, dy1_pixel = {dev_y1_pixel}, dy2_pixel = {dev_y2_pixel}")
    print(f"Pixel deviations: dx1={dev_x1_pixel:.2f}, dy1={dev_y1_pixel:.2f}, dx2={dev_x2_pixel:.2f}, dy2={dev_y2_pixel:.2f}")

    # Check if all axes are within margin
    if abs(dev_x1_pixel) <= margin and abs(dev_y1_pixel) <= margin and abs(dev_x2_pixel) <= margin and abs(dev_y2_pixel) <= margin:
        print(f"Success! Target reached within margin. Final deviation: dx1={dev_x1_pixel}, dy1={dev_y1_pixel}, dx2={dev_x2_pixel}, dy2={dev_y2_pixel}")
        flipmirror(2)
        break

    # Store initial deviations for this attempt
    deviations.append({
        'attempt': attempt + 1,
        'dx1_before': dev_x1_pixel,
        'dy1_before': dev_y1_pixel,
        'dx2_before': dev_x2_pixel,
        'dy2_before': dev_y2_pixel
    })

    # Convert pixel movements to meters
    dev_x1_meter = dev_x1_pixel * pixelsize
    dev_y1_meter = dev_y1_pixel * pixelsize
    dev_x2_meter = dev_x2_pixel * pixelsize
    dev_y2_meter = dev_y2_pixel * pixelsize

    # Calculate required mirror movements (in radians)
    # required mirror movement is half of required beam movement, because when you turn the mirror with an angle alpha,
    # the beam turns with angle 2*alpha. We call the optical angle alpha_o and the mechanical alpha alpha_m, which is 
    # half
    
    alpha_y = math.atan((dev_y2_meter - dev_y1_meter) / C)
    h_y = -((A + B + C)*math.tan(alpha_y) - dev_y2_meter)
    beta_y = math.atan(h_y/A)
    
    # treat special cases correctly
    if h_y > 0 and dev_y1_meter > dev_y2_meter and (dev_y1_meter > 0 and dev_y2_meter > 0):
        movement_MM1_y = abs(alpha_y) - abs(beta_y)
        movement_MM2_y = abs(beta_y)
    elif h_y < 0 and (dev_y1_meter <0 and dev_y2_meter <0) and dev_y1_meter < dev_y2_meter:
        movement_MM1_y = -(abs(alpha_y)-abs(beta_y))
        movement_MM2_y = -abs(beta_y)
    # elif h_y <0 and (dev_y1_meter and dev_y2_meter>0) and dev_y1_meter > dev_y2_meter:
    #     movement_MM1_y = -(abs(alpha_y)-abs(beta_y))
    #     movement_MM2_y = -abs(beta_y)
    elif h_y < 0 and (dev_y1_meter > 0 or dev_y2_meter > 0):
        movement_MM1_y = -(abs(alpha_y)-abs(beta_y))
        movement_MM2_y = -abs(beta_y)
        #lijkt te werken
    elif h_y > 0 and (dev_y1_meter < 0 or dev_y2_meter < 0):
        movement_MM1_y = -(abs(beta_y) - abs(alpha_y))
        movement_MM2_y = abs(beta_y)
        # werkt niet!!!!!!

    #deze situatie is verwerkt in de 'normale' situatie en dus overbodig
    # elif h_y < 0 and (dev_y1_meter and dev_y2_meter < 0) and dev_y2_meter < dev_y1_meter:
    #     movement_MM1_y = alpha_y+beta_y
    #     movement_MM2_y = - beta_y
    #deze situatie is dezelfde als de normale en dus eig overbodig
    # elif h_y > 0 and (dev_y1_meter and dev_y2_meter > 0) and dev_y1_meter < dev_y2_meter:
    #     movement_MM1_y = -(alpha_y+beta_y)
    #     movement_MM2_y = beta_y
    else:
        if h_y > 0: 
            movement_MM2_y = abs(beta_y) 
            movement_MM1_y = -(abs(alpha_y) + abs(beta_y)) # radians that MM1 has to move in y-direction
        else: 
            movement_MM2_y = -abs(beta_y)
            movement_MM1_y = (abs(alpha_y) + abs(beta_y)) # radians that MM1 has to move in y-direction#radians that MM2 has to move in y-direction

    alpha_x = math.atan((dev_x2_meter - dev_x1_meter) / C)
    h_x = -((A + B + C)* math.tan(alpha_x) - dev_x2_meter)
    beta_x = math.atan(h_x/A)
    #treat special cases correctly
    # treat special cases correctly
    if h_x > 0 and dev_x1_meter > dev_x2_meter and (dev_x1_meter > 0 and dev_x2_meter > 0):
        movement_MM1_x = abs(alpha_x) - abs(beta_x)
        movement_MM2_x = abs(beta_x)
    elif h_x < 0 and (dev_x1_meter < 0 and dev_x2_meter <0) and dev_x1_meter < dev_x2_meter:
        movement_MM1_x = -(abs(alpha_x)-abs(beta_x))
        movement_MM2_x = -abs(beta_x)
    elif h_x < 0 and (dev_x1_meter > 0 or dev_x2_meter > 0):
        movement_MM1_x = -(abs(alpha_x)-abs(beta_x))
        movement_MM2_x = - abs(beta_x)
    elif h_x > 0 and (dev_x1_meter < 0 or dev_x2_meter < 0):
        movement_MM1_x = -(abs(beta_x)-abs(alpha_x))
        movement_MM2_x = abs(beta_x)
        #lijkt te werken

    # elif h_x < 0 and (dev_x1_meter and dev_x2_meter < 0) and dev_x2_meter < dev_x1_meter:
    #     movement_MM1_x = alpha_x+beta_x
    #     movement_MM2_x = - beta_x
    elif h_x > 0 and (dev_x1_meter > 0  and dev_x2_meter > 0) and (dev_x1_meter < dev_x2_meter):
         movement_MM1_x = -(alpha_x+beta_x)
         movement_MM2_x = beta_x
         #Dit is toch de situatie bij Dx1=5 en Dx2=19?, waarom was dit uitgecomment?
         #En als deze niet uitgecomment hoeft, moet hij bovenaan door de and and and
    else:
        if h_x > 0: 
            movement_MM2_x = (abs(beta_x))
            movement_MM1_x = -(abs(alpha_x) + abs(beta_x)) # radians that MM1 has to move in y-direction
        else: 
            movement_MM2_x= -abs(beta_x)
            movement_MM1_x = (abs(alpha_x) + abs(beta_x)) # radians that MM1 has to move in y-direction#radians that MM2 has to move in y-direction

  # radians that MM2 has to move in x-direction
    print(f"Angles (radians): MM1_x={movement_MM1_x:.6f}, MM1_y={movement_MM1_y:.6f}, MM2_x={movement_MM2_x:.6f}, MM2_y={movement_MM2_y:.6f}")
    
    # Convert angular movements to pixel movements on camera 2, as the calibration was done with camera 2. This way,
    # movement in radians can be converted to a number of steps of the piezomotor that moves the kinematic mirrors.
    MM2_x_pixels = (movement_MM2_x * (B+C))/pixelsize
    MM2_y_pixels = (movement_MM2_y *(B+C))/pixelsize
    MM1_x_pixels = (movement_MM1_x *(A+B+C))/pixelsize
    MM1_y_pixels = (movement_MM1_y *(A+B+C))/pixelsize
    print(f"Pixel movements: MM1_x={MM1_x_pixels:.2f}, MM1_y={MM1_y_pixels:.2f}, MM2_x={MM2_x_pixels:.2f}, MM2_y={MM2_y_pixels:.2f}")
    
    # Determine direction of movement for both mirrors for tip and tilt
    dx1_dir = 1 if MM1_x_pixels > 0 else -1
    dy1_dir = 1 if MM1_y_pixels > 0 else -1
    dx2_dir = 1 if MM2_x_pixels > 0 else -1
    dy2_dir = 1 if MM2_y_pixels > 0 else -1

    # Calculate number of steps using slopes
    dx1_steps = int(MM1_x_pixels*slopes[f"chan1_dir{dx1_dir}"]["slope_x"]) 
    dy1_steps = int(MM1_y_pixels*slopes[f"chan2_dir{dy1_dir}"]["slope_y"])  
    dx2_steps = int(MM2_x_pixels*slopes[f"chan3_dir{dx2_dir}"]["slope_x"])
    dy2_steps = int(MM2_y_pixels*slopes[f"chan4_dir{dy2_dir}"]["slope_y"])
    print(f"Steps: dx1 steps={dx1_steps}, dy1 steps={dy1_steps}, dx2 steps={dx2_steps}, dy2 steps={dy2_steps}")

    # Skip movement if there is no movement needed 
    if dx1_steps == 0 and dy1_steps == 0 and dx2_steps == 0 and dy2_steps == 0:
        print("No movement needed; all axes within margin or no valid correction.")
        break

    # Perform calculated amount of steps   
    motor.move_steps(dx1_steps, dy1_steps, dx2_steps, dy2_steps, backlash_correction=False)
    time.sleep(2)
    attempt += 1
    
    # Capture current positions and calculate shift
    image1 = cam.capture_image(1)
    _,dx1_pixel_after,dy1_pixel_after = localize_beam_center(target_img_cam1,image1)
    if dx1_pixel_after is None or dy1_pixel_after is None:
        print("Error: Could not determine beam center at camera 1.")
        break
    
    image2 = cam.capture_image(2)
    _,dx2_pixel_after,dy2_pixel_after = localize_beam_center(target_img_cam2,image2)
    if dev_x2_pixel is None or dev_y2_pixel is None:
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
    print(f"improvement: dx1: {dev_x1_pixel} -> {dx1_pixel_after}, dx2: {dev_x2_pixel}-> {dx2_pixel_after}, dy1: {dev_y1_pixel}-> {dy1_pixel_after},dy2: {dev_y2_pixel}->{dy2_pixel_after}")
    # Check if all axes are within margin
    if abs(dx1_pixel_after) <= margin and abs(dy1_pixel_after) <= margin and abs(dx2_pixel_after) <= margin and abs(dy2_pixel_after) <= margin:
        print(f"Success! Target reached within margin. Final deviation: dx1={dx1_pixel_after}, dy1={dy1_pixel_after}, dx2={dx2_pixel_after}, dy2={dy2_pixel_after}")
        flipmirror(2)
        break
else:
    print("Failed to reach target within margin after maximum attempts.")


flipmirror(2)
motor.shutdown()
# Write deviations to CSV
csv_filename = f"beam_alignment_deviations_{wavelength}_18-6_{current_time}.csv"
with open(csv_filename, mode='w', newline='', encoding='utf-8') as file:
    headers = ['attempt', 'dx1_before', 'dy1_before', 'dx2_before', 'dy2_before', 'dx1_after', 'dy1_after', 'dx2_after', 'dy2_after']
    writer = csv.DictWriter(file, fieldnames=headers)
    writer.writeheader()
    for dev in deviations:
        writer.writerow(dev)
print(f"Deviations data saved to {csv_filename}")

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

