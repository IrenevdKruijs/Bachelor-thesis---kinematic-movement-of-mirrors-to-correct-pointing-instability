"""
TO DO:
- implement this script in the main algorithm alignment so users do not have to run a separate script after alignment (it only has to be done once or after new alignment   
    so not every day, is it still nessecery to implement it?)
    
This script finds the middle of the beam and displays those coordinates. This script can be used after alignment to find the target pixels.
"""

from functions import *
import cv2
import os

exposuretime = 50
cam = camera_controller(exposuretime)

img = cam.capture_image(1)
filename = "target_pixels_cam1.png"

if os.path.exists(filename):
    overwrite = input(f"The file '{filename}' already exists. Do you want to overwrite it? (yes/no): ").strip().lower()
    if overwrite == "yes":
        cv2.imwrite(filename, img)
        print(f"Image saved as {filename}")
    else:
        print("File not overwritten.")
else:
    cv2.imwrite(filename, img)
    print(f"Image saved as {filename}")

#make target image for camera 2
img = cam.capture_image(2)
filename = "target_pixels_cam2.png"

if os.path.exists(filename):
    overwrite = input(f"The file '{filename}' already exists. Do you want to overwrite it? (yes/no): ").strip().lower()
    if overwrite == "yes":
        cv2.imwrite(filename, img)
        print(f"Image saved as {filename}")
    else:
        print("File not overwritten.")
else:
    cv2.imwrite(filename, img)
    print(f"Image saved as {filename}")