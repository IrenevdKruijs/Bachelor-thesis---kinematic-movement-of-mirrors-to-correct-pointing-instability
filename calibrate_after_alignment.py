"""
TO DO:
- implement this script in the main algorithm alignment so users do not have to run a separate script after alignment (it only has to be done once or after new alignment   
    so not every day, is it still nessecery to implement it?)
    
This script finds the middle of the beam and displays those coordinates. This script can be used after alignment to find the target pixels.
"""

from functions import *
filename = "target_pixels_cam1.txt"
exposuretime=10000
cam=camera_controller(exposuretime)
img = cam.capture_image(1)
target_x, target_y = localize_beam_center(img)
print(f"target pixel is ({target_x},{target_y})")
if os.path.exists(filename):
    overwrite = input(f"The file '{filename}' already exists. Do you want to overwrite it? (yes/no): ").strip().lower()
    if overwrite == "yes":
        with open(filename, "w") as f:
            f.write(f"{target_x},{target_y}")
    else:
        print("File not overwritten.")
else:
    with open(filename, "w") as f:
        f.write(f"{target_x},{target_y}")
        

filename = "target_pixels_cam2.txt"
img2 = cam.capture_image(2)
target_x2, target_y2 = localize_beam_center(img2)
print(f"target pixel is ({target_x2},{target_y2})")
if os.path.exists(filename):
    overwrite = input(f"The file '{filename}' already exists. Do you want to overwrite it? (yes/no): ").strip().lower()
    if overwrite == "yes":
        with open(filename, "w") as f:
            f.write(f"{target_x},{target_y}")
    else:
        print("File not overwritten.")
else:
    with open(filename, "w") as f:
        f.write(f"{target_x},{target_y}")