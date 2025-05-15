"""
TO DO:
- implement this script in the main algorithm alignment so users do not have to run a separate script after alignment (it only has to be done once or after new alignment   
    so not every day, is it still nessecery to implement it?)
    
This script finds the middle of the beam and displays those coordinates. This script can be used after alignment to find the target pixels.
"""

from functions import *

cam=camera_controller()
img = cam.capture_image()
target_x, target_y = coordinates(img)
print(f"target pixel is ({target_x},{target_y})")
with open("alignment_result.txt", "w") as f:
    f.write(f"{target_x},{target_y}")

