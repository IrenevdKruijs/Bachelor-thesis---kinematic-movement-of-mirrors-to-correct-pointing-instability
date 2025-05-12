from functions import *
camera=camera_setup()
img = image(camera,5000)
target_x, target_y = coordinates(img,0,0,0,0)
print(f"target pixel is ({target_x},{target_y})")
