import cv2
import numpy as np
import os
from datetime import datetime
import clr
import matplotlib.pyplot as plt
import csv
import time

# Initialize camera and motor libraries
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.InertialMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *
from functions import *

# Initialize camera
cam = camera_controller()
pixelsize = 5.5e-6  # m
runtime = 60  # time in minutes
wavelength = 800  # nm
laser_power = 6  #%
camera_model = "Basler acA2000-165umNIR" 
rest_interval = 300
# Create directory for saving data
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
save_dir = f"beam_pointing_instability_data_{current_time}"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Save metadata to text file
metadata_path = os.path.join(save_dir, "metadata.txt")
with open(metadata_path, "w") as f:
    f.write(f"Date and Time: {current_time}\n")
    f.write(f"Wavelength: {wavelength} nm\n")
    f.write(f"Laser Power: {laser_power} %")
    f.write(f"Camera Model: {camera_model}\n")
    f.write(f"Pixel Size: {pixelsize} m\n")
    f.write(f"Runtime: {runtime} minutes\n")
    f.write(f"pause between measurements: {rest_interval} seconds")
    f.write("Cameras Used: Camera 1 and Camera 2\n")

# Capture initial beam positions for both cameras
start_time = time.time()  # Record start time
initial_img_cam1 = cam.capture_image(1)
initial_img_cam2 = cam.capture_image(2)

# Save initial images
initial_image_path_cam1 = os.path.join(save_dir, "initial_image_cam1.png")
cv2.imwrite(initial_image_path_cam1,initial_img_cam1)

initial_image_path_cam2 = os.path.join(save_dir, "initial_image_cam2.png")
cv2.imwrite(initial_image_path_cam2, initial_img_cam2)

# Lists to store data for both cameras
times = []
x_deviations_cam1 = []
y_deviations_cam1 = []
x_deviations_cam2 = []
y_deviations_cam2 = []
image_paths_cam1 = []
image_paths_cam2 = []

# CSV file setup for both cameras
csv_filename = os.path.join(save_dir, f'beam_deviation_over_time_{current_time}.csv')
with open(csv_filename, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['Time (s)', 
                        'Cam1 X Deviation (pixels)', 'Cam1 Y Deviation (pixels)', 
                        'Cam1 X Deviation (m)', 'Cam1 Y Deviation (m)',
                        'Cam2 X Deviation (pixels)', 'Cam2 Y Deviation (pixels)', 
                        'Cam2 X Deviation (m)', 'Cam2 Y Deviation (m)'])

    # Main loop
    for i in range(runtime):
        time.sleep(rest_interval)
        current_time = time.time()
        elapsed_time = current_time - start_time  # Time since start in seconds
        
        # Capture images from both cameras
        img_cam1 = cam.capture_image(1)
        img_cam2 = cam.capture_image(2)

        image_path_cam1 = os.path.join(save_dir, f"image_cam1_{i:03d}.png")
        cv2.imwrite(image_path_cam1)
        image_paths_cam1.append(image_path_cam1)

        image_path_cam2 = os.path.join(save_dir, f"image_cam2_{i:03d}.png")
        cv2.imwrite(image_path_cam2)
        image_paths_cam2.append(image_path_cam2)

        # Deviation in pixels
        x_dev_cam1,y_dev_cam1 = localize_beam_center(initial_img_cam1,img_cam1)
        x_dev_cam2,y_dev_cam2 = localize_beam_center(initial_img_cam2,img_cam2)
        
        # Store data
        times.append(elapsed_time)
        x_deviations_cam1.append(x_dev_cam1)
        y_deviations_cam1.append(y_dev_cam1)
        x_deviations_cam2.append(x_dev_cam2)
        y_deviations_cam2.append(y_dev_cam2)

        
        # Write to CSV
        csv_writer.writerow([elapsed_time, 
                           x_dev_cam1, y_dev_cam1,
                           x_dev_cam2, y_dev_cam2])

# Create videos from saved images for both cameras
for cam_id, image_paths in [(1, image_paths_cam1), (2, image_paths_cam2)]:
    if image_paths:
        # Read first image to get dimensions
        first_img = cv2.imread(image_paths[0])
        height, width = first_img.shape[:2]
        
        # Initialize video writer
        video_path = os.path.join(save_dir, f"beam_deviation_video_cam{cam_id}.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MP4V codec
        fps = 10  # Frames per second
        video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
        
        # Write initial image
        video_writer.write(cv2.imread(f"initial_image_cam{cam_id}.png"))
        
        # Write all images to video
        for img_path in image_paths:
            img = cv2.imread(img_path)
            video_writer.write(img)
        
        # Release video writer
        video_writer.release()
        print(f"Video for camera {cam_id} saved as {video_path}")

# Plotting for both cameras
plt.figure(figsize=(10, 6))
plt.plot(times, x_deviations_cam1, color='blue', linestyle="-", label='Cam1 X Deviation', alpha=0.3)
plt.plot(times, y_deviations_cam1, color='red', linestyle="-", label='Cam1 Y Deviation', alpha=0.3)
plt.plot(times, x_deviations_cam2, color='cyan', linestyle="-", label='Cam2 X Deviation', alpha=0.3)
plt.plot(times, y_deviations_cam2, color='magenta', linestyle="-", label='Cam2 Y Deviation', alpha=0.3)
plt.scatter(times, x_deviations_cam1, color='blue', label='Cam1 X Deviation', alpha=0.5)
plt.scatter(times, y_deviations_cam1, color='red', label='Cam1 Y Deviation', alpha=0.5)
plt.scatter(times, x_deviations_cam2, color='cyan', label='Cam2 X Deviation', alpha=0.5)
plt.scatter(times, y_deviations_cam2, color='magenta', label='Cam2 Y Deviation', alpha=0.5)
plt.xlabel('Time (seconds)')
plt.ylabel('Deviation (pixels)')
plt.title('Beam Center Deviation Over Time (Pixels)')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(save_dir, 'beam_deviation_plot_pixels.png'))
plt.show()

