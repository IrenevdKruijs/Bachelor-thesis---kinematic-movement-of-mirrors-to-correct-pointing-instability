import cv2
import numpy as np
import os
from datetime import datetime
import clr
import matplotlib.pyplot as plt
import csv
import time

# [Previous imports and class definitions remain unchanged]
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
runtime = 30  # time in minutes
wavelength = 532  # nm, example value
laser_power = 100  # mW, example value
camera_model = "Basler acA1300-60gm"  # Example camera model

# Create directory for saving data
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
save_dir = f"beam_data_{current_time}"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Save metadata to text file
metadata_path = os.path.join(save_dir, "metadata.txt")
with open(metadata_path, "w") as f:
    f.write(f"Date and Time: {current_time}\n")
    f.write(f"Wavelength: {wavelength} nm\n")
    f.write(f"Laser Power: {laser_power} mW\n")
    f.write(f"Camera Model: {camera_model}\n")
    f.write(f"Pixel Size: {pixelsize} m\n")
    f.write(f"Runtime: {runtime} minutes\n")

# Capture initial beam position
start_time = time.time()  # Record start time
img = cam.capture_image(2)
x0, y0 = localize_beam_center(img)

# Save initial image with green dot
img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
cv2.circle(img_color, (x0, y0), 5, (0, 255, 0), -1)  # Green dot
initial_image_path = os.path.join(save_dir, "initial_image.png")
cv2.imwrite(initial_image_path, img_color)

# Lists to store data
times = []
x_deviations = []
y_deviations = []
x_deviations_dist = []
y_deviations_dist = []
image_paths = []  # Store paths for video creation

# CSV file setup
csv_filename = os.path.join(save_dir, 'beam_deviation_over_time.csv')
with open(csv_filename, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['Time (s)', 'X Deviation (pixels)', 'Y Deviation (pixels)', 'X Deviation (m)', 'Y Deviation (m)'])

    # Main loop
    for i in range(runtime):
        time.sleep(60)
        current_time = time.time()
        elapsed_time = current_time - start_time  # Time since start in seconds
        img = cam.capture_image(2)
        x1, y1 = localize_beam_center(img)

        # Plot green dot on image and save
        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.circle(img_color, (x1, y1), 5, (0, 255, 0), -1)  # Green dot
        image_path = os.path.join(save_dir, f"image_{i:03d}.png")
        cv2.imwrite(image_path, img_color)
        image_paths.append(image_path)  # Store for video

        # Deviation in pixels
        x_dev = x1 - x0
        y_dev = y1 - y0
        # Deviation in distance
        x_dev_dist = x_dev * pixelsize
        y_dev_dist = y_dev * pixelsize
        
        # Store data
        times.append(elapsed_time)
        x_deviations.append(x_dev)
        y_deviations.append(y_dev)
        x_deviations_dist.append(x_dev_dist)
        y_deviations_dist.append(y_dev_dist)
        
        # Write to CSV
        csv_writer.writerow([elapsed_time, x_dev, y_dev, x_dev_dist, y_dev_dist])

# Create video from saved images
if image_paths:
    # Read first image to get dimensions
    first_img = cv2.imread(image_paths[0])
    height, width = first_img.shape[:2]
    
    # Initialize video writer
    video_path = os.path.join(save_dir, "beam_deviation_video.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MP4V codec
    fps = 10  # Frames per second
    video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
    
    # Write initial image
    video_writer.write(first_img)
    
    # Write all images to video
    for img_path in image_paths:
        img = cv2.imread(img_path)
        video_writer.write(img)
    
    # Release video writer
    video_writer.release()
    print(f"Video saved as {video_path}")

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(times, x_deviations, color='blue', linestyle="-", label='X Deviation', alpha=0.3)
plt.plot(times, y_deviations, color='red', linestyle="-", label='Y Deviation', alpha=0.3)
plt.scatter(times, x_deviations, color='blue', label='X Deviation', alpha=0.5)
plt.scatter(times, y_deviations, color='red', label='Y Deviation', alpha=0.5)
plt.xlabel('Time (seconds)')
plt.ylabel('Deviation (pixels)')
plt.title('Beam Center Deviation Over Time (Pixels)')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(save_dir, 'beam_deviation_plot_pixels.png'))
plt.show()

plt.figure(figsize=(10, 6))
plt.scatter(times, x_deviations_dist, color='blue', label='X Deviation', alpha=0.5)
plt.scatter(times, y_deviations_dist, color='red', label='Y Deviation', alpha=0.5)
plt.xlabel('Time (seconds)')
plt.ylabel('Deviation (m)')
plt.title('Beam Center Deviation Over Time (Distance)')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(save_dir, 'beam_deviation_plot_distance.png'))
plt.show()

# [Rest of your code, including class definitions, remains unchanged]