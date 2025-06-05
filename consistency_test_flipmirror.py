import cv2
import numpy as np
import os
from datetime import datetime
import clr
import matplotlib.pyplot as plt
import csv
import time
from functions import *

# Initialize camera and motor libraries
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.InertialMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *

def measure_beam_deviation(repeat=4):
    # Initialize camera
    cam = camera_controller()
    pixelsize = 5.5e-6  # m
    wavelength = 532  # nm, example value
    laser_power = 100  # mW, example value
    camera_model = "Basler acA1300-60gm"  # Example camera model
    
    # Create directory for saving data
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    save_dir = f"beam_data_flipmirror_{current_time}"
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
        f.write(f"Repeats: {repeat}\n")
        f.write(f"Flips per Repeat: 10\n")
        f.write(f"Experiment: Mirror Flip Measurement with Per-Flip Data\n")
    
    deviations = []
    image_paths = []
    start_time = time.time()  # Record start time
    
    # CSV file setup
    csv_filename = os.path.join(save_dir, 'beam_deviations_after_movement_flipmirror.csv')
    with open(csv_filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Repeat', 'Flip', 'Elapsed Time (s)', 'dx1 (pixels)', 'dy1 (pixels)', 'dx2 (pixels)', 'dy2 (pixels)', 'dx1 (m)', 'dy1 (m)', 'dx2 (m)', 'dy2 (m)'])
    
        for i in range(repeat):
            # Capture initial beam positions
            flipmirror(1)
            current_time = time.time()
            elapsed_time = current_time - start_time
            img1 = cam.capture_image(1)
            img2 = cam.capture_image(2)
            x1, y1 = localize_beam_center(img1)
            x2, y2 = localize_beam_center(img2)
            
            # Save initial images with green dot
            img1_color = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
            cv2.circle(img1_color, (x1, y1), 5, (0, 255, 0), -1)
            img1_path = os.path.join(save_dir, f"initial_cam1_repeat_{i:02d}.png")
            cv2.imwrite(img1_path, img1_color)
            image_paths.append(img1_path)
            
            img2_color = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
            cv2.circle(img2_color, (x2, y2), 5, (0, 255, 0), -1)
            img2_path = os.path.join(save_dir, f"initial_cam2_repeat_{i:02d}.png")
            cv2.imwrite(img2_path, img2_color)
            image_paths.append(img2_path)
            
            # Flip mirror 10 times and measure after each flip
            flips = 10
            repeat_deviations = []
            for j in range(flips):
                flipmirror(2)
                flipmirror(1)
                current_time = time.time()
                elapsed_time_after = current_time - start_time
                img1 = cam.capture_image(1)
                img2 = cam.capture_image(2)
                x_moved1, y_moved1 = localize_beam_center(img1)
                x_moved2, y_moved2 = localize_beam_center(img2)
                
                # Save images after flip with green dot
                img1_color = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
                cv2.circle(img1_color, (x_moved1, y_moved1), 5, (0, 255, 0), -1)
                img1_moved_path = os.path.join(save_dir, f"cam1_repeat_{i:02d}_flip_{j:02d}.png")
                cv2.imwrite(img1_moved_path, img1_color)
                image_paths.append(img1_moved_path)
                
                img2_color = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
                cv2.circle(img2_color, (x_moved2, y_moved2), 5, (0, 255, 0), -1)
                img2_moved_path = os.path.join(save_dir, f"cam2_repeat_{i:02d}_flip_{j:02d}.png")
                cv2.imwrite(img2_moved_path, img2_color)
                image_paths.append(img2_moved_path)
                
                # Calculate deviation
                dx1 = x_moved1 - x1
                dy1 = y_moved1 - y1
                dx2 = x_moved2 - x2
                dy2 = y_moved2 - y2
                
                # Write to CSV
                writer.writerow([
                    i+1, j+1, elapsed_time_after,
                    dx1, dy1, dx2, dy2,
                    dx1 * pixelsize, dy1 * pixelsize, dx2 * pixelsize, dy2 * pixelsize
                ])
                
                # Store deviation for this flip
                repeat_deviations.append([dx1, dy1, dx2, dy2])
            
            # Store deviations for the last flip of this repeat (for compatibility with original return)
            deviations.append(repeat_deviations[-1])
    
    # Create video from saved images
    if image_paths:
        first_img = cv2.imread(image_paths[0])
        height, width = first_img.shape[:2]
        video_path = os.path.join(save_dir, "beam_deviation_flipmirror_video.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = 10
        video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
        
        for img_path in image_paths:
            img = cv2.imread(img_path)
            video_writer.write(img)
        
        video_writer.release()
        print(f"Video saved as {video_path}")
    
    return np