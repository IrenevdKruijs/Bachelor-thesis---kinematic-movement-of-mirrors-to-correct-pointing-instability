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
    wavelength = 800 # nm
    laser_power = 6  # %, 
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
        writer.writerow(['Repeat', 'Flip', 'Elapsed Time (s)', 'dx1 (pixels)', 'dy1 (pixels)', 'dx2 (pixels)', 'dy2 (pixels)'])
    
        for i in range(repeat):
            # Capture initial beam positions
            flipmirror(1)
            current_time = time.time()
            elapsed_time = current_time - start_time
            initial_img1 = cam.capture_image(1)
            initial_img2 = cam.capture_image(2)
            
            img1_path = os.path.join(save_dir, f"initial_cam1_repeat_{i:02d}.png")
            cv2.imwrite(img1_path)
            image_paths.append(img1_path)

            img2_path = os.path.join(save_dir, f"initial_cam2_repeat_{i:02d}.png")
            cv2.imwrite(img2_path)
            image_paths.append(img2_path)
            
            # Flip mirror 10 times and measure after each flip
            flips = 10
            repeat_deviations = []
            for j in range(flips):
                flipmirror(2)
                flipmirror(1)
                img1 = cam.capture_image(1)
                img2 = cam.capture_image(2)
                dx1, dy1 = localize_beam_center(initial_img1, img1)
                dx2, dy2 = localize_beam_center(initial_img2,img2)
                
                # Save images after flip with green dot
                img1_moved_path = os.path.join(save_dir, f"cam1_repeat_{i:02d}_flip_{j:02d}.png")
                cv2.imwrite(img1_moved_path)
                image_paths.append(img1_moved_path)
                
                img2_moved_path = os.path.join(save_dir, f"cam2_repeat_{i:02d}_flip_{j:02d}.png")
                cv2.imwrite(img2_moved_path)
                image_paths.append(img2_moved_path)
                
                # Write to CSV
                writer.writerow([
                    i+1, j+1,
                    dx1, dy1, dx2, dy2
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

measure_beam_deviation(4)