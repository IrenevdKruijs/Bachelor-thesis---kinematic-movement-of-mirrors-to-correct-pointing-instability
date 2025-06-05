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

# Initialize motor and camera
motor = PiezoMotor()
camera = camera_controller()

    # Create directory for saving data
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
save_dir = f"beam_data_backlash_{current_time}"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

pixelsize = 5.5e-6  # m
wavelength = 800  # nm, example value
laser_power = 6  # mW, example value
camera_model = "Basler acA2000-165umNIR"  # Example camera model

    # Save metadata to text file
metadata_path = os.path.join(save_dir, "metadata.txt")
with open(metadata_path, "w") as f:
    f.write(f"Date and Time: {current_time}\n")
    f.write(f"Wavelength: {wavelength} nm\n")
    f.write(f"Laser Power: {laser_power} mW\n")
    f.write(f"Camera Model: {camera_model}\n")
    f.write(f"Pixel Size: {pixelsize} m\n")
    f.write(f"Experiment: Backlash Measurement\n")

def check_backlash(channel, direction, steprange, output_file, repeats=3):
    """
    Measure backlash for a given channel and direction, repeated multiple times.
    Save results to a CSV file, images with green dots, video, and metadata in a dated folder.
    """
    if channel not in [1, 2, 3, 4]:
        raise ValueError("Channel must be 1, 2, 3, or 4")
    if direction not in [1, -1]:
        raise ValueError("Direction must be 1 or -1")
    # Experiment parameters
    dir_str = "up" if direction == 1 else "down"
    
    # Select camera
    cam = 2
    image_paths = []
    start_time = time.time()  # Record start time
    
    # Store results for all repeats
    all_results = []
    
    with open(output_file, 'w', newline='') as f:  # Use output_file directly
        writer = csv.writer(f)
        writer.writerow(['Repeat', 'Step', 'Elapsed Time (s)', 'End Pos (pixels)', 'Residue (pixels)', 'End Pos (m)', 'Residue (m)'])
        for rep in range(repeats):
            results = []
            for step in steprange:
                # Remove initial backlash
                init_step = 100 * direction
                
                if channel == 1:
                    motor.move_steps(-init_step,0,0,0,False)
                    motor.move_steps(init_step, 0, 0, 0, False)
                elif channel == 2:
                    motor.move_steps(0,-init_step,0,0,False)
                    motor.move_steps(0, init_step, 0, 0, False)
                elif channel == 3:
                    motor.move_steps(0,0,-init_step,0,False)
                    motor.move_steps(0, 0, init_step, 0, False)
                else:
                    motor.move_steps(0,0,0,-init_step,False)
                    motor.move_steps(0, 0, 0, init_step, False)
                
                # Define starting position
                current_time = time.time()
                elapsed_time = current_time - start_time
                img = camera.capture_image(cam)
                x0, y0 = localize_beam_center(img)
                
                # Save initial image with green dot
                img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                cv2.circle(img_color, (x0, y0), 5, (0, 255, 0), -1)
                img_path = os.path.join(save_dir, f"initial_repeat_{rep:02d}_step_{step}.png")
                cv2.imwrite(img_path, img_color)
                image_paths.append(img_path)
                
                # Move with step in steprange
                step = step * direction
                if channel == 1:
                    motor.move_steps(step, 0, 0, 0, False)
                elif channel == 2:
                    motor.move_steps(0, step, 0, 0, False)
                elif channel == 3:
                    motor.move_steps(0, 0, step, 0, False)
                else:
                    motor.move_steps(0, 0, 0, step, False)
                
                # Define ending position
                current_time = time.time()
                elapsed_time_forward = current_time - start_time
                img = camera.capture_image(cam)
                x1, y1 = localize_beam_center(img)
                
                # Save forward image with green dot
                img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                cv2.circle(img_color, (x1, y1), 5, (0, 255, 0), -1)
                img_path = os.path.join(save_dir, f"forward_repeat_{rep:02d}_step_{step}.png")
                cv2.imwrite(img_path, img_color)
                image_paths.append(img_path)
                
                # Move back
                if channel == 1:
                    motor.move_steps(int(-step), 0, 0, 0, False)
                elif channel == 2:
                    motor.move_steps(0, int(-step), 0, 0, False)
                elif channel == 3:
                    motor.move_steps(0, 0, int(-step), 0, False)
                else:
                    motor.move_steps(0, 0, 0, int(-step), False)
                
                # Define position after moving back
                current_time = time.time()
                elapsed_time_backward = current_time - start_time
                img = camera.capture_image(cam)
                x2, y2 = localize_beam_center(img)
                
                # Save backward image with green dot
                img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                cv2.circle(img_color, (x2, y2), 5, (0, 255, 0), -1)
                img_path = os.path.join(save_dir, f"backward_repeat_{rep:02d}_step_{step}.png")
                cv2.imwrite(img_path, img_color)
                image_paths.append(img_path)
                
                # Normalize x1 and x2 with starting position
                if channel in [1, 3]:
                    end_pos = x1 - x0
                    residue = x2 - x0
                else:  # channel 2 or 4
                    end_pos = y1 - y0
                    residue = y2 - y0
                
                # Save to results
                results.append((step, end_pos, residue))
                writer.writerow([
                    rep + 1, step, elapsed_time_backward,
                    end_pos, residue,
                    end_pos * pixelsize, residue * pixelsize
                ])
                print(f"Repeat {rep + 1}, Channel {channel}, Direction {dir_str}, Step: {step}, End Pos: {end_pos:.2f}, Residue: {residue:.2f}, Elapsed Time: {elapsed_time_backward:.2f} s")
                
                time.sleep(0.1)  # Small delay to stabilize hardware
            
            all_results.append(results)
    
    # Create video from saved images
    # if image_paths:
    #     first_img = cv2.imread(image_paths[0])
    #     height, width = first_img.shape[:2]
    #     video_path = os.path.join(save_dir, f"backlash_video_chan{channel}_{dir_str}.mp4")
    #     fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    #     fps = 10
    #     video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
        
    #     for img_path in image_paths:
    #         img = cv2.imread(img_path)
    #         video_writer.write(img)
        
    #     video_writer.release()
    #     print(f"Video saved as {video_path}")
    
        return all_results

def plot_backlash(input_file, channel, direction, residue_plot_file, movement_plot_file):
    """
    Plot mean backlash data (movement and residue) with error bars for standard deviation.
    Save plots in the same directory as input_file.
    """
    # Extract directory from input_file
    save_dir = os.path.dirname(input_file)
    residue_plot_file = os.path.join(save_dir, os.path.basename(residue_plot_file))
    movement_plot_file = os.path.join(save_dir, os.path.basename(movement_plot_file))
    
    if not os.path.exists(input_file):
        print(f"Error: Input file {input_file} does not exist.")
        return
    if not os.access(input_file, os.R_OK):
        print(f"Error: No read permission for {input_file}.")
        return

    # Read data
    steps = []
    repeats = []
    end_pos_values = []
    residue_values = []
    
    try:
        with open(input_file, 'r', newline='') as f:
            reader = csv.reader(f)
            next(reader)  # Skip header
            for row in reader:
                if len(row) != 7:
                    print(f"Warning: Skipping invalid row with {len(row)} columns: {row}")
                    continue
                rep, step, elapsed_time, end_pos, residue, end_pos_m, residue_m = map(float, row)
                repeats.append(int(rep))
                steps.append(step)
                end_pos_values.append(end_pos)
                residue_values.append(residue)
    except Exception as e:
        print(f"Error reading {input_file}: {e}")
        return

    if not steps:
        print(f"Error: No valid data found in {input_file}.")
        return

    # Organize data by step size
    unique_steps = sorted(set(steps))
    mean_end_pos = []
    std_end_pos = []
    mean_residue = []
    std_residue = []
    
    for step in unique_steps:
        step_indices = [i for i, s in enumerate(steps) if s == step]
        step_end_pos = [end_pos_values[i] for i in step_indices]
        step_residue = [residue_values[i] for i in step_indices]
        mean_end_pos.append(np.mean(step_end_pos))
        std_end_pos.append(np.std(step_end_pos))
        mean_residue.append(np.mean(step_residue))
        std_residue.append(np.std(step_residue))
    
    # Plot 1: Residue vs. Steps with error bars
    plt.figure(figsize=(8, 6))
    plt.errorbar(unique_steps, mean_residue, yerr=std_residue, fmt='r-o', label='Mean Residue', capsize=5)
    plt.xlabel('Step Size')
    plt.ylabel('Residue (pixels)')
    plt.title(f'Backlash Residue for Channel {channel}, Direction {direction}')
    plt.grid(True)
    plt.legend()
    plt.savefig(residue_plot_file)
    plt.close()
    print(f"Residue plot saved to {residue_plot_file}")
    
    plt.figure(figsize=(8, 6))
    plt.errorbar(unique_steps, mean_end_pos, yerr=std_end_pos, fmt='b-o', label='Mean Movement', capsize=5)
    plt.xlabel('Number of Steps')
    plt.ylabel('Mean Movement (pixels)')
    plt.title(f'Ending position for Channel {channel}, Direction {direction}')
    plt.grid(True)
    plt.legend()
    plt.savefig(movement_plot_file)
    plt.close()
    print(f"Ending positions plot saved to {movement_plot_file}")
    # Main execution
if __name__ == "__main__":
    steprange = range(200, 1000, 100)
    repeats = 3
    channels = [2]
    directions = [1, -1]
    
for channel in channels:
    for direction in directions:
        # Define output file names with save_dir
        dir_str = "up" if direction == 1 else "down"
        input_file = os.path.join(save_dir, f"backlash_data_chan{channel}_{dir_str}.csv")
        residue_plot_file = os.path.join(save_dir, f"residue_backlash_chan{channel}_{dir_str}.png")
        movement_plot_file = os.path.join(save_dir, f"movement_backlash_chan{channel}_{dir_str}.png")
        
        # Run backlash measurement
        results = check_backlash(channel, direction, steprange, input_file, repeats)
        plot_backlash(input_file,channel,direction,residue_plot_file,movement_plot_file)
        # Plot results
