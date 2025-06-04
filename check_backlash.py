from functions import *
import clr
import matplotlib.pyplot as plt
import csv
import numpy as np
import os
import time

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

def check_backlash(channel, direction, steprange, output_file, repeats=3):
    """
    Measure backlash for a given channel and direction, repeated multiple times.
    Save results to a CSV file and return the data.
    """
    if channel not in [1, 2, 3, 4]:
        raise ValueError("Channel must be 1, 2, 3, or 4")
    if direction not in [1, -1]:
        raise ValueError("Direction must be 1 or -1")
    
    # Select camera based on channel
    cam = 1 if channel in [1, 2] else 2
    
    # Store results for all repeats
    all_results = []
    
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['repeat', 'step', 'end_pos', 'residue'])  # Updated header
        
        for rep in range(repeats):
            results = []
            for step in steprange:
                # Remove initial backlash
                init_step = 100 * direction
                if channel == 1:
                    motor.move_steps(init_step, 0, 0, 0, False)
                elif channel == 2:
                    motor.move_steps(0, init_step, 0, 0, False)
                elif channel == 3:
                    motor.move_steps(0, 0, init_step, 0, False)
                else:
                    motor.move_steps(0, 0, 0, init_step, False)
                
                # Define starting position
                img = camera.capture_image(cam)
                x0, y0 = localize_beam_center(img)
                
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
                img = camera.capture_image(cam)
                x1, y1 = localize_beam_center(img)
                
                # Move back
                if channel == 1:
                    motor.move_steps(int(-step), 0, 0, 0, False)
                elif channel == 2:
                    motor.move_steps(0, int(-step), 0, 0, False)
                elif channel == 3:
                    motor.move_steps(0, 0, int(-step), 0, False)
                else:
                    motor.move_steps(0, 0, 0, int(-step), False)
                
                img = camera.capture_image(cam)
                x2, y2 = localize_beam_center(img)
                
                # Normalize x1 and x2 with starting position
                if channel in [1, 3]:
                    end_pos = x1 - x0
                    residue = x2 - x0
                else:  # channel 2 or 4
                    end_pos = y1 - y0
                    residue = y2 - y0
                
                # Save to results
                results.append((step, end_pos, residue))
                writer.writerow([rep + 1, step, end_pos, residue])
                print(f"Repeat {rep + 1}, Channel {channel}, Direction {direction}, Step: {step}, End Pos: {end_pos:.2f}, Residue: {residue:.2f}")
                
                time.sleep(0.1)  # Small delay to stabilize hardware
            
            all_results.append(results)
    
    return all_results

def plot_backlash(input_file, channel, direction, residue_plot_file, movement_plot_file):
    """
    Plot mean backlash data (movement and residue) with error bars for standard deviation.
    """
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
                if len(row) != 4:
                    print(f"Warning: Skipping invalid row with {len(row)} columns: {row}")
                    continue
                rep, step, end_pos, residue = map(float, row)
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

    # Plot 2: Normalized Movement vs. Steps with error bars
    plt.figure(figsize=(8, 6))
    plt.errorbar(unique_steps, mean_end_pos, yerr=std_end_pos, fmt='b-o', label='Mean Movement', capsize=5)
    plt.xlabel('Step Size')
    plt.ylabel('Normalized Movement (pixels)')
    plt.title(f'Backlash Movement for Channel {channel}, Direction {direction}')
    plt.grid(True)
    plt.legend()
    plt.savefig(movement_plot_file)
    plt.close()
    print(f"Movement plot saved to {movement_plot_file}")

# Main execution
if __name__ == "__main__":
    steprange = range(200, 1000, 100)
    repeats = 3
    channels = [1, 2, 3, 4]
    directions = [1, -1]
    
    for channel in channels:
        for direction in directions:
            # Define output file names
            dir_str = "up" if direction == 1 else "down"
            input_file = f"backlash_data_chan{channel}_{dir_str}.csv"
            residue_plot_file = f"residue_backlash_chan{channel}_{dir_str}.png"
            movement_plot_file = f"movement_backlash_chan{channel}_{dir_str}.png"
            
            # Run backlash measurement
            results = check_backlash(channel, direction, steprange, input_file, repeats)
            
            # Plot results
            plot_backlash(input_file, channel, direction, residue_plot_file, movement_plot_file)