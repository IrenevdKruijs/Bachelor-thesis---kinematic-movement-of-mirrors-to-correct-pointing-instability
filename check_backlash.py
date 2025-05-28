from functions import *
import clr
import matplotlib.pyplot as plt
import csv

# Initialize camera and motor libraries
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.InertialMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *

motor = PiezoMotor()
camera = camera_controller()

def check_backlash(channel,direction,steprange,output_file):
    results = []
    if channel == 1 or channel ==2: 
        cam =1
    else: cam = 2
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['step', 'x1', 'x2'])  # Header
        
        if channel not in [1, 2, 3, 4]:
            raise ValueError("Channel must be 1, 2, 3, or 4")
        if direction not in [1, -1]:
            raise ValueError("Direction must be 1 or -1")
        
        for step in steprange:
            #remove initial backlash
            init_step = 100*direction
            if channel ==1: 
                motor.move_steps(init_step,0,0,0,False)
            elif channel ==2: 
                motor.move_steps(0,init_step,0,0,False)
            elif channel == 3: 
                motor.move_steps(0,0,init_step,0,False)
            else: motor.move_steps(0,0,0,init_step,False)
            
            # define starting position
            img = camera.capture_image(cam)
            x0,y0 = localize_beam_center(img)
            print(x0)
            #move with step in steprange
            step = step*direction
            print(f"step={step}")
            if channel ==1: 
                motor.move_steps(step,0,0,0,False)
            elif channel ==2:
                motor.move_steps(0,step,0,0,False)
            elif channel == 3: 
                motor.move_steps(0,0,step,0,False)
            else: 
                motor.move_steps(0,0,0,step,False)  
            # define ending position
            img = camera.capture_image(cam)
            plt.imshow(img)
            plt.show
            x1,y1 = localize_beam_center(img)
            print(f"x1 = {x1}")
            time.sleep(0.1)
            
            if channel ==1: 
                motor.move_steps(int(-1*step),0,0,0,False)
            elif channel ==2: 
                motor.move_steps(0,int(-1*step),0,0,False)
            elif channel == 3: 
                motor.move_steps(0,0,int(-1*step),0,False)
            else: 
                motor.move_steps(0,0,0,-step,False)  
            img = camera.capture_image(cam)
            x2,y2 = localize_beam_center(img)
            print(f"x2 = {x2}")
            #normalize x1 and x2 with starting position
            if channel ==1 or channel ==3:
                end_pos = x1-x0
                residue = x2-x0
            elif channel ==2 or channel ==4:
                end_pos = y1-y0
                residue = y2-y0
            # Save to results
            results.append((step, end_pos, residue))
            writer.writerow([step, end_pos, residue])
            print(f"Step: {step}, Endposition: {end_pos}, residue: {residue}")
        return results

import csv
import matplotlib.pyplot as plt
import os

def plot_backlash(input_file, channel, residue_plot_file="backlash_residue.png", movement_plot_file="backlash_movement.png"):
    """
    Plot backlash data: residue (x2) and normalized x1 vs. steps.
    """
    if not os.path.exists(input_file):
        print(f"Error: Input file {input_file} does not exist.")
        return
    if not os.access(input_file, os.R_OK):
        print(f"Error: No read permission for {input_file}.")
        return

    steps = []
    x1_values = []
    x2_values = []

    try:
        with open(input_file, 'r', newline='') as f:
            reader = csv.reader(f)
            next(reader)  # Skip header
            for row in reader:
                if len(row) != 3:
                    print(f"Warning: Skipping invalid row with {len(row)} columns: {row}")
                    continue
                step, x1, residue = map(float, row)  # Expect three columns
                steps.append(step)
                x1_values.append(x1)
                x2_values.append(residue)
    except Exception as e:
        print(f"Error reading {input_file}: {e}")
        return

    if not steps:
        print(f"Error: No valid data found in {input_file}.")
        return

    # Plot 1: Residue (x2) vs. Steps
    plt.figure(figsize=(8, 6))
    plt.plot(steps, x2_values, 'r-o', label='Residue')
    plt.xlabel('Step Size')
    plt.ylabel('Residue (pixels)')
    plt.title(f'Backlash Residue for Channel {channel}')
    plt.grid(True)
    plt.legend()
    plt.savefig(residue_plot_file)
    plt.close()
    print(f"Residue plot saved to {residue_plot_file}")

    # Plot 2: Normalized x1 vs. Steps
    plt.figure(figsize=(8, 6))
    plt.plot(steps, x1_values, 'b-o', label='Movement')
    plt.xlabel('Step Size')
    plt.ylabel('Normalized Movement (pixels)')
    plt.title(f'Backlash Movement for Channel {channel}')
    plt.grid(True)
    plt.legend()
    plt.savefig(movement_plot_file)
    plt.close()
    print(f"Movement plot saved to {movement_plot_file}")
    
steps = range(200,1000,100)
input_file = "backlash_data_chan4_down.csv"
output_file_residue = "residue_backlash_chan4_down.png"
output_file_steps = "movement_per_steps_chan4_down.png"
check_backlash(4,-1,steps,input_file)
plot_backlash(input_file,4,output_file_residue,output_file_steps)