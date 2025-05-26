import clr
from functions import *
import matplotlib as plt

# Initialize camera and motor libraries
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.InertialMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *

def test_backlash(start_pos, backlash_correction=False):
    """Test backlash from a given starting position with optional backlash correction."""
    # Initialize PiezoMotor and camera
    motor = PiezoMotor()
    cam = camera_controller()

    # Define initial position
    img = cam.capture_image()
    x0, y0 = localize_beam_center(img)

    # Move first channel to 200 and back to 0
    motor.move_steps(start_pos,backlash_correction)
    motor.move_steps(-start_pos,backlash_correction)

    # Define final position
    img = cam.capture_image()
    x1, y1 = localize_beam_center(img)

    # Calculate and print backlash
    dx = x1 - x0
    dy = y1 - y0
    print(f"Backlash correction: {backlash_correction}")
    print(f"dx = {dx}, dy = {dy}")
    return dx,dy

# Define starting positions
starting_positions = [
    (200,200,0,0)
    (-200, 200, 0, 0),
    (200,-200,0,0),
    (-200, -200, 0, 0)
]

# Collect data for plotting
no_correction_data = []
with_correction_data = []

# Test without and with backlash correction for each starting position
for pos in starting_positions:
    print(f"\nTesting from starting position {pos}")
    # Test without backlash correction
    dx, dy = test_backlash(pos, backlash_correction=False)
    no_correction_data.append((dx,dy))
    # Test with backlash correction
    dx, dy = test_backlash(pos, backlash_correction=True)
    with_correction_data.append((dx,dy))
    
# Plotting function
def plot_backlash(data, title, filename):
    plt.figure(figsize=(8, 8))
    plt.grid(True)
    
    # Plot normalized starting position (0, 0) as a black dot
    plt.scatter([0], [0], color='black', s=100, label='Start (0,0)')
    
    # Plot ending positions and lines to (0,0)
    colors = ['red', 'blue', 'green', 'purple', 'orange']  # Add more colors if needed
    for i, ((dx, dy)) in enumerate(data):
        # Plot ending position
        plt.scatter([dx], [dy], color=colors[i % len(colors)], s=50, label=f'End (Start: {starting_positions[i][:2]})')
        # Plot line from start to end
        plt.plot([0, dx],[0,dy], color=colors[i % len(colors)], linestyle='-', linewidth=0.5)
    
    plt.title(title)
    plt.xlabel('dx (pixels)')
    plt.ylabel('dy (pixels)')
    plt.legend()
    plt.axis('equal')  # Equal scaling for x and y axes
    plt.savefig(filename)
    plt.close()

# Create plots
plot_backlash(no_correction_data, 'Backlash Without Correction', 'backlash_no_correction.png')
plot_backlash(with_correction_data, 'Backlash With Correction', 'backlash_with_correction.png')