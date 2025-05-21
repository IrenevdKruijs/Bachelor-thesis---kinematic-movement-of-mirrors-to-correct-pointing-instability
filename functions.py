from pypylon import pylon
import numpy as np
import time
import clr
import os
import json
#Load piezo
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.InertialMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *
#Load data for flipmirror
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\ThorLabs.MotionControl.FilterFlipperCLI.dll")
from Thorlabs.MotionControl.FilterFlipperCLI import *
from System import Decimal, UInt32

class camera_controller:
    """
    This function initializes the camera, works for basler cameras 
    """
    def __init__(self,exposuretime):
        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()
        if not devices:
            raise Exception("No Basler cameras found")
        for device in devices:
            print(device.GetFriendlyName())
            
        # install instant camera
       
        self.camera = pylon.InstantCamera()
        self.camera.Attach(tl_factory.CreateFirstDevice())
        self.camera.ExposureTime.SetValue(exposuretime)
        

    def capture_image(self):  
        """
        This function makes an image using the camera initialized in camera_setup
        Args:
            camera: The input the function needs is which camera should be used, is the output of camera_setup

        Returns:
            image: the function returns the image with one frame 
        """
        try:
            if not self.camera.IsOpen():
                self.camera.Open()
            if self.camera.IsGrabbing():
                self.camera.StopGrabbing()
            self.camera.StartGrabbing(1)
            grab = self.camera.RetrieveResult(2000, pylon.TimeoutHandling_Return)
            if not grab.GrabSucceeded():
                raise Exception("failed to grab image")
            img = grab.GetArray()
            return img 
        finally:
            if self.camera.IsGrabbing():
                self.camera.StopGrabbing()
            self.camera.Close()

def localize_beam_center(inputimage):
    """
    TO DO: 
    - Investigate how to define the maximum input image and ensure that pixels with slightly lower intensity are also included -> otherwise, errors occur at higher wavelengths.
    
    This function takes the image of a beam on the camera and 
    returns the coordinates (x,y) of the middle pixel of the beam
    inputimage = the image of the beam
    """
    # Find max intensity and add margin for pixels that do not have the highest intensity but just below that
    max_intensity = np.max(inputimage)-25
    
    # Find all pixels with max intensity
    max_pixel_locs = np.where(inputimage >= max_intensity)
    max_pixel_coords = list(zip(max_pixel_locs[0], max_pixel_locs[1]))  # List of (y, x)
    
    if not max_pixel_coords:
        raise Exception(f"No pixels with maximum intensity found.")

    
    # Filter pixels that border other pixels with max intensity
    connected_pixel_coords = []
    height, width = inputimage.shape
    for y, x in max_pixel_coords:
        # Check 8-connectivity
        has_neighbor = False
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dy == 0 and dx == 0:
                    continue
                ny, nx = y + dy, x + dx
                # check if the neighbour is inside the image
                if 0 <= ny < height and 0 <= nx < width:
                    if inputimage[ny, nx] >= max_intensity:
                        has_neighbor = True
                        break
            if has_neighbor:
                break
        if has_neighbor:
            connected_pixel_coords.append((y, x))
    
    if not connected_pixel_coords:
        print(f"Geen verbonden pixels met maximale intensiteit gevonden.")
        raise  Exception("No connected pixels with max intensity found")  
    
    # calculate middle pixel and save
    y_coords, x_coords = zip(*connected_pixel_coords)
    middle_y = int(np.mean(y_coords))
    middle_x = int(np.mean(x_coords))
    
    return middle_x, middle_y

class PiezoMotor:
    def __init__(self,steprate,serial_number="97251304"):
        """
        function to move the piezomotors of the different mirrors using a Thorlabs KCube 
        input: new_pos_chan: the new positions the motors have to go to for 4 different channels
            steprate: the steprate at which the motor has to move
            camera: the camera that is used for taking the picture
        output: this function does not have an output
    

        """

        DeviceManagerCLI.BuildDeviceList()

        # create new device
        self.serial_number = str(serial_number)  # Serial number of device
        self.device = KCubeInertialMotor.CreateKCubeInertialMotor(self.serial_number)
        self.steprate = steprate
        self.last_movement = (0,0,0,0)
        self.current_position = (0,0,0,0)
        # Connect
        self.device.Connect(self.serial_number)
        time.sleep(0.25)

            # Ensure that the device settings have been initialized
        if not self.device.IsSettingsInitialized():
            self.device.WaitForSettingsInitialized(10000)  # 10 second timeout
            assert self.device.IsSettingsInitialized() is True

        print(f"Connected to: {self.device.GetDeviceInfo().Description}")
        
        # Start polling and enable channel
        self.device.StartPolling(250)  # 250ms polling rate
        time.sleep(0.25)
        self.device.EnableDevice()
        time.sleep(0.25)  # Wait for device to enable


            # Load any configuration settings needed by the controller/stage
        config = self.device.GetInertialMotorConfiguration(self.serial_number)
        settings = ThorlabsInertialMotorSettings.GetSettings(config)

            # Get parameters related to homing/zeroing/moving
            # Step parameters for an inertial motor channel
        self.chan1 = InertialMotorStatus.MotorChannels.Channel1
        self.chan2 = InertialMotorStatus.MotorChannels.Channel2
        self.chan3 = InertialMotorStatus.MotorChannels.Channel3
        self.chan4 = InertialMotorStatus.MotorChannels.Channel4
        
        for chan in [self.chan1, self.chan2, self.chan3, self.chan4]:
            settings.Drive.Channel(chan).StepRate = self.steprate
            settings.Drive.Channel(chan).StepAcceleration = 100000
        # Send settings to the device
        self.device.SetSettings(settings, True, True)

        # Home or Zero the device (if a motor/piezo)
        print("Zeroing device")
        self.device.SetPositionAs(self.chan1, 0)
        self.device.SetPositionAs(self.chan2, 0)
        self.device.SetPositionAs(self.chan3, 0)
        self.device.SetPositionAs(self.chan4, 0)
    
    def correct_backlash(self,step_chan1,step_chan2,step_chan3,step_chan4,backlash_steps = 100):
        """
        TO DO: also correct backlash when there is not yet a last movement (so with the first movement)
        Corrects backlash only when the direction of movement changes for each channel.
        
        Args:
            new_pos_chan1, new_pos_chan2, new_pos_chan3, new_pos_chan4: Target positions for channels 1-4.
            backlash_steps: Number of steps to overshoot for backlash correction (default: 100).
        """
        new_relative = [step_chan1, step_chan2, step_chan3, step_chan4]
        current_positions = list(self.current_position)
        last_moves = list(self.last_movement)
        correction_moves = [0, 0, 0, 0]
        return_moves = [0,0,0,0]

        for i in range(4):
            if new_relative[i] == 0:
                continue
            current_move = current_positions[i]+new_relative[i]
            last_move = last_moves[i]
            # Check if direction has changed (opposite signs and non-zero last move)
            if current_move * last_move < 0 and last_move != 0:
                # Direction reversed: apply backlash correction
                if current_move > current_positions[i]:
                    # Moving forward after backward: move backward first
                    correction_moves[i] = current_positions[i] - backlash_steps
                    return_moves[i]= correction_moves[i]+backlash_steps
                else:
                    # Moving backward after forward: move forward first
                    correction_moves[i] = current_positions[i] + backlash_steps
                    return_moves[i]=correction_moves[i]-backlash_steps
            else:
                # No direction change: move directly to target
                correction_moves[i] = 0
                return_moves[i]=0

        # Perform backlash correction if needed
        if any(c != 0 for c in correction_moves):
            self.move_steps(
                correction_moves[0], correction_moves[1], correction_moves[2], correction_moves[3]
            )
            # Move back to original positions (absolute)
            self.move_steps(
                return_moves[0], return_moves[1], return_moves[2], return_moves[3]
            )
            
    def move_steps(self,pos_chan1,pos_chan2,pos_chan3,pos_chan4):

        """
        Move the motor to the specified absolute positions for each channel and capture beam position.
        
        Args:
            pos_chan1, pos_chan2, pos_chan3, pos_chan4: Absolute positions for channels 1-4.
        
        Returns:
            middle_x, middle_y: Coordinates of the beam center after movement.
            
        """
                # Move to final target positions (absolute)
        #dit moet pas na opnieuw positie hebben gemeten dus ws in move_steps
        new_relative = [pos_chan1,pos_chan2,pos_chan3,pos_chan4]
        target_positions = [
            current_positions[i] + new_relative[i] if new_relative[i] != 0 else current_positions[i]
            for i in range(4)
        ]
        # self.move_steps(
        #     target_positions[0], target_positions[1], target_positions[2], target_positions[3]
        # )

        # Update movement history
        self.last_movement = tuple(new_relative)
        current_positions = list(self.current_position)
        # Input positions are absolute, derived from current_position + relative steps
        max_steps = 0

        try:
            if pos_chan1 != current_positions[0]:
                self.device.MoveTo(self.chan1, int(target_positions[0]), 6000)
                max_steps = max(max_steps, abs(pos_chan1 - current_positions[0]))
            
            if pos_chan2 != current_positions[1]:
                self.device.MoveTo(self.chan2, int(target_positions[1]), 6000)
                max_steps = max(max_steps, abs(pos_chan2 - current_positions[1]))

            if pos_chan3 != current_positions[2]:
                self.device.MoveTo(self.chan3, int(target_positions[2]), 6000)
                max_steps = max(max_steps, abs(pos_chan3 - current_positions[2]))
            
            if pos_chan4 != current_positions[3]:
                self.device.MoveTo(self.chan4, int(pos_chan4,target_positions[3]), 6000)
                max_steps = max(max_steps, abs(pos_chan4 - current_positions[3]))
        except Exception as e:
            raise RuntimeError(f"Failed to move motor: {e}")

        # Update current position
        self.current_position = tuple(target_positions)

        # Dynamic wait time based on steps moved
        wait_time = max_steps / self.steprate + 0.5 if max_steps > 0 else 1.0
        time.sleep(wait_time)

        
    def shutdown(self):
            # Stop Polling and Disconnect
            self.device.StopPolling()
            self.device.Disconnect()
        # Extract parameters from options with default values

def calibrate_mirror1_2D(amount_steps, stepsize, repeats,steprate):
    
    all_shifts = []
    motor = PiezoMotor(steprate=steprate, serial_number="97251304",)
    cam = camera_controller()
    for h in range(repeats):
        motor.correct_backlash(100,100,0,0)
        img = cam.capture_image()
        x0, y0 = localize_beam_center(img)
        current_step = 0   #initialize stap for correct data savings
        shifts = []     #initialize shifts to save the shifts
        for _ in range(amount_steps + 1):  # endpoint included
            current_step += stepsize 
            motor.move_steps(stepsize, stepsize, 0, 0,steprate)
            img = cam.capture_image()
            x,y=localize_beam_center()
            dx = x - x0
            dy = y - y0
            shifts.append((current_step, dx, dy))
            print(f"Steps: {current_step} | Δx = {dx}, Δy = {dy}")
            # Save shifts of this repeat
            with open(f"kalibratie_spiegel1_2D_herhaling_{h+1}.txt", "w") as f:
                f.write("Motorstap\tDeltaX_pixels\tDeltaY_pixels\n")
                for current_step, dx, dy in shifts:
                    f.write(f"{current_step}\t{dx}\t{dy}\n")
        all_shifts.append(shifts)
        motor.move_steps(
        (-(amount_steps*stepsize)),
        (-(amount_steps*stepsize)),
        0, 0, steprate
        )
    return all_shifts     


def get_cached_calibration(amount_steps, stepsize, repeats, steprate, cache_file="calibration_cache.json"):
    """
    Checks if calibration data is cached. If not, performs calibration and stores result.
    Returns calibration data as a list of (step, dx, dy) tuples.
    """
    if os.path.exists(cache_file):
        with open(cache_file, 'r') as f:
            cache = json.load(f)
    else:
        cache = {}

    key = f"{amount_steps}_{stepsize}_{repeats}_{steprate}"

    if key in cache:
        print(f"[CACHE] Using cached calibration for steprate = {steprate}")
        return cache[key]
    else:
        print(f"[CALIBRATION] Performing calibration for steprate = {steprate}")
        calibration_data = calibrate_mirror1_2D(amount_steps, stepsize, repeats, steprate)
        
        # Convert to JSON-safe format
        serializable_data = [
            [[int(s), float(dx), float(dy)] for s, dx, dy in run]
            for run in calibration_data
        ]
        cache[key] = serializable_data

        with open(cache_file, 'w') as f:
            json.dump(cache, f)

        return serializable_data    

import clr
import time


def flipmirror(position):
    # Uncomment this line if you are using a simulation
    #SimulationManager.Instance.InitializeSimulations()

    try:
        # Build device list. 
        DeviceManagerCLI.BuildDeviceList()

        # create new device.
        serial_no = "37006200"
        device = FilterFlipper.CreateFilterFlipper(serial_no)

        # Connect to device.
        device.Connect(serial_no)

        # Ensure that the device settings have been initialized.
        if not device.IsSettingsInitialized():
            device.WaitForSettingsInitialized(10000)  # 10 second timeout.
            assert device.IsSettingsInitialized() is True

        # Start polling loop and enable device.
        device.StartPolling(250)  #250ms polling rate.
        time.sleep(0.25)
        device.EnableDevice()
        time.sleep(0.25)  # Wait for device to enable.

        # Get Device Information and display description.
        device_info = device.GetDeviceInfo()
        print(device_info.Description)

        time.sleep(2)
        new_pos = UInt32(position)  # Must be a .NET decimal.
        print(f'Moving to {new_pos}')
        device.SetPosition(new_pos,6000)  # 10 second timeout.
        print("Done")


        # Stop polling loop and disconnect device before program finishes. 
        device.StopPolling()
        device.Disconnect()


    except Exception as e:
        print(e)
        






 

   
