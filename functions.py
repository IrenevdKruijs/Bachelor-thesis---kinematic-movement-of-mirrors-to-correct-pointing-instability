from pypylon import pylon
import numpy as np
import time
import clr
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

def camera_setup():
    """
    This function initializes the camera, works for basler camera's 
    """
    tl_factory = pylon.TlFactory.GetInstance()
    devices = tl_factory.EnumerateDevices()
    for device in devices:
        print(device.GetFriendlyName())
        
    # install instant camera
    tl_factory = pylon.TlFactory.GetInstance()
    camera = pylon.InstantCamera()
    camera.Attach(tl_factory.CreateFirstDevice())
    return camera

def image(camera):  
    """
    This function makes an image using the camera initialized in camera_setup
    Args:
        camera: The input the function needs is which camera should be used, is the output of camera_setup

    Returns:
        image: the function returns the image with one frame 
    """
    try:
        if not camera.IsOpen():
            camera.Open()
        camera.ExposureTime.SetValue(5000)
        if camera.IsGrabbing():
            camera.StopGrabbing()
        camera.StartGrabbing(1)
        grab = camera.RetrieveResult(2000, pylon.TimeoutHandling_Return)
        if grab.GrabSucceeded():
            img = grab.GetArray()
            return img 
        else: 
            print("failed to grab image")
            return None
    except Exception as e:
        print(f'Error in image capture: {e}')
        return None
    finally:
        if camera.IsGrabbing():
            camera.StopGrabbing()
        camera.Close()

def coordinates(inputimage):
    """
    TO DO: 
    - Investigate how to define the maximum input image and ensure that pixels with slightly lower intensity are also included -> otherwise, errors occur at higher wavelengths.
    
    This function takes the image of a beam on the camera and 
    returns the coordinates (x,y) of the middle pixel of the beam
    inputimage = the image of the beam
    kijken hoe max input image definieren en zorgen dat pixels met iets lagere intensiteit ook worden meegenomen-> anders error bij hogere golflengtes
    """
    # Find max intensity and add margin for pixels that do not have the highest intensity but just below that
    max_intensity = np.max(inputimage)-25
    
    # Find all pixels with max intensity
    max_pixel_locs = np.where(inputimage >= max_intensity)
    max_pixel_coords = list(zip(max_pixel_locs[0], max_pixel_locs[1]))  # List of (y, x)
    
    if not max_pixel_coords:
        print(f"No pixels with maximum intensity found.")
        return None
    
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
                    if inputimage[ny, nx] == max_intensity:
                        has_neighbor = True
                        break
            if has_neighbor:
                break
        if has_neighbor:
            connected_pixel_coords.append((y, x))
    
    if not connected_pixel_coords:
        print(f"Geen verbonden pixels met maximale intensiteit gevonden.")
        return None
    
    # calculate middle pixel and save
    y_coords, x_coords = zip(*connected_pixel_coords)
    middle_y = int(np.mean(y_coords))
    middle_x = int(np.mean(x_coords))
    
    return middle_x, middle_y

def piezomotor(new_pos_chan1, new_pos_chan2, new_pos_chan3, new_pos_chan4,steprate,camera):
    """
    TO DO: 
    - Do I want the image and coordinates function to be included in this function or to be separate? 
        I think separate is better so that the function requires as little information as possible.
    - adjust time.sleep for different step sizes so you are not waiting unneceserry long
    
    function to move the piezomotors of the different mirrors using a Thorlabs KCube 
    input: new_pos_chan: the new positions the motors have to go to for 4 different channels
           steprate: the steprate at which the motor has to move
           camera: the camera that is used for taking the picture
    output: middle_x, middle_y: the new coordinates of the middle of the laser beam after movement

    """
    try:
        DeviceManagerCLI.BuildDeviceList()

        # create new device
        serial_no = "97251304"  # Serial number of device
        device = KCubeInertialMotor.CreateKCubeInertialMotor(serial_no)

        # Connect
        device.Connect(serial_no)
        time.sleep(0.25)

        # Ensure that the device settings have been initialized
        if not device.IsSettingsInitialized():
            device.WaitForSettingsInitialized(10000)  # 10 second timeout
            assert device.IsSettingsInitialized() is True

        # Get Device Information and display description
        device_info = device.GetDeviceInfo()
        print(device_info.Description)
        # Start polling and enable channel
        device.StartPolling(250)  # 250ms polling rate
        time.sleep(0.25)
        device.EnableDevice()
        time.sleep(0.25)  # Wait for device to enable

        # Load any configuration settings needed by the controller/stage
        inertial_motor_config = device.GetInertialMotorConfiguration(serial_no)

        # Get parameters related to homing/zeroing/moving
        device_settings = ThorlabsInertialMotorSettings.GetSettings(inertial_motor_config)

        # Step parameters for an inertial motor channel
        chan1 = InertialMotorStatus.MotorChannels.Channel1
        chan2 = InertialMotorStatus.MotorChannels.Channel2
        chan3 = InertialMotorStatus.MotorChannels.Channel3
        chan4 = InertialMotorStatus.MotorChannels.Channel4
        device_settings.Drive.Channel(chan1).StepRate = steprate
        device_settings.Drive.Channel(chan1).StepAcceleration = 100000
        device_settings.Drive.Channel(chan2).StepRate = steprate
        device_settings.Drive.Channel(chan2).StepAcceleration = 100000
        # Send settings to the device
        device.SetSettings(device_settings, True, True)

        # Home or Zero the device (if a motor/piezo)
        print("Zeroing device")
        device.SetPositionAs(chan1, 0)
        device.SetPositionAs(chan2, 0)
        device.SetPositionAs(chan3, 0)
        device.SetPositionAs(chan4, 0)

        # Move the device to a new position
        channel = chan1
        if new_pos_chan1 != 0:
            device.MoveTo(channel, int(new_pos_chan1), 6000)  # 60 second timeout
        
        channel = chan2
        if new_pos_chan2 != 0:
            device.MoveTo(channel, int(new_pos_chan2), 6000)  # 60 second timeout

        channel = chan3
        if new_pos_chan3 != 0:
            device.MoveTo(channel, int(new_pos_chan3), 6000)  # 3 second timeout
        
        channel = chan4
        if new_pos_chan4 != 0:
            device.MoveTo(channel, int(new_pos_chan4), 6000)  # 3 second timeout
        
        img = image(camera)
        middle_x, middle_y = coordinates(img)
        if steprate <= 200:
            time.sleep(10)
        else: 
            time.sleep(5)

        # Stop Polling and Disconnect
        device.StopPolling()
        device.Disconnect()
        return middle_x, middle_y
    except Exception as e:
        print(e)
        return None, None
    
    
    # Extract parameters from options with default values

def calibrate_mirror1_2D(amount_steps, stepsize, repeats,steprate,camera):
    
    all_shifts = []
        
    for h in range(repeats):
        piezomotor(-100, -100, 0, 0,steprate,camera)  # backlash compensation
        x0, y0 = piezomotor(100, 100, 0, 0,steprate,camera)
        print(f"Startpositie (pixels): {x0}, {y0}")
        stap = stepsize   #initialize stap for correct data savings
        shifts = []     #initialize verschuivingen to save the shifts
        for _ in range(amount_steps + 1):  # inclusief eindpunt
            huidig_stap = stepsize 
            x,y= piezomotor(huidig_stap, huidig_stap, 0, 0,steprate,camera)
            dx = x - x0
            dy = y - y0
            shifts.append((stap, dx, dy))
            print(f"Stappen: {stap} | Δx = {dx}, Δy = {dy}")
            # Opslaan van de verschuivingen voor deze herhaling
            with open(f"kalibratie_spiegel1_2D_herhaling_{h+1}.txt", "w") as f:
                f.write("Motorstap\tDeltaX_pixels\tDeltaY_pixels\n")
                for stap, dx, dy in shifts:
                    f.write(f"{stap}\t{dx}\t{dy}\n")
            stap += stepsize 
        all_shifts.append(shifts)
        piezomotor(
        (-(stepsize * amount_steps) ),
        (-(stepsize * amount_steps)),
        0, 0, steprate,camera
        )
    return all_shifts     

import os
import json
from functions import calibrate_mirror1_2D  # Adjust if it's from a different module

def get_cached_calibration(amount_steps, stepsize, repeats, steprate, camera, cache_file="calibration_cache.json"):
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
        calibration_data = calibrate_mirror1_2D(amount_steps, stepsize, repeats, steprate, camera)
        
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
        






 

   