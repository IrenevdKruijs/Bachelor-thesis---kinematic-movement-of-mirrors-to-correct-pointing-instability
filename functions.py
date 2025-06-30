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
    Args: wavelength - wavelength that the laser is used on
          serial_number_cam2 - serial number of camera 2, so the correct camera is selected
          seral_number_cam1 - serial number of camera 1, so the correct camera is selected
          camera 1 is the camera closest to the beam splitter.
    """
    def __init__(self,wavelength,serial_number_cam2=22357092,serial_number_cam1=23572269):
        #initialize cameras
        
        #select correct exposure time for wavelength of the laser that is being used. When using a different laser and/or cameras, find the exposure time for these wavelengths
        # at which the beam is not overexposed and change the following values accordingly:
        wavelength_exposure = {
            730: 100,
            800: 42,
            850: 60,
            900: 125,
            950: 235,
            1000: 800,
            1050: 3443,
            1100: 13000
        }
        if wavelength not in wavelength_exposure:
            raise ValueError(f"Unsupported wavelength: {wavelength}. Please enter 800, 850, 900, 950, 1000, 1050, or 1100.")
        
        exposuretime = wavelength_exposure[wavelength]

        tl_factory = pylon.TlFactory.GetInstance()
        self.devices = tl_factory.EnumerateDevices()
        if not self.devices:
            raise Exception("No Basler cameras found")
        
        for device in self.devices:
            print(f"{device.GetFriendlyName()}")

        # Create and attach the camera
        camera = pylon.InstantCameraArray(len(self.devices))
        self.cameras = []
        for i in range(len(self.devices)):
            camera = pylon.InstantCamera()  # Create a single InstantCamera
            camera.Attach(tl_factory.CreateDevice(self.devices[i]))
            camera.Open()
            camera_serial_number = camera.GetDeviceInfo().GetSerialNumber()
            if camera_serial_number == str(serial_number_cam2):
                print(f"Using device with serial number: {camera_serial_number}")
                self.cam2 = camera
            elif camera_serial_number == str(serial_number_cam1):
                print(f"Using device with serial number: {camera_serial_number}")
                self.cam1 = camera
            else:
                camera.Close()  # Close camera if serial number doesn't match
                raise Exception("Device with unknown serial number connected. Change the serial numbers or connect the correct cameras")
            self.cameras.append(camera)  # Store camera in list 
        self.cam1.ExposureTime.SetValue(exposuretime)
        self.cam2.ExposureTime.SetValue(exposuretime)
        self.cam1.Close()
        self.cam2.Close()
            
            
    def capture_image(self,camera_choice):  
        """
        This function makes an image using the camera initialized in camera_setup
        Args:
            camera_choice: The input the function needs is which camera should be used, put in 1 for camera 1 and 2 for camera 2.

        Returns:
            image: the function returns the image with one frame 
        """
        #select the correct camera that needs to be used for imaging
        camera = self.cam1 if camera_choice == 1 else self.cam2
        
        #take the image 
        try:
            if not camera.IsOpen():
                camera.Open()
            if camera.IsGrabbing():
                camera.StopGrabbing()
            camera.StartGrabbing()
            grab = camera.RetrieveResult(2000, pylon.TimeoutHandling_Return)
            if not grab.GrabSucceeded():
                raise Exception("failed to grab image")
            img = grab.GetArray()
            return img 
        finally:
            if camera.IsGrabbing():
                camera.StopGrabbing()
            camera.Close()
            
import os
from PIL import Image
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Use Tkinter to avoid Qt/COM issues
import matplotlib.pyplot as plt
from scipy import signal

def localize_beam_center(initial_image, new_image):
    """
    Calculate the beam center shift between two images using cross-correlation.
    Converts input images to grayscale before analysis. The shift of new_image is calculated 
    relative to the initial_image

    Args:
        initial_image: Input image (PIL Image or NumPy array, color or grayscale).
        new_image: Input image (PIL Image or NumPy array, color or grayscale).

    Returns:
        tuple: (correlated_image, shift_x, shift_y) where correlated_image is the
               cross-correlation result, and shift_x, shift_y are the pixel shifts.
    
    Raises:
        ValueError: If inputs are invalid or cannot be converted to grayscale.
    """
    print("Calculating beam center...")

    # Helper function to convert any image to grayscale NumPy array
    def to_grayscale(img):
        if isinstance(img, Image.Image):
            # Convert PIL image to grayscale
            return np.array(img.convert('L'), dtype=np.float32)
        elif isinstance(img, np.ndarray):
            # Handle NumPy arrays
            if len(img.shape) == 3 and img.shape[2] in [3, 4]:  # Color image (RGB/BGR/RGBA)
                # Convert to grayscale
                import cv2
                if img.shape[2] == 4:  # Handle RGBA
                    img = img[:, :, :3]
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                return gray.astype(np.float32)
            elif len(img.shape) == 2:  # Already grayscale
                return img.astype(np.float32)
            else:
                raise ValueError(f"Invalid NumPy array shape: {img.shape}. Expected 2D or 3D with 3/4 channels.")
        else:
            raise ValueError(f"Unsupported image type: {type(img)}. Expected PIL Image or NumPy array.")

    # Convert both images to grayscale
    try:
        initial_image = to_grayscale(initial_image)
        new_image = to_grayscale(new_image)
    except Exception as e:
        raise ValueError(f"Error converting images to grayscale: {e}")

    # Verify input shapes
    print(f"Initial image shape: {initial_image.shape}")
    print(f"New image shape: {new_image.shape}")

    # Ensure inputs are 2D (grayscale)
    if len(initial_image.shape) != 2 or len(new_image.shape) != 2:
        raise ValueError("Converted images must be grayscale (2D arrays)")

    # Compute cross-correlation
    correlated_image = signal.correlate(initial_image, new_image, method="fft")
    print(f"Correlated image shape: {correlated_image.shape}, type: {type(correlated_image)}")

    # Find the coordinates of the maximum value
    max_idx = np.argmax(correlated_image)
    middle_y, middle_x = np.unravel_index(max_idx, correlated_image.shape)
    print(f"Beam shift found at (x, y): ({middle_x}, {middle_y})")

    # Adjust coordinates to get the relative shift
    corr_height, corr_width = correlated_image.shape
    center_y, center_x = corr_height // 2, corr_width // 2
    shift_x = middle_x - center_x
    shift_y = middle_y - center_y
    print(f"Raw max coordinates: (x, y) = ({middle_x}, {middle_y})")
    print(f"Beam shift (pixels): (x, y) = ({shift_x}, {shift_y})")

    return correlated_image, shift_x, shift_y  # Return image and coordinates
    
class PiezoMotor:
    def __init__(self,serial_number="97251304"):
        """
        function to initialize the piezomotor that steers two motorized mirrors using a Thorlabs KCube 
        Args: serial number of piezomotor
        """

        DeviceManagerCLI.BuildDeviceList()

        # create new device
        self.serial_number = str(serial_number)  # Serial number of device
        self.device = KCubeInertialMotor.CreateKCubeInertialMotor(self.serial_number)
        self.last_movement = (0,0,0,0)
        self.current_position = (0,0,0,0)
        self.steprate = 500
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

        # Zero the device 
        print("Zeroing device")
        self.device.SetPositionAs(self.chan1, 0)
        self.device.SetPositionAs(self.chan2, 0)
        self.device.SetPositionAs(self.chan3, 0)
        self.device.SetPositionAs(self.chan4, 0)
            
    def move_steps(self,pos_chan1,pos_chan2,pos_chan3,pos_chan4):

        """
        Move the motor to the specified absolute positions for each channel.
        
        Args:
            pos_chan1, pos_chan2, pos_chan3, pos_chan4: Absolute positions for channels 1-4.
        

        Raises:
            RunTimeError: when the motor did not succeed in moving on time
        """
        # Update movement history
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
        
        new_relative = [pos_chan1,pos_chan2,pos_chan3,pos_chan4]
        self.last_movement = tuple(new_relative)
        current_positions = list(self.current_position)
        
        # Move to final target positions (absolute)
        target_positions = [
            current_positions[i] + new_relative[i] if new_relative[i] != 0 else current_positions[i]
            for i in range(4)
        ]
        
        # Input positions are absolute, derived from current_position + relative steps
        max_steps = 0

        try:
                if pos_chan1!= current_positions[0]:
                    self.device.MoveTo(self.chan1, target_positions[0], 100000)
                    max_steps = max(max_steps, abs(pos_chan1 - current_positions[0]))

                    # Dynamic wait time based on steps moved
                    wait_time = max_steps / self.steprate + 0.5 if max_steps > 0 else 1.0
                    time.sleep(wait_time)
                else:
                    print("continuing measurement")
                
                
                if pos_chan2!=current_positions[1]:
                    self.device.MoveTo(self.chan2, target_positions[1], 100000)
                    max_steps = max(max_steps, abs(pos_chan2))

                    # Dynamic wait time based on steps moved
                    wait_time = max_steps/self.steprate + 0.5 if max_steps > 0 else 1.0
                    time.sleep(wait_time)
                else: 
                    print("continuing measurement")

                if pos_chan3!=current_positions[2]:
                    self.device.MoveTo(self.chan3, target_positions[2], 100000)
                    max_steps = max(max_steps, abs(pos_chan3))
                
                    wait_time = max_steps/self.steprate + 0.5 if max_steps > 0 else 1.0
                    time.sleep(wait_time)
                else: 
                    print("continuing measurement")
  
                
                if pos_chan4!=current_positions[3]: 
                    self.device.MoveTo(self.chan4, target_positions[3], 100000)
                    max_steps = max(max_steps, abs(pos_chan4))
                
                    wait_time = max_steps/self.steprate + 0.5 if max_steps > 0 else 1.0
                    time.sleep(wait_time)
                else:
                    print("continuing measurement")
                        
        except Exception as e:
            raise RuntimeError(f"Failed to move motor: {e}")
        # Update current position
        self.current_position = tuple(target_positions)
        
    def shutdown(self):
            # Stop Polling and Disconnect
            self.device.StopPolling()
            self.device.Disconnect()
        # Extract parameters from options with default values    


def flipmirror(position):
    """ Function to move motorized flip mirror:
        Args: position the flipmirror has to move to, with 1 being the upright position and 2 
              being the downward position
        """

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
        

import os
import csv
import json
from scipy.stats import linregress
import numpy as np

def create_slope_lookup(channels=[1, 2, 3, 4], directions=[1, -1], output_file="slope_lookup.json"):
    """
    Create a slope lookup table from backlash data CSV files for specified channels and directions.
    Computes slopes for both x and y axes using linear regression on end positions.
    Saves results to a JSON file.
    
    Args:
        channels (list): List of channel numbers (1, 2, 3, 4).
        directions (list): List of directions (1 for up, -1 for down).
        output_file (str): Path to save the JSON output.
    
    Returns:
        dict: Dictionary containing slopes and R² values for each channel and direction.
    """
    # initialize dictionary
    slopes = {}
    
    # calculate slopes for each channel in each direction
    for channel in channels:
        for direction in directions:
            #load calibration files
            direction_str = "up" if direction == 1 else "down"
            file_name = f"backlash_data_chan{channel}_{direction_str}.csv"
            if not os.path.exists(file_name):
                print(f"Error: {file_name} not found. Please run check_backlash for channel {channel}, direction {direction}.")
                continue
            
            steps = []
            end_pos_x = []
            end_pos_y = []
            try:
                # load data from calibration files
                with open(file_name, 'r', newline='') as f:
                    reader = csv.reader(f)
                    header = next(reader)  # Skip header
                    if len(header) != 7:
                        print(f"Error: {file_name} has unexpected column count ({len(header)}). Expected 7 columns.")
                        continue
                    for row in reader:
                        if len(row) != 7:
                            print(f"Warning: Skipping invalid row in {file_name} with {len(row)} columns.")
                            continue
                        try:
                            rep, step, elapsed_time, pos_x, pos_y, res_x, res_y = map(float, row)
                            steps.append(step)
                            end_pos_x.append(pos_x)
                            end_pos_y.append(pos_y)
                        except ValueError as e:
                            print(f"Warning: Skipping invalid row in {file_name}: {e}")
                            continue
            except Exception as e:
                print(f"Error reading {file_name}: {e}")
                continue
            
            if not steps or len(steps) < 2:
                print(f"Insufficient data in {file_name} for linear regression.")
                continue
            
            # Compute slopes for x and y axes
            key = f"chan{channel}_dir{direction}"
            slopes[key] = {}
            
            # Linear regression for x-axis
            if channel == 1 or channel ==3:
                slope_x, _, r_value_x, _, _ = linregress( end_pos_x,steps)
                r_squared_x = r_value_x**2
                if r_squared_x < 0.8:
                    print(f"Warning: Low fit quality for {key}_x (R² = {r_squared_x:.4f}).")
                slopes[key]["slope_x"] = slope_x
                slopes[key]["r_squared_x"] = r_squared_x
                print(f"Computed slope for {key}_x: {slope_x:.4f} steps/pixel (R² = {r_squared_x:.4f})")
            else:
                # Linear regression for y-axis
                slope_y, _, r_value_y, _, _ = linregress(end_pos_y,steps)
                r_squared_y = r_value_y**2
                if r_squared_y < 0.8:
                    print(f"Warning: Low fit quality for {key}_y (R² = {r_squared_y:.4f}).")
                slopes[key]["slope_y"] = slope_y
                slopes[key]["r_squared_y"] = r_squared_y
                print(f"Computed slope for {key}_y: {slope_y:.4f} steps/pixel (R² = {r_squared_y:.4f})")
        
    # Save to JSON
    try:
        with open(output_file, 'w') as f:
            json.dump(slopes, f, indent=4)
        print(f"Slope lookup table saved to {output_file}")
    except Exception as e:
        print(f"Error saving {output_file}: {e}")
    
    return slopes







 

   
