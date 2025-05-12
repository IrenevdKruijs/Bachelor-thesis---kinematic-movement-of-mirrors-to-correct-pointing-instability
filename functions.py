from pypylon import pylon
import numpy as np
import time
import clr
##Piezo inladen
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.InertialMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *
def camera_setup():
    """
    This function initializes the camera, works for basler camera's 
    """
    tl_factory = pylon.TlFactory.GetInstance()
    devices = tl_factory.EnumerateDevices()
    for device in devices:
        print(device.GetFriendlyName())
        
    # installeren van instant camera
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
            print(f'Size of image: {img.shape}')
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

def coordinates(inputimage, pos_chan1, pos_chan2, pos_chan3, pos_chan4):
    """
    TO DO: kunnen de posities eruit als ik de identifiers toch niet gebruik om de data op te slaan?
    This function takes the image of a beam on the camera and 
    returns the coordinates (x,y) of the middle pixel of the beam
    inputimage = the image of the beam
    """
    # Bestand voor coördinaten
    output_file = 'coordinates.txt'
    
    # Vind de maximale intensiteit
    max_intensity = np.max(inputimage)
    
    # Vind alle pixels met de maximale intensiteit
    max_pixel_locs = np.where(inputimage == max_intensity)
    max_pixel_coords = list(zip(max_pixel_locs[0], max_pixel_locs[1]))  # Lijst van (y, x)
    
    if not max_pixel_coords:
        print(f"Geen pixels met maximale intensiteit gevonden.")
        return None
    
    # Filter pixels die grenzen aan andere pixels met maximale intensiteit
    connected_pixel_coords = []
    height, width = inputimage.shape
    for y, x in max_pixel_coords:
        # Controleer 8-connectiviteit (boven, onder, links, rechts, diagonaal)
        has_neighbor = False
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dy == 0 and dx == 0:
                    continue
                ny, nx = y + dy, x + dx
                # Controleer of de buur binnen de afbeelding ligt
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
    
    # Bereken de middelste pixel en sla dit op
    y_coords, x_coords = zip(*connected_pixel_coords)
    middle_y = int(np.mean(y_coords))
    middle_x = int(np.mean(x_coords))
    
    # Sla coördinaten op in tekstbestand
    identifier = f"{pos_chan1,pos_chan2,pos_chan3,pos_chan4}"
    with open(output_file, 'a') as f:
        f.write(f"{identifier}:{middle_x}, {middle_y}\n")
    
    print(f"Coördinates saved for {identifier}: ({middle_x}, {middle_y})")
    return middle_x, middle_y

def piezomotor(new_pos_chan1, new_pos_chan2, new_pos_chan3, new_pos_chan4,steprate,camera):
    """
    TO DO: wil ik dat de img en coordinates functie in deze functie zitten of dat die los zijn? 
            Ik denk zelf dat los beter is zodat je functie zo min mogelijk informatie nodig heeft.
    function to move the piezomotors of the different mirrors using a Thorlabs KCube 
    input: new_pos_chan: the new positions the motors have to go to for 4 different channels
           steprate: the steprate at which the motor has to move
    output: 
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
        middle_x, middle_y = coordinates(img, new_pos_chan1, new_pos_chan2, new_pos_chan3, new_pos_chan4)

        time.sleep(1)

        # Stop Polling and Disconnect
        device.StopPolling()
        device.Disconnect()
        return middle_x, middle_y
    except Exception as e:
        print(e)
        return None, None
    
    
    # Extract parameters from options with default values

def calibrate_mirror1_2D(amount_steps, stepsize, repeats,direction,steprate,camera):
    
    all_shifts = []
        
    for h in range(repeats):
        if direction == "up":
            piezomotor(-100, -100, 0, 0,steprate,camera)  # backlash compensation
            x0, y0 = piezomotor(100, 100, 0, 0,steprate,camera)
        else:
            piezomotor(100, 100, 0, 0,steprate,camera)
            x0, y0 = piezomotor(-100, -100, 0, 0,steprate,camera)
        print(f"Startpositie (pixels): {x0}, {y0}")
        stap = stepsize   #initialize stap for correct data savings
        shifts = []     #initialize verschuivingen to save the shifts
        for _ in range(amount_steps + 1):  # inclusief eindpunt
            huidig_stap = stepsize if direction == 'up' else -stepsize
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
            stap += stepsize if direction == 'up' else -stepsize
        all_shifts.append(shifts)
        piezomotor(
        (-(stepsize * amount_steps) if direction == "up" else stepsize * amount_steps),
        (-(stepsize * amount_steps) if direction == "up" else stepsize * amount_steps),
        0, 0, steprate,camera
        )
    return all_shifts            