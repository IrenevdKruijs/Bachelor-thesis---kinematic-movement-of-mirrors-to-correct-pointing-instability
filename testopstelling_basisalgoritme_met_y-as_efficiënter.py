import numpy as np
import time
import clr
from pypylon import pylon
##Piezo inladen
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.InertialMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *

## camera, info van:https://pythonforthelab.com/blog/getting-started-with-basler-cameras/ 
# camera verbinden


tl_factory = pylon.TlFactory.GetInstance()
devices = tl_factory.EnumerateDevices()
for device in devices:
    print(device.GetFriendlyName())
    
# installeren van instant camera
tl_factory = pylon.TlFactory.GetInstance()
camera = pylon.InstantCamera()
camera.Attach(tl_factory.CreateFirstDevice())

## functie voor het maken van een afbeelding
def image():   # afbeelding maken
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
            #cv2.imwrite(f'{channel}{new_pos}.tiff', img)
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
    
def coordinates(inputimage,pos_chan1,pos_chan2,pos_chan3,pos_chan4):
    
    # Bestand voor coördinaten
    output_file = 'coordinates.txt'
    # if os.path.exists(output_file):
    #     os.remove(output_file)  # Verwijder oud bestand
        
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
    
    print(f"Coördinaten opgeslagen voor {identifier}: ({middle_x}, {middle_y})")
    return middle_x, middle_y

def piezomotor(new_pos_chan1,new_pos_chan2,new_pos_chan3,new_pos_chan4):
    """The main entry point for the application"""
    try:

        DeviceManagerCLI.BuildDeviceList()

        # create new device
        serial_no = "97251304"  #Serial number of device
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
        device.StartPolling(250)  #250ms polling rate
        time.sleep(0.25)
        device.EnableDevice()
        time.sleep(0.25)  # Wait for device to enable

        # Load any configuration settings needed by the controller/stage
        inertial_motor_config = device.GetInertialMotorConfiguration(serial_no)

        # Get parameters related to homing/zeroing/moving
        device_settings = ThorlabsInertialMotorSettings.GetSettings(inertial_motor_config)

        # Step parameters for an intertial motor channel
        chan1 = InertialMotorStatus.MotorChannels.Channel1
        chan2 = InertialMotorStatus.MotorChannels.Channel2
        chan3 = InertialMotorStatus.MotorChannels.Channel3
        chan4 = InertialMotorStatus.MotorChannels.Channel4# enum chan ident
        device_settings.Drive.Channel(chan1).StepRate = 500
        device_settings.Drive.Channel(chan1).StepAcceleration = 100000
        device_settings.Drive.Channel(chan2).StepRate = 500
        device_settings.Drive.Channel(chan2).StepAcceleration = 100000
        # Send settings to the device
        device.SetSettings(device_settings, True, True)

        #Home or Zero the device (if a motor/piezo)
        print("Zeroing device")
        device.SetPositionAs(chan1, 0)
        device.SetPositionAs(chan2,0)
        device.SetPositionAs(chan3, 0)
        device.SetPositionAs(chan4,0)

        # Move the device to a new position
        '''
        Methods that take an integer argument as an input move in terms of 
        device steps (step size can be user-defined). These methods are 
        used in open-loop operation.     
        '''
        channel = chan1
        if new_pos_chan1 !=0:
            device.MoveTo(channel, int(new_pos_chan1), 6000)  # 60 second timeout

        
        channel = chan2
        if new_pos_chan2 !=0:
            device.MoveTo(channel, int(new_pos_chan2), 6000) # 60 second timeout

        channel = chan3
        if new_pos_chan3 !=0:
            device.MoveTo(channel, int(new_pos_chan3), 6000)  # 3 second timeout
        
        channel = chan4
        if new_pos_chan4 !=0:
            device.MoveTo(channel, int(new_pos_chan4), 6000)  # 3 second timeout
        
        img = image()
        
        middle_x,middle_y = coordinates(img,new_pos_chan1,new_pos_chan2,new_pos_chan3,new_pos_chan4)

        time.sleep(1)

        # Stop Polling and Disconnect
        device.StopPolling()
        device.Disconnect()
        return middle_x,middle_y
    except Exception as e:
        print(e)
 
    ...

# if __name__ == "__main__":
#     main()
im = image()
middle_x,middle_y = coordinates(im,0,0,0,0)
print(middle_x)

## loop for finding back the middle pixel
#initializing loop
x_target = 1283 #set the target pixel the middle of the laser has to go to
y_target = 901
margin = 100 #set the margin of the system
slowmargin = 250
maximum_tries=0

while (abs(middle_x - x_target) > margin or abs(middle_y - y_target) > margin) and maximum_tries < 10:
    maximum_tries += 1

    if middle_x is None or middle_y is None:
        print("Fout: middle_x of middle_y is None (ongeldige afbeelding of coördinaten), stop de loop")
        break

    print(f"Iteratie {maximum_tries}: middle_x = {middle_x}, middle_y = {middle_y}")

    # Bereken benodigde stappen
    dx = 0
    dy = 0

    # X-richting
    if abs(middle_x - x_target) > margin:
        if abs(middle_x - x_target) < slowmargin:
            dx = 250 if middle_x < x_target else -250
        else:
            dx = 1000 if middle_x < x_target else -1000

    # Y-richting
    if abs(middle_y - y_target) > margin:
        if abs(middle_y - y_target) < slowmargin:
            dy = -250 if middle_y < y_target else 250
        else:
            dy = -1000 if middle_y < y_target else 1000

    # Beweeg in beide richtingen tegelijk
    if dx != 0 or dy != 0:
        print(f"Beweeg met dx = {dx}, dy = {dy}")
        middle_x, middle_y = piezomotor(dx, dy, 0, 0)
        time.sleep(0)
    else:
        print("Calibratie voltooid: beide coördinaten binnen marge.")
        break

 
# Controleer of de loop succesvol was
if abs(middle_x - x_target) <= margin:
    print(f"\nSucces! Eindcoördinaten: middle_x = {middle_x}, middle_y = {middle_y}")
    print(f"Bereikt in {maximum_tries} pogingen")
else:
    print(f"\nMislukt! Kon x_target ({x_target}) niet bereiken binnen {margin} pixels en 10 stappen")
    print(f"Eindcoördinaten: middle_x = {middle_x}, middle_y = {middle_y}")
    print(f"Aantal pogingen: {maximum_tries}")




  