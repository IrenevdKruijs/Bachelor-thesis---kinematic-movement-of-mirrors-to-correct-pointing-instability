from functions import *
import cv2
import csv
import os

wavelength = int(input("What wavelength are you using?"))
cam = camera_controller(wavelength)
flipmirror(1)

deviations = []
try:
    target_img_cam1 = cv2.imread("target_pixels_cam1.png")
    target_img_cam2 = cv2.imread("target_pixels_cam2.png")
except FileNotFoundError:
    print("Align setup and run calibrate_after_alignment")
    exit()
    
image1 = cam.capture_image(1)
image2 = cam.capture_image(2)

# Do a cross correlation of the target image and the current image of the beam to find the shift
correlated_img1, x1_dev, y1_dev = localize_beam_center(target_img_cam1, image1)
correlated_img2, x2_dev, y2_dev = localize_beam_center(target_img_cam2, image2)

# Store initial deviations for this attempt
deviations.append({
    'wavelength': wavelength,
    'dx1': x1_dev,
    'dx2': x2_dev,
    'dy1': y1_dev,
    'dy2': y2_dev
})

csv_filename = "final_experiment_deviation_per_wavelength_19_04.csv"
# Check if file exists to decide whether to write header
file_exists = os.path.isfile(csv_filename)

with open(csv_filename, mode='a', newline='', encoding='utf-8') as file:
    headers = ['wavelength', 'dx1', 'dy1', 'dx2', 'dy2']
    writer = csv.DictWriter(file, fieldnames=headers)
    # Write header only if the file is new
    if not file_exists:
        writer.writeheader()
    for dev in deviations:
        writer.writerow(dev)
print(f"Deviations data appended to {csv_filename}")

flipmirror(2)