import wiringpi
import cv2
import time
import numpy as np
# Constants

camera_matrix = np.array([[1.36173853e+03, 0.00000000e+00, 9.07939104e+02],
                          [0.00000000e+00, 1.35749773e+03, 5.49729293e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

# Distortion coefficients
dist_coeffs = np.array([[-0.20485858, -0.16969606, -0.00192307, 0.00045057, 0.14444974]])
def undistorter(img):
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    # Undistort
    dst = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)
    x, y, w, h = roi
    #undistorted_cropped_img = dst[y:y+h, x:x+w]
    return dst

OUTPUT = 1
PIN_TO_PWM = 6
MIN_PULSE = 5    # 1ms pulse width (~5% duty cycle of 20ms)
MAX_PULSE = 25   # 2ms pulse width (~10% duty cycle of 20ms)
ROTATION_DELAY = 0.5 

# Setup WiringPi
wiringpi.wiringPiSetup()
wiringpi.pinMode(PIN_TO_PWM, OUTPUT)
wiringpi.softPwmCreate(PIN_TO_PWM, MIN_PULSE, 200)  # Range of 0-200 to adjust pulse

# Function to move servo based on degree input
def move_servo(angle):
    # Ensure angle is between 0 and 180
    if 0 <= angle <= 180:
        # Map the angle to the pulse width (MIN_PULSE to MAX_PULSE)
        pulse_width = MIN_PULSE + (angle / 180) * (MAX_PULSE - MIN_PULSE)
        wiringpi.softPwmWrite(PIN_TO_PWM, int(pulse_width))
    else:
        raise "Invalid angle. Please enter an angle between 0 and 180."


def capture_frames(n):
    if n <= 0:
        print("Invalid input. n must be greater than 0.")
        return []

    camera = cv2.VideoCapture(1)
    if not camera.isOpened():
        print("Error: Could not open camera.")
        return []

    frames = []
    step_angle = 180 / n  # Calculate the rotation step per capture

    # Rotate the servo and capture images n times
    for i in range(n):
        angle = i * step_angle  # Increment the angle for each iteration
        move_servo(angle)       # Move servo to the calculated angle
        time.sleep(ROTATION_DELAY)  # Delay to allow the camera to stabilize

        # Capture a frame from the camera
        ret, frame = camera.read()
        frame = undistorter(frame)
        if ret:
            frames.append(frame)
        else:
            print(f"Failed to capture frame at angle {angle} degrees.")

    # Return the list of captured frames
    camera.release()
    return frames

stitcher = cv2.Stitcher.create(cv2.Stitcher_PANORAMA)

def stitch(frames):
    status, panorama = stitcher.stitch(frames)
    if status == cv2.Stitcher_OK:
        cv2.imwrite('panorama.jpg', panorama)
    else :
        raise "error"
    
frames = capture_frames(4)
stitch(frames)