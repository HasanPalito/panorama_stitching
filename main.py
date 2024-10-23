import cv2
import time
import numpy as np
import wiringpi

# Camera calibration parameters
camera_matrix = np.array([[1.36173853e+03, 0.00000000e+00, 9.07939104e+02],
                          [0.00000000e+00, 1.35749773e+03, 5.49729293e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

# Distortion coefficients
dist_coeffs = np.array([[-0.20485858, -0.16969606, -0.00192307, 0.00045057, 0.14444974]])

# Function to undistort the frames
def undistorter(img):
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    # Undistort the image
    dst = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)
    return dst

# Servo control setup
OUTPUT = 1
PIN_TO_PWM = 6
MIN_PULSE = 5    # 1ms pulse width (~5% duty cycle of 20ms)
MAX_PULSE = 25   # 2ms pulse width (~10% duty cycle of 20ms)
ROTATION_DELAY = 2 

# Setup WiringPi
wiringpi.wiringPiSetup()
wiringpi.pinMode(PIN_TO_PWM, OUTPUT)
wiringpi.softPwmCreate(PIN_TO_PWM, MIN_PULSE, 200)  # Range of 0-200 to adjust pulse

# Function to move servo based on degree input
def move_servo(current_angle, target_angle, increment=1, delay=0.05):
    if current_angle < target_angle:
        # Move servo up in small increments
        for a in range(current_angle, int(target_angle), increment):
            pulse_width = MIN_PULSE + (a / 180) * (MAX_PULSE - MIN_PULSE)
            wiringpi.softPwmWrite(PIN_TO_PWM, int(pulse_width))
            time.sleep(delay)  # Slow down the movement by adding a delay
    elif current_angle > target_angle:
        # Move servo down in small increments
        for a in range(current_angle, int(target_angle), -increment):
            pulse_width = MIN_PULSE + (a / 180) * (MAX_PULSE - MIN_PULSE)
            wiringpi.softPwmWrite(PIN_TO_PWM, int(pulse_width))
            time.sleep(delay)  # Slow down the movement by adding a delay

# Capture and record video
def capture_video(filename, duration=10, fps=20):
    camera = cv2.VideoCapture(1)
    if not camera.isOpened():
        print("Error: Could not open camera.")
        return

    # Define the codec and create a VideoWriter object for MP4
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    frame_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out = cv2.VideoWriter(filename, fourcc, fps, (frame_width, frame_height))

    current_angle = 0
    target_angle = 180  # Rotate from 0 to 180 degrees

    start_time = time.time()

    while time.time() - start_time < duration:
        # Capture the frame
        ret, frame = camera.read()
        if ret:
            frame = undistorter(frame)
            out.write(frame)

        # Move the servo while capturing video
        move_servo(current_angle, target_angle, increment=1, delay=0.1)
        current_angle = target_angle

    # Release everything when done
    camera.release()
    out.release()
    print(f"Video saved to {filename}")

# Example usage
capture_video('panorama_video.mp4', duration=20, fps=20)
