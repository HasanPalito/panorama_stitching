import cv2
import time
import threading
import numpy as np
import wiringpi

# Camera calibration parameters
camera_matrix = np.array([[1.36173853e+03, 0.00000000e+00, 9.07939104e+02],
                          [0.00000000e+00, 1.35749773e+03, 5.49729293e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

# Distortion coefficients
dist_coeffs = np.array([[-0.20485858, -0.16969606, -0.00192307, 0.00045057, 0.14444974]])

# Function to undistort frames
def undistorter(img):
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
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

# Function to move the servo
def move_servo(current_angle, target_angle, increment=1, delay=0.05):
    if current_angle < target_angle:
        for a in range(current_angle, int(target_angle), increment):
            pulse_width = MIN_PULSE + (a / 180) * (MAX_PULSE - MIN_PULSE)
            wiringpi.softPwmWrite(PIN_TO_PWM, int(pulse_width))
            time.sleep(delay)
    elif current_angle > target_angle:
        for a in range(current_angle, int(target_angle), -increment):
            pulse_width = MIN_PULSE + (a / 180) * (MAX_PULSE - MIN_PULSE)
            wiringpi.softPwmWrite(PIN_TO_PWM, int(pulse_width))
            time.sleep(delay)

# Thread function to capture video
def capture_video_thread(filename, duration=10, fps=20):
    camera = cv2.VideoCapture(1)
    if not camera.isOpened():
        print("Error: Could not open camera.")
        return

    # Define the codec and create a VideoWriter object for MP4
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    frame_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out = cv2.VideoWriter(filename, fourcc, fps, (frame_width, frame_height))

    start_time = time.time()
    total_frames = 0

    # Capture video for the specified duration
    while time.time() - start_time < duration:
        ret, frame = camera.read()
        if ret:
            #frame = undistorter(frame)
            out.write(frame)
            total_frames += 1
            print(f"Captured frame {total_frames}")
        else:
            print("Failed to capture frame")

    camera.release()
    out.release()
    print(f"Video saved to {filename}. Total frames captured: {total_frames}")

# Function to run both servo movement and video capture
def run_servo_and_video():
    # Thread to capture video
    video_thread = threading.Thread(target=capture_video_thread, args=('panorama_video.mp4', 10, 20))
    
    # Start the video capture thread
    video_thread.start()

    # Move the servo while video is being captured
    current_angle = 0
    target_angle = 180  # Rotate from 0 to 180 degrees
    move_servo(current_angle, target_angle, increment=1, delay=0.1)

    # Wait for the video thread to complete
    video_thread.join()

# Example usage
run_servo_and_video()
