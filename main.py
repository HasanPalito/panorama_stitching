import cv2
import time
import threading
import numpy as np
import wiringpi
import math
import brisque

obj = brisque.BRISQUE(url=False)

def eval(frame):
    cv2_rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    ndarray = np.asarray(cv2_rgb_image)
    return obj.score(img=ndarray)
    
    
# Thread function to capture video and store frames
captured_frames = [] 
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

def sharpen_image(image):
    # Define a sharpening kernel
    sharpening_kernel = np.array([[ 0, -1,  0],
                                  [-1,  5, -1],
                                  [ 0, -1,  0]])
    
    # Apply the sharpening filter using cv2.filter2D
    sharpened_image = cv2.filter2D(image, -1, sharpening_kernel)
    
    return sharpened_image

# Servo control setup
OUTPUT = 1
PIN_TO_PWM = 6
MIN_PULSE = 5    # 1ms pulse width (~5% duty cycle of 20ms)
MAX_PULSE = 25   # 2ms pulse width (~10% duty cycle of 20ms)
ROTATION_DELAY = 2

total_frames= 0

# Setup WiringPi
wiringpi.wiringPiSetup()
wiringpi.pinMode(PIN_TO_PWM, OUTPUT)
wiringpi.softPwmCreate(PIN_TO_PWM, MIN_PULSE, 200)  # Range of 0-200 to adjust pulse

# Function to move the servo
def move_servo(current_angle, target_angle, steps=100, duration=5):
    total_steps = steps
    step_delay = duration / total_steps
    
    for i in range(total_steps + 1):
        t = i / total_steps  # Progress from 0 to 1
        smoothed_angle = current_angle + (target_angle - current_angle) * (1 - math.cos(t * math.pi)) / 2  # Smooth interpolation using cosine easing
        
        pulse_width = MIN_PULSE + (smoothed_angle / 180) * (MAX_PULSE - MIN_PULSE)
        wiringpi.softPwmWrite(PIN_TO_PWM, int(pulse_width))
        time.sleep(step_delay)

    # Ensure it reaches the exact target angle
    pulse_width = MIN_PULSE + (target_angle / 180) * (MAX_PULSE - MIN_PULSE)
    wiringpi.softPwmWrite(PIN_TO_PWM, int(pulse_width))

# Thread function to capture video
def capture_image():
    global captured_frames  
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)  # Value between 0 and 1
    cap.set(cv2.CAP_PROP_CONTRAST, 0.5)
    cap.set(cv2.CAP_PROP_SATURATION, 0.5)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    for i in range(5):  # Flush 5 frames (adjust as needed)
        ret, frame = cap.read()
    ret, frame = cap.read()
    if ret :
        if eval(frame) < 15 : 
            dst = cv2.fastNlMeansDenoisingColored(frame,None,10,10,7,21)
            return dst

for i in range (0,180,30):
    frame = capture_image()
    cv2.imwrite(f"{i}.jpg",frame)
    move_servo(i)
    captured_frames.append(frame)

stitcher = cv2.Stitcher.create(cv2.Stitcher_PANORAMA)
status, panorama = stitcher.stitch(captured_frames)
if status == cv2.Stitcher_OK:
    cv2.imwrite('panorama.jpg', panorama)
else : 
    print("woelah rek")
    

    
 