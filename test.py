import cv2

# Initialize the video capture object (0 is usually the default camera)
cap = cv2.VideoCapture(1)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
else:
    # Capture a single frame
    ret, frame = cap.read()

    if ret:
        cv2.imwrite('captured_frame.png', frame)

    # Release the video capture object
    cap.release()
    cv2.destroyAllWindows()
