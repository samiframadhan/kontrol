import cv2

# Open video file
cap = cv2.VideoCapture('output.avi')  # Change to your .avi file path

if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Show the HSV frame
    cv2.imshow('HSV Video', hsv_frame)

    # Exit on 'q' key
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break
    elif cv2.waitKey(30) & 0xFF == ord('p'):
        while True:
            cv2.imshow('HSV Video', hsv_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

cap.release()
cv2.destroyAllWindows()

