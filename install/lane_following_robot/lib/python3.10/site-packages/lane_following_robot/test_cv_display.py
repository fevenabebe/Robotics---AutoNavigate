import cv2
import numpy as np

print("Attempting to create and show a simple OpenCV window...")

try:
    # Create a blank black image
    img = np.zeros((400, 600, 3), dtype=np.uint8)
    # Draw a white rectangle
    cv2.rectangle(img, (100, 100), (500, 300), (255, 255, 255), -1)

    cv2.namedWindow("Test Window", cv2.WINDOW_NORMAL)
    cv2.imshow("Test Window", img)
    print("Window should be visible now. Press any key to close.")
    cv2.waitKey(0) # Wait indefinitely until a key is pressed
    cv2.destroyAllWindows()
    print("Window closed successfully.")
except Exception as e:
    print(f"An error occurred: {e}")

print("Script finished.")