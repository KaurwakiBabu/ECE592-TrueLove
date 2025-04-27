"""

Main function tracks an object based on the color category and returns the HSV of the center pixel of the "blob"
Used for testing as the center pixel hsv is printed and used to better identify threshold algorithm

"""
import cv2
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path

def get_combined_thresholds(colors):
    threshold_list = []
    current_dir = Path(__file__).parent.resolve()
    tree = ET.parse(current_dir / 'thresholds.xml')
    root = tree.getroot()

    for color in colors:
        node = root.find(color)
        if node is not None:
            for thresh in list(node):
                values = [int(val.text) for val in list(thresh)]
                threshold_list.append(values)
    return threshold_list

def main():
    color_categories = ['default', 'dark_red', 'pink', 'orange','inside_red']
    combined_thresholds = get_combined_thresholds(color_categories)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        im_hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Get center pixel HSV
        bgr = frame[frame.shape[0] // 2, frame.shape[1] // 2]
        center_hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]
        print("Center pixel HSV:", center_hsv)

        # Create mask from all thresholds
        masks = [cv2.inRange(im_hsv,
                             (hsv[0], hsv[1], hsv[2]),
                             (hsv[3], hsv[4], hsv[5])) for hsv in combined_thresholds]
        full_mask = masks[0]
        for m in masks[1:]:
            full_mask = cv2.add(full_mask, m)

        # Show output
        cv2.imshow("Camera Feed", frame)
        cv2.imshow("Combined Mask", full_mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
