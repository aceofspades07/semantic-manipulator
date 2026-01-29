import cv2
import numpy as np

# Load the dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Generate 4 markers
for id_ in [0, 1, 2, 3]:
    # 200x200 pixel image
    img = cv2.aruco.generateImageMarker(aruco_dict, id_, 200) 
    cv2.imwrite(f"marker_{id_}.png", img)
    print(f"Saved marker_{id_}.png")


