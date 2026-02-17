"""Generate ArUco markers for calibration."""

import cv2
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

for id_ in [4, 5, 6, 7, 8, 9, 10]:
    img = cv2.aruco.generateImageMarker(aruco_dict, id_, 200) 
    cv2.imwrite(f"marker_{id_}.png", img)
    print(f"Saved marker_{id_}.png")


