"""Color segmentation and distance estimation for Jenga blocks."""

import cv2
import numpy as np

KNOWN_WIDTH_CM = 7.5
FOCAL_LENGTH = 1000

def get_hsv_ranges():
    """Define HSV ranges for Jenga block colors."""
    colors = {
        "Blue":   (np.array([100, 150, 0]), np.array([140, 255, 255])),
        "Green":  (np.array([40, 70, 70]),  np.array([80, 255, 255])),
        "Yellow": (np.array([20, 100, 100]), np.array([30, 255, 255])),
        "Orange": (np.array([10, 100, 20]), np.array([20, 255, 255])),
        "Pink":   (np.array([140, 50, 50]), np.array([170, 255, 255]))
    }
    # Red wraps around 0/180 in HSV
    red_lower1 = np.array([0, 70, 50])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([170, 70, 50])
    red_upper2 = np.array([180, 255, 255])
    
    return colors, (red_lower1, red_upper1, red_lower2, red_upper2)

def process_frame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    colors, red_ranges = get_hsv_ranges()
    
    tasks = []

    for name, (lower, upper) in colors.items():
        mask = cv2.inRange(hsv, lower, upper)
        tasks.append((mask, name, (255, 255, 255)))

    # Combine red ranges
    mask1 = cv2.inRange(hsv, red_ranges[0], red_ranges[1])
    mask2 = cv2.inRange(hsv, red_ranges[2], red_ranges[3])
    red_mask = cv2.add(mask1, mask2)
    tasks.append((red_mask, "Red", (0, 0, 255)))

    for mask, color_name, draw_color in tasks:
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) < 500:
                continue

            rect = cv2.minAreaRect(cnt)
            (x, y), (w, h), angle = rect
            
            apparent_width = max(w, h)
            
            if apparent_width > 0:
                distance = (KNOWN_WIDTH_CM * FOCAL_LENGTH) / apparent_width
                
                box = cv2.boxPoints(rect) 
                box = np.int0(box)
                
                cv2.drawContours(frame, [box], 0, draw_color, 2)
                cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)
                
                label = f"{color_name}: {distance:.2f}cm"
                cv2.putText(frame, label, (int(x) - 20, int(y) - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return frame


cap = cv2.VideoCapture("/dev/video8")

while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    processed_frame = process_frame(frame)
    
    cv2.imshow("Jenga Distance & Segmentation", processed_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()