import cv2
import numpy as np

# --- CONFIGURATION ---
# Real dimension of the Jenga block's longest side in cm (Standard Jenga is 7.5cm)
KNOWN_WIDTH_CM = 7.5 
# You must calibrate this value! (See instructions above)
# Example: If a 7.5cm block at 50cm distance is 150 pixels wide: F = (150*50)/7.5 = 1000
FOCAL_LENGTH = 1000 

def get_hsv_ranges():
    """
    Define HSV ranges for Jenga colors. 
    NOTE: These values depend heavily on lighting. You may need to tune them.
    """
    colors = {
        "Blue":   (np.array([100, 150, 0]), np.array([140, 255, 255])),
        "Green":  (np.array([40, 70, 70]),  np.array([80, 255, 255])),
        "Yellow": (np.array([20, 100, 100]), np.array([30, 255, 255])),
        "Orange": (np.array([10, 100, 20]), np.array([20, 255, 255])),
        "Pink":   (np.array([140, 50, 50]), np.array([170, 255, 255]))
    }
    # Red is special because it wraps around 0/180 in HSV
    red_lower1 = np.array([0, 70, 50])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([170, 70, 50])
    red_upper2 = np.array([180, 255, 255])
    
    return colors, (red_lower1, red_upper1, red_lower2, red_upper2)

def process_frame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    colors, red_ranges = get_hsv_ranges()
    
    # List to store processing instructions: (Mask, ColorName, ColorBGR)
    tasks = []

    # Prepare standard colors
    for name, (lower, upper) in colors.items():
        mask = cv2.inRange(hsv, lower, upper)
        tasks.append((mask, name, (255, 255, 255))) # White text for visibility

    # Prepare Red (Combine the two ranges)
    mask1 = cv2.inRange(hsv, red_ranges[0], red_ranges[1])
    mask2 = cv2.inRange(hsv, red_ranges[2], red_ranges[3])
    red_mask = cv2.add(mask1, mask2)
    tasks.append((red_mask, "Red", (0, 0, 255)))

    # Loop through each color mask
    for mask, color_name, draw_color in tasks:
        # Clean up noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            # Filter small noise
            if cv2.contourArea(cnt) < 500:
                continue

            # --- KEY PART: ROTATED RECTANGLE ---
            # minAreaRect returns: (center(x,y), (width, height), angle)
            rect = cv2.minAreaRect(cnt)
            (x, y), (w, h), angle = rect
            
            # Ensure 'w' is always the longest side to match our KNOWN_WIDTH_CM
            # This handles the case where the block is rotated 90 degrees
            apparent_width = max(w, h)
            
            # --- DISTANCE CALCULATION ---
            if apparent_width > 0:
                distance = (KNOWN_WIDTH_CM * FOCAL_LENGTH) / apparent_width
                
                # --- DRAWING ---
                # Get the 4 corners of the rotated rect
                box = cv2.boxPoints(rect) 
                box = np.int0(box) # Convert to integer
                
                # Draw the aligned rectangle (Red contours)
                cv2.drawContours(frame, [box], 0, draw_color, 2)
                
                # Draw center point
                cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)
                
                # Display text
                label = f"{color_name}: {distance:.2f}cm"
                cv2.putText(frame, label, (int(x) - 20, int(y) - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return frame

# --- MAIN LOOP ---
# Initialize camera (0 is usually default webcam)
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