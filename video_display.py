import cv2
import numpy as np

# ---------- Camera ----------
cap = cv2.VideoCapture("/dev/video8", cv2.CAP_V4L2)

if not cap.isOpened():
    print("❌ Could not open camera")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# ---------- HSV Color Ranges (TUNE THESE) ----------
# Format: (lower HSV, upper HSV)
COLOR_RANGES = {
    "Red": [
        ((0, 120, 70), (10, 255, 255)),
        ((170, 120, 70), (180, 255, 255))
    ],
    "Green": [((35, 80, 70), (85, 255, 255))],
    "Blue":  [((100, 150, 70), (130, 255, 255))],
    "Yellow":[((20, 100, 100), (30, 255, 255))]
}

# ---------- Morphology ----------
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

print("🎥 Running... Press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Frame grab failed")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color_name, ranges in COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        # Combine multiple ranges (important for red)
        for lower, upper in ranges:
            lower = np.array(lower)
            upper = np.array(upper)
            mask |= cv2.inRange(hsv, lower, upper)

        # Clean mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours (each ≈ one Jenga block)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 600:   # ignore noise
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h),
                          (0, 255, 0), 2)

            # Label
            cv2.putText(frame, color_name, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)

    cv2.imshow("Jenga Block Color Segmentation", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
