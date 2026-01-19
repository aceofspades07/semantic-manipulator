import cv2
import numpy as np
import pyrealsense2 as rs

class JengaBlockDetector:
    def __init__(self, focal_length=None, real_block_length=7.5):
        """
        Initialize the Jenga block detector
        
        Args:
            focal_length: Camera focal length in pixels
            real_block_length: The length of the Jenga block's LONGEST side in cm.
                               Standard Jenga is 1.5 x 2.5 x 7.5 cm.
                               We use the longest side for better distance stability.
        """
        self.focal_length = focal_length
        self.real_block_length = real_block_length
        
        # Define HSV color ranges for each Jenga block color
        self.color_ranges = {
            'red': [(np.array([0, 100, 100]), np.array([10, 255, 255])),
                    (np.array([160, 100, 100]), np.array([180, 255, 255]))],
            'blue': [(np.array([100, 100, 100]), np.array([130, 255, 255]))],
            'green': [(np.array([40, 50, 50]), np.array([80, 255, 255]))],
            'yellow': [(np.array([20, 100, 100]), np.array([35, 255, 255]))],
            'pink': [(np.array([140, 50, 50]), np.array([170, 255, 255]))],
            'orange': [(np.array([10, 100, 100]), np.array([25, 255, 255]))]
        }
    
    def segment_color(self, hsv_image, color_name):
        """Create mask for a specific color"""
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        for lower, upper in self.color_ranges[color_name]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv_image, lower, upper))
        
        # Clean up the mask (Morphological operations)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        return mask
    
    def find_aligned_rectangle(self, contour):
        """Find the minimum area rectangle aligned with block edges"""
        # minAreaRect returns (center(x, y), (width, height), angle of rotation)
        rect = cv2.minAreaRect(contour)
        
        # FIX: np.int0 is deprecated in newer numpy versions, use int32
        box = cv2.boxPoints(rect)
        box = np.int32(box) 
        
        center = rect[0]
        width, height = rect[1]
        angle = rect[2]
        
        # Standardize: Ensure 'width' is always the longest side.
        # This is crucial because our distance calc relies on the known length (7.5cm).
        if width < height:
            width, height = height, width
            angle += 90  # Adjust angle to match the swap
        
        return {
            'center': center,
            'width': width,   # Longest side in pixels
            'height': height, # Shortest side in pixels
            'angle': angle,
            'box': box
        }
    
    def calculate_distance(self, pixel_width):
        """Calculate distance from camera to block center using Pinhole model"""
        if self.focal_length is None:
            return 0.0
            
        # D = (Real_Width * Focal_Length) / Pixel_Width
        distance = (self.real_block_length * self.focal_length) / pixel_width
        return distance
    
    def detect_blocks(self, image):
        """Detect all Jenga blocks in the image"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detected_blocks = []
        
        for color_name in self.color_ranges.keys():
            mask = self.segment_color(hsv, color_name)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter noise
                if area < 1000: # Increased threshold slightly for cleaner results
                    continue
                
                rect_info = self.find_aligned_rectangle(contour)
                
                # Calculate distance using the LONGEST side (stored in rect_info['width'])
                dist = 0.0
                if self.focal_length is not None:
                    dist = self.calculate_distance(rect_info['width'])
                
                block_data = {
                    'color': color_name,
                    'center': rect_info['center'],
                    'width': rect_info['width'],
                    'height': rect_info['height'],
                    'angle': rect_info['angle'],
                    'box': rect_info['box'],
                    'area': area,
                    'distance': dist
                }
                
                detected_blocks.append(block_data)
        
        return detected_blocks
    
    def draw_results(self, image, blocks):
        """Draw detected blocks and information on the image"""
        result = image.copy()
        
        for block in blocks:
            # Draw the aligned rectangle
            cv2.drawContours(result, [block['box']], 0, (0, 255, 0), 2)
            
            # Draw center point
            center = tuple(map(int, block['center']))
            cv2.circle(result, center, 5, (0, 0, 255), -1)
            
            # Text information
            text_lines = [
                f"{block['color'].upper()}",
                f"Dist: {block['distance']:.1f} cm"
            ]
            
            y_offset = -20
            for line in text_lines:
                cv2.putText(result, line, (center[0] - 40, center[1] + y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                y_offset += 20
        
        return result

# --- MAIN EXECUTION ---
if __name__ == "__main__":
    # Standard Jenga block length is 7.5 cm (Longest side)
    detector = JengaBlockDetector(real_block_length=7.5)
    
    print("Starting RealSense Stream...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # Initialize frame count variable
    frame_count = 0
    
    try:
        profile = pipeline.start(config)
        
        # Get intrinsics for automatic focal length
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        
        # fx is the focal length in pixels for the width axis
        detector.focal_length = intrinsics.fx
        print(f"Camera Initialized. Focal Length: {detector.focal_length:.2f} px")
        
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue
            
            frame = np.asanyarray(color_frame.get_data())
            frame_count += 1
            
            # Detection
            blocks = detector.detect_blocks(frame)
            result_frame = detector.draw_results(frame, blocks)
            
            # GUI Overlays
            cv2.putText(result_frame, f"Blocks: {len(blocks)}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow('RealSense Jenga Detection', result_frame)
            
            # Print periodic logs
            if frame_count % 60 == 0 and blocks:
                print(f"--- Frame {frame_count} ---")
                for b in blocks:
                    print(f"[{b['color']}] Dist: {b['distance']:.1f}cm | Angle: {b['angle']:.1f}")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()