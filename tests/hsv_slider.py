"""RGB color segmenter with adjustable sliders for tuning Jenga block detection."""

import cv2
import numpy as np
import pyrealsense2 as rs

class RGBColorSegmenter:
    def __init__(self):
        """Initialize RGB Color Segmenter with adjustable thresholds."""
        
        self.color_ranges = {
            'red': [(np.array([0, 0, 100]), np.array([100, 100, 255]))],
            'blue': [(np.array([100, 0, 0]), np.array([255, 100, 100]))],
            'green': [(np.array([0, 100, 0]), np.array([100, 255, 100]))],
            'yellow': [(np.array([0, 150, 150]), np.array([100, 255, 255]))],
            'pink': [(np.array([150, 100, 150]), np.array([255, 200, 255]))],
            'orange': [(np.array([0, 100, 150]), np.array([100, 200, 255]))]
        }
        
        self.current_color = 'red'
        self.current_range_index = 0
        
        self.window_name = 'RGB Color Segmenter'
        cv2.namedWindow(self.window_name)
        
        self._create_trackbars()
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
    def _create_trackbars(self):
        """Create trackbars for RGB adjustment."""
        current_range = self.color_ranges[self.current_color][self.current_range_index]
        lower = current_range[0]
        upper = current_range[1]
        
        cv2.createTrackbar('B_min', self.window_name, int(lower[0]), 255, self._on_trackbar)
        cv2.createTrackbar('G_min', self.window_name, int(lower[1]), 255, self._on_trackbar)
        cv2.createTrackbar('R_min', self.window_name, int(lower[2]), 255, self._on_trackbar)
        
        cv2.createTrackbar('B_max', self.window_name, int(upper[0]), 255, self._on_trackbar)
        cv2.createTrackbar('G_max', self.window_name, int(upper[1]), 255, self._on_trackbar)
        cv2.createTrackbar('R_max', self.window_name, int(upper[2]), 255, self._on_trackbar)
        
        cv2.createTrackbar('Color', self.window_name, 0, 5, self._on_color_change)
        cv2.createTrackbar('Range', self.window_name, 0, 0, self._on_range_change)
    
    def _on_trackbar(self, val):
        """Callback for trackbar changes."""
        pass
    
    def _on_color_change(self, val):
        """Callback when color selection changes."""
        colors = ['red', 'blue', 'green', 'yellow', 'pink', 'orange']
        self.current_color = colors[val]
        self.current_range_index = 0
        self._update_trackbars()
    
    def _on_range_change(self, val):
        """Callback when range selection changes."""
        if val < len(self.color_ranges[self.current_color]):
            self.current_range_index = val
            self._update_trackbars()
    
    def _update_trackbars(self):
        """Update trackbar positions to reflect current color range."""
        current_range = self.color_ranges[self.current_color][self.current_range_index]
        lower = current_range[0]
        upper = current_range[1]
        
        cv2.setTrackbarPos('B_min', self.window_name, int(lower[0]))
        cv2.setTrackbarPos('G_min', self.window_name, int(lower[1]))
        cv2.setTrackbarPos('R_min', self.window_name, int(lower[2]))
        cv2.setTrackbarPos('B_max', self.window_name, int(upper[0]))
        cv2.setTrackbarPos('G_max', self.window_name, int(upper[1]))
        cv2.setTrackbarPos('R_max', self.window_name, int(upper[2]))
    
    def _get_current_rgb_range(self):
        """Get current RGB range from trackbars."""
        b_min = cv2.getTrackbarPos('B_min', self.window_name)
        g_min = cv2.getTrackbarPos('G_min', self.window_name)
        r_min = cv2.getTrackbarPos('R_min', self.window_name)
        b_max = cv2.getTrackbarPos('B_max', self.window_name)
        g_max = cv2.getTrackbarPos('G_max', self.window_name)
        r_max = cv2.getTrackbarPos('R_max', self.window_name)
        
        lower = np.array([b_min, g_min, r_min])
        upper = np.array([b_max, g_max, r_max])
        
        return lower, upper
    
    def segment_color(self, bgr_image, color_name):
        """Create mask for a specific color using stored ranges."""
        mask = np.zeros(bgr_image.shape[:2], dtype=np.uint8)
        
        for lower, upper in self.color_ranges[color_name]:
            mask = cv2.bitwise_or(mask, cv2.inRange(bgr_image, lower, upper))
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        return mask
    
    def segment_current_color_with_sliders(self, bgr_image):
        """Segment using current slider values."""
        lower, upper = self._get_current_rgb_range()
        mask = cv2.inRange(bgr_image, lower, upper)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        return mask
    
    def start_camera(self):
        """Initialize RealSense camera."""
        try:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(self.config)
            print("Camera started successfully")
            return True
        except Exception as e:
            print(f"Error starting camera: {e}")
            return False
    
    def stop_camera(self):
        """Stop RealSense camera."""
        try:
            self.pipeline.stop()
            print("Camera stopped")
        except:
            pass
    
    def run(self):
        """Main loop for RGB segmentation with live camera feed."""
        if not self.start_camera():
            print("Failed to start camera. Exiting...")
            return
        
        print("\nRGB Color Segmenter")
        print("Controls:")
        print("  Color slider: 0=Red, 1=Blue, 2=Green, 3=Yellow, 4=Pink, 5=Orange")
        print("  Range slider: switch between ranges if available")
        print("  s: save current range")
        print("  p: print all current ranges")
        print("  a: segment ALL colors at once")
        print("  q: quit")
        print()
        
        show_all_colors = False
        
        try:
            while True:
                # Get frames
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                
                if not color_frame:
                    continue
                
                # Convert to numpy array
                color_image = np.asanyarray(color_frame.get_data())
                
                # Image is already in BGR format (OpenCV default)
                bgr_image = color_image
                
                if show_all_colors:
                    # Segment all colors and show them
                    combined_mask = np.zeros(bgr_image.shape[:2], dtype=np.uint8)
                    colored_output = np.zeros_like(color_image)
                    
                    # Define colors for visualization
                    color_bgr = {
                        'red': (0, 0, 255),
                        'blue': (255, 0, 0),
                        'green': (0, 255, 0),
                        'yellow': (0, 255, 255),
                        'pink': (203, 192, 255),
                        'orange': (0, 165, 255)
                    }
                    
                    for color_name in self.color_ranges.keys():
                        mask = self.segment_color(bgr_image, color_name)
                        combined_mask = cv2.bitwise_or(combined_mask, mask)
                        
                        # Color the mask for visualization
                        colored_output[mask > 0] = color_bgr[color_name]
                    
                    # Blend with original image
                    result = cv2.addWeighted(color_image, 0.5, colored_output, 0.5, 0)
                    
                    # Add text overlay
                    cv2.putText(result, "ALL COLORS MODE", (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    
                    cv2.imshow(self.window_name, result)
                else:
                    # Show current color segmentation with adjustable sliders
                    mask = self.segment_current_color_with_sliders(bgr_image)
                    
                    # Create colored overlay
                    result = color_image.copy()
                    result[mask > 0] = (0, 255, 0)  # Green overlay for detected regions
                    
                    # Blend with original
                    result = cv2.addWeighted(color_image, 0.6, result, 0.4, 0)
                    
                    # Get current range for display
                    lower, upper = self._get_current_rgb_range()
                    
                    # Add text overlay
                    text = f"Color: {self.current_color.upper()}"
                    if len(self.color_ranges[self.current_color]) > 1:
                        text += f" (Range {self.current_range_index})"
                    cv2.putText(result, text, (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    
                    range_text = f"B:[{lower[0]}-{upper[0]}] G:[{lower[1]}-{upper[1]}] R:[{lower[2]}-{upper[2]}]"
                    cv2.putText(result, range_text, (10, 60),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    cv2.imshow(self.window_name, result)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    lower, upper = self._get_current_rgb_range()
                    self.color_ranges[self.current_color][self.current_range_index] = (lower, upper)
                    print(f"\nSaved range for {self.current_color} (index {self.current_range_index}):")
                    print(f"  Lower: {lower}")
                    print(f"  Upper: {upper}")
                elif key == ord('p'):
                    print("\nCurrent Color Ranges:")
                    print("self.color_ranges = {")
                    for color_name, ranges in self.color_ranges.items():
                        if len(ranges) == 1:
                            lower, upper = ranges[0]
                            print(f"    '{color_name}': [(np.array({list(lower)}), np.array({list(upper)}))],")
                        else:
                            print(f"    '{color_name}': [", end="")
                            for i, (lower, upper) in enumerate(ranges):
                                if i > 0:
                                    print("                ", end="")
                                print(f"(np.array({list(lower)}), np.array({list(upper)}))", end="")
                                if i < len(ranges) - 1:
                                    print(",")
                                else:
                                    print("],")
                    print("}")
                elif key == ord('a'):
                    show_all_colors = not show_all_colors
                    if show_all_colors:
                        print("\nSwitched to ALL COLORS mode")
                    else:
                        print("\nSwitched to SINGLE COLOR mode")
        
        finally:
            self.stop_camera()
            cv2.destroyAllWindows()


def main():
    segmenter = RGBColorSegmenter()
    segmenter.run()


if __name__ == "__main__":
    main()
