"""Jenga block detector using color segmentation and distance estimation."""

import cv2
import numpy as np
import pyrealsense2 as rs

TABLE_Z_HEIGHT = -120.0
BLOCK_HEIGHT = 15.0

class JengaBlockDetector:
    def __init__(self, focal_length=None, real_block_length=7, camera_intrinsics=None):
        self.focal_length = focal_length
        self.real_block_length = real_block_length

        # Known Jenga block dimensions in cm (1.5 x 2.5 x 7)
        self.real_block_short_sides = (2.5, 1.5)
        self.split_tolerance = 0.30
        self.max_split_per_axis = 8
        
        # Camera intrinsics for 3D coordinate conversion
        self.camera_intrinsics = camera_intrinsics or {
            'fx': focal_length,
            'fy': focal_length,
            'ppx': 320,
            'ppy': 240
        }
        
        # HSV color ranges for each block color
        self.color_ranges = {
            'red': [(np.array([0, 100, 100]), np.array([5, 255, 255])),
                    (np.array([160, 100, 100]), np.array([180, 255, 255]))],
            'blue': [(np.array([70, 100, 100]), np.array([130, 255, 255]))],
            'green': [(np.array([40, 50, 50]), np.array([80, 255, 255]))],
            'yellow': [(np.array([21, 100, 100]), np.array([35, 255, 255]))],
            'pink': [(np.array([140, 25, 150]), np.array([165, 255, 255]))],
            'orange': [(np.array([6, 100, 100]), np.array([20, 255, 255]))]
        }
        
        # Homography matrix for coordinate frame transformation
        self.homography_matrix = None
    
    def segment_color(self, hsv_image, color_name):
        """Create mask for a specific color."""
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        for lower, upper in self.color_ranges[color_name]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv_image, lower, upper))
        
        # Clean up mask with morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        return mask
    
    def find_aligned_rectangle(self, contour):
        """Find the minimum area rectangle aligned with block edges."""
        rect = cv2.minAreaRect(contour)
        
        box = cv2.boxPoints(rect)
        box = np.int32(box) 
        
        center = rect[0]
        width, height = rect[1]
        angle = rect[2]
        
        # Ensure width is always the longest side
        if width < height:
            width, height = height, width
            angle += 90
        
        return {
            'center': center,
            'width': width,   # Longest side in pixels
            'height': height, # Shortest side in pixels
            'angle': angle,
            'box': box
        }
    
    def calculate_distance(self, pixel_width):
        """Calculate distance from camera to block center using pinhole model."""
        if self.focal_length is None:
            return 0.0
            
        distance = (self.real_block_length * self.focal_length) / pixel_width
        return distance
    
    def pixel_to_3d_world(self, pixel_x, pixel_y, distance_cm):
        """Convert 2D image coordinates to 3D world coordinates."""
        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        ppx = self.camera_intrinsics['ppx']
        ppy = self.camera_intrinsics['ppy']
        
        # Pinhole camera model conversion
        x = (pixel_x - ppx) * distance_cm / fx
        y = (pixel_y - ppy) * distance_cm / fy
        z = distance_cm
        
        return {
            'x': x,
            'y': y,
            'z': z
        }
    
    def load_calibration_matrix(self, matrix_path="/home/arduino/Qualcomm-AI-Challenge/calibration/calibration_matrix.npy"):
        """Load homography matrix from calibration file."""
        try:
            self.homography_matrix = np.load(matrix_path)
            print(f"Loaded calibration matrix from {matrix_path}")
            return True
        except FileNotFoundError:
            print(f"Error: {matrix_path} not found! Run calibrate.py first!")
            self.homography_matrix = None
            return False
    
    def transform_to_new_frame(self, camera_coords):
        """Transform coordinates from camera frame to robot frame using homography."""
        if self.homography_matrix is None:
            return None
            
        # Project 3D camera coordinates to 2D image coordinates
        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        ppx = self.camera_intrinsics['ppx']
        ppy = self.camera_intrinsics['ppy']
        
        # Convert 3D camera coords back to image pixels
        if abs(camera_coords['z']) < 1e-6:
            return None
            
        u = (camera_coords['x'] * fx / camera_coords['z']) + ppx
        v = (camera_coords['y'] * fy / camera_coords['z']) + ppy
        
        # Apply homography to transform image coordinates to new frame coordinates
        image_point = np.array([u, v, 1.0], dtype=np.float64)
        transformed_point = self.homography_matrix @ image_point
        
        # Normalize homogeneous coordinates
        if abs(transformed_point[2]) < 1e-6:
            return None
            
        x_new = transformed_point[0] / transformed_point[2]
        y_new = transformed_point[1] / transformed_point[2]
        
        # Calculate z coordinate in robot frame
        camera_z_in_new_frame = 78.5
        z_new = camera_z_in_new_frame - camera_coords['z'] + TABLE_Z_HEIGHT + BLOCK_HEIGHT / 2.0
        
        return {
            'x': float(x_new),
            'y': float(y_new),
            'z': float(z_new)
        }

    def _major_axis_unit_vector(self, angle_deg):
        """Return a unit vector along the block's longest side in image coords."""
        theta = np.deg2rad(angle_deg)
        vx = float(np.cos(theta))
        vy = float(np.sin(theta))
        norm = (vx * vx + vy * vy) ** 0.5
        if norm < 1e-8:
            return 1.0, 0.0
        return vx / norm, vy / norm

    def _direction_away_from_observer_bottom(self, vx, vy):
        """Resolve vector direction to point away from bottom of frame."""
        eps = 1e-6
        if (vy > eps) or (abs(vy) <= eps and vx < 0):
            return -vx, -vy
        return vx, vy
    
    def _perpendicular_anticlockwise(self, vx, vy):
        """Return a vector perpendicular (90 degrees anticlockwise) to input."""
        return vy, -vx
    
    def _line_segment_intersection(self, p1, p2, p3, p4):
        """Find intersection point between two line segments."""
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        
        if abs(denom) < 1e-10:
            return None  # Lines are parallel
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
        
        # Check if intersection is within both line segments
        if 0 <= t <= 1 and 0 <= u <= 1:
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            return (x, y)
        
        return None
    
    def _find_bbox_intersection(self, center, direction_vx, direction_vy, box_points):
        """Find where a ray from center intersects the bounding box."""
        cx, cy = center
        
        # Create a long ray from center in the given direction
        ray_length = 1000  # Large enough to ensure it crosses the box
        ray_end = (cx + direction_vx * ray_length, cy + direction_vy * ray_length)
        
        # Check intersection with each of the 4 edges of the box
        closest_intersection = None
        min_distance = float('inf')
        
        for i in range(4):
            p1 = tuple(box_points[i])
            p2 = tuple(box_points[(i + 1) % 4])
            
            intersection = self._line_segment_intersection(center, ray_end, p1, p2)
            
            if intersection is not None:
                # Calculate distance from center to intersection
                dist = np.sqrt((intersection[0] - cx)**2 + (intersection[1] - cy)**2)
                if dist < min_distance:
                    min_distance = dist
                    closest_intersection = intersection
        
        return closest_intersection

    def _split_rect_if_merged(self, rect_info):
        """Split a merged min-area rectangle into multiple block-sized rectangles.
        
        When multiple same-color blocks touch, they form a single contour.
        This splits oversized rectangles based on expected Jenga dimensions.
        """
        width = float(rect_info['width'])
        height = float(rect_info['height'])
        if width <= 1e-6 or height <= 1e-6:
            return [rect_info]

        observed_aspect = width / height

        # Choose expected aspect ratio that best matches observation
        expected_aspects = [self.real_block_length / s for s in self.real_block_short_sides]
        expected_aspect = min(expected_aspects, key=lambda a: abs(observed_aspect - a))

        tol = float(self.split_tolerance)

        # Determine how many blocks are merged along each axis
        n_major = 1
        if observed_aspect > expected_aspect * (1.0 + tol):
            n_major = int(round(observed_aspect / expected_aspect))

        n_minor = 1
        # Equivalent to: (height/width) > (1/expected_aspect)*(1+tol)
        if (height / width) * expected_aspect > (1.0 + tol):
            n_minor = int(round((height / width) * expected_aspect))

        n_major = max(1, min(self.max_split_per_axis, n_major))
        n_minor = max(1, min(self.max_split_per_axis, n_minor))

        if n_major == 1 and n_minor == 1:
            return [rect_info]

        # Subdivide into grid of rectangles aligned with same angle
        angle = float(rect_info['angle'])
        theta = np.deg2rad(angle)
        ux, uy = float(np.cos(theta)), float(np.sin(theta))
        vx, vy = -uy, ux

        sub_w = width / n_major
        sub_h = height / n_minor

        cx0, cy0 = float(rect_info['center'][0]), float(rect_info['center'][1])
        out = []

        for i in range(n_major):
            for j in range(n_minor):
                # Center offsets to keep grid centered on original rect
                off_major = (i - (n_major - 1) / 2.0) * sub_w
                off_minor = (j - (n_minor - 1) / 2.0) * sub_h
                cx = cx0 + off_major * ux + off_minor * vx
                cy = cy0 + off_major * uy + off_minor * vy

                sub_rect = ((cx, cy), (sub_w, sub_h), angle)
                box = cv2.boxPoints(sub_rect)
                box = np.int32(box)

                out.append({
                    'center': (cx, cy),
                    'width': sub_w,
                    'height': sub_h,
                    'angle': angle,
                    'box': box
                })

        return out
    
    def detect_blocks(self, image):
        """Detect all Jenga blocks in the image."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detected_blocks = []
        
        for color_name in self.color_ranges.keys():
            mask = self.segment_color(hsv, color_name)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by minimum area
                if area < 500:
                    continue
                
                rect_info = self.find_aligned_rectangle(contour)
                # Split merged blobs into individual block rectangles
                rect_infos = self._split_rect_if_merged(rect_info)
                
                # Filter by solidity
                hull = cv2.convexHull(contour)
                hull_area = cv2.contourArea(hull)
                if hull_area > 0:
                    solidity = area / hull_area
                    if solidity < 0.6:
                        continue

                # Process each rectangle
                approx_area_each = float(area) / max(1, len(rect_infos))

                for ri in rect_infos:
                    width = float(ri['width'])
                    height = float(ri['height'])
                    if width <= 1e-6 or height <= 1e-6:
                        continue

                    aspect_ratio = width / height
                    # Filter invalid aspect ratios
                    if aspect_ratio < 1.2 or aspect_ratio > 6:
                        continue

                    # Calculate distance using longest side
                    dist = 0.0
                    if self.focal_length is not None:
                        dist = self.calculate_distance(width)

                    # Convert to 3D coordinates
                    center_pixel = ri['center']
                    world_coords = self.pixel_to_3d_world(center_pixel[0], center_pixel[1], dist)
                    
                    # Transform to robot frame if calibration available
                    new_frame_coords = self.transform_to_new_frame(world_coords)
                    
                    # Calculate perpendicular intersection point
                    vx, vy = self._major_axis_unit_vector(ri['angle'])
                    vx, vy = self._direction_away_from_observer_bottom(vx, vy)
                    perp_vx, perp_vy = self._perpendicular_anticlockwise(vx, vy)
                    intersection = self._find_bbox_intersection(ri['center'], perp_vx, perp_vy, ri['box'])
                    
                    # Calculate 3D coordinates for intersection point
                    intersection_world_coords = None
                    intersection_new_frame_coords = None
                    if intersection is not None:
                        intersection_world_coords = self.pixel_to_3d_world(intersection[0], intersection[1], dist)
                        intersection_new_frame_coords = self.transform_to_new_frame(intersection_world_coords)

                    block_data = {
                        'color': color_name,
                        'center': ri['center'],
                        'width': width,
                        'height': height,
                        'angle': ri['angle'],
                        'box': ri['box'],
                        'area': approx_area_each,
                        'distance': dist,
                        'aspect_ratio': aspect_ratio,
                        'solidity': solidity,
                        'world_coords': world_coords,
                        'new_frame_coords': new_frame_coords,
                        'intersection_point': intersection,
                        'intersection_world_coords': intersection_world_coords,
                        'intersection_new_frame_coords': intersection_new_frame_coords
                    }

                    detected_blocks.append(block_data)
        
        return detected_blocks
    
    def draw_results(self, image, blocks):
        """Draw detected blocks and information on the image."""
        result = image.copy()
        
        for block in blocks:
            # Draw aligned rectangle
            cv2.drawContours(result, [block['box']], 0, (0, 255, 0), 2)
            
            # Draw center point
            center = tuple(map(int, block['center']))
            cv2.circle(result, center, 5, (0, 0, 255), -1)

            # Draw direction arrow along block length
            vx, vy = self._major_axis_unit_vector(block['angle'])
            vx, vy = self._direction_away_from_observer_bottom(vx, vy)
            arrow_len = int(max(30, min(0.6 * float(block['width']), 180)))
            end_pt = (
                int(round(block['center'][0] + vx * arrow_len)),
                int(round(block['center'][1] + vy * arrow_len)),
            )
            # Draw arrow with outline for visibility
            cv2.arrowedLine(result, center, end_pt, (0, 0, 0), 10, tipLength=0.35)
            cv2.arrowedLine(result, center, end_pt, (255, 255, 0), 5, tipLength=0.35)
            
            # Draw perpendicular vector to intersection
            if block['intersection_point'] is not None:
                intersection = block['intersection_point']
                intersection_pt = (int(round(intersection[0])), int(round(intersection[1])))
                cv2.arrowedLine(result, center, intersection_pt, (0, 0, 0), 8, tipLength=0.35)
                cv2.arrowedLine(result, center, intersection_pt, (0, 255, 255), 4, tipLength=0.35)
                cv2.circle(result, intersection_pt, 6, (255, 0, 255), -1)
            
            # Display coordinate and color info
            text_lines = []

            if block['new_frame_coords'] is not None:
                coords = block['new_frame_coords']
                text_lines.append(f"({coords['x']:.1f},{coords['y']:.1f},{coords['z']:.1f})")
                text_lines.append(f"{block['color'].upper()}")
                text_lines.append(f"{block['angle']:.0f} DEG")
            else:
                text_lines.append("(N/A,N/A,N/A)")
                text_lines.append(f"{block['color'].upper()}")
                text_lines.append(f"{block['angle']:.0f} DEG")

            y_offset = -30
            for line in text_lines:
                cv2.putText(result, line, (center[0] - 80, center[1] + y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                y_offset += 15
        
        return result

# Main execution
if __name__ == "__main__":
    detector = JengaBlockDetector(real_block_length=7)
    
    print("Starting RealSense Stream...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    

    

    frame_count = 0
    
    try:
        profile = pipeline.start(config)
        
        # Get camera intrinsics
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        
        detector.focal_length = intrinsics.fx
        
        detector.camera_intrinsics = {
            'fx': intrinsics.fx,
            'fy': intrinsics.fy,
            'ppx': intrinsics.ppx,
            'ppy': intrinsics.ppy
        }
        
        print(f"Camera Initialized.")
        print(f"  Focal Length: fx={intrinsics.fx:.2f}px, fy={intrinsics.fy:.2f}px")
        print(f"  Principal Point: ({intrinsics.ppx:.2f}, {intrinsics.ppy:.2f})")
        print(f"  Resolution: {intrinsics.width} x {intrinsics.height}")
        
        # Load calibration matrix
        detector.load_calibration_matrix("/home/arduino/Qualcomm-AI-Challenge/calibration/calibration_matrix.npy")
        
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue
            
            frame = np.asanyarray(color_frame.get_data())
            frame_count += 1
            
            # Run detection
            blocks = detector.detect_blocks(frame)
            result_frame = detector.draw_results(frame, blocks)
            
            # Display block count
            cv2.putText(result_frame, f"Blocks: {len(blocks)}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow('RealSense Jenga Detection', result_frame)
            
            # Log intersection coordinates periodically
            if frame_count % 60 == 0 and blocks:
                print(f"--- Frame {frame_count} ---")
                for b in blocks:
                    coords = b['world_coords']
                    new_coords = b['new_frame_coords']
                    
                    if b['intersection_world_coords'] is not None:
                        int_cam = b['intersection_world_coords']
                        int_new = b['intersection_new_frame_coords']
                        print(f"  Intersection -> Camera: X={int_cam['x']:.1f}, Y={int_cam['y']:.1f}, Z={int_cam['z']:.1f}cm", end="")
                        if int_new is not None:
                            print(f" | New Frame: X={int_new['x']:.1f}, Y={int_new['y']:.1f}, Z={int_new['z']:.1f}cm")
                        else:
                            print()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()