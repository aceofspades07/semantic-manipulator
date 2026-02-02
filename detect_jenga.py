import cv2
import numpy as np
import pyrealsense2 as rs

TABLE_Z_HEIGHT = -120.0
BLOCK_HEIGHT = 15.0

class JengaBlockDetector:
    def __init__(self, focal_length=None, real_block_length=7, camera_intrinsics=None):
        """
        Initialize the Jenga block detector
        
        Args:
            focal_length: Camera focal length in pixels
            real_block_length: The length of the Jenga block's LONGEST side in cm.
                               Standard Jenga is 1.5 x 2.5 x 7 cm.
                               We use the longest side for better distance stability.
            camera_intrinsics: Dict with keys 'fx', 'fy', 'ppx', 'ppy' for converting to 3D coords
        """
        self.focal_length = focal_length
        self.real_block_length = real_block_length

        # Known real-world short sides (cm). A Jenga block is 1.5 x 2.5 x 7 cm.
        # Depending on orientation, the visible "breadth" can be 2.5cm (top face)
        # or 1.5cm (side face). We use both to avoid over-splitting.
        self.real_block_short_sides = (2.5, 1.5)
        self.split_tolerance = 0.30
        self.max_split_per_axis = 8
        
        # Camera intrinsics for 3D coordinate conversion
        self.camera_intrinsics = camera_intrinsics or {
            'fx': focal_length,
            'fy': focal_length,
            'ppx': 320,  # Default principal point x (half of 640)
            'ppy': 240   # Default principal point y (half of 480)
        }
        
        # Define HSV color ranges for each Jenga block color
        # Tightened ranges to reduce false positives
        self.color_ranges = {
            'red': [(np.array([0, 100, 100]), np.array([5, 255, 255])),
                    (np.array([160, 100, 100]), np.array([180, 255, 255]))],
            'blue': [(np.array([70, 100, 100]), np.array([130, 255, 255]))],
            'green': [(np.array([40, 50, 50]), np.array([80, 255, 255]))],
            'yellow': [(np.array([21, 100, 100]), np.array([35, 255, 255]))], # Start at 21 to avoid Orange
            'pink': [(np.array([140, 25, 150]), np.array([165, 255, 255]))],   # End at 159 to avoid high Red
            'orange': [(np.array([6, 100, 100]), np.array([20, 255, 255]))]  # 11-20 to fit between Red and Yellow
        }
        
        # Homography matrix for coordinate frame transformation
        self.homography_matrix = None
    
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
    
    def pixel_to_3d_world(self, pixel_x, pixel_y, distance_cm):
        """
        Convert 2D image coordinates to 3D world coordinates with camera as origin.
        
        Args:
            pixel_x: X coordinate in image (column)
            pixel_y: Y coordinate in image (row)
            distance_cm: Distance from camera (depth) in cm
            
        Returns:
            Dict with 'x', 'y', 'z' in cm, where camera is at origin (0, 0, 0)
            Z-axis points forward (away from camera)
            X-axis points right
            Y-axis points down
        """
        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        ppx = self.camera_intrinsics['ppx']
        ppy = self.camera_intrinsics['ppy']
        
        # Using pinhole camera model
        # X = (u - ppx) * Z / fx
        # Y = (v - ppy) * Z / fy
        # Z = distance
        
        x = (pixel_x - ppx) * distance_cm / fx
        y = (pixel_y - ppy) * distance_cm / fy
        z = distance_cm
        
        return {
            'x': x,
            'y': y,
            'z': z
        }
    
    def load_calibration_matrix(self, matrix_path="/home/arduino/Qualcomm-AI-Challenge/calibration/calibration_matrix.npy"):
        """Load homography matrix from calibration file"""
        try:
            self.homography_matrix = np.load(matrix_path)
            print(f"Loaded calibration matrix from {matrix_path}")
            return True
        except FileNotFoundError:
            print(f"Error: {matrix_path} not found! Run calibrate.py first!")
            self.homography_matrix = None
            return False
    
    def transform_to_new_frame(self, camera_coords):
        """Transform coordinates from camera frame to new frame using homography
        
        Args:
            camera_coords: Dict with 'x', 'y', 'z' in camera frame (cm)
            
        Returns:
            Dict with 'x', 'y', 'z' in new frame, or None if no calibration matrix
        """
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
        
        # Calculate z coordinate: camera is at z=78.5cm, new frame origin at z=0
        # Block's z in new frame = camera_z - distance_from_camera
        camera_z_in_new_frame = 78.5  # cm
        z_new = camera_z_in_new_frame - camera_coords['z'] + TABLE_Z_HEIGHT + BLOCK_HEIGHT / 2.0
        
        return {
            'x': float(x_new),
            'y': float(y_new),
            'z': float(z_new)
        }

    def _major_axis_unit_vector(self, angle_deg):
        """Return a unit vector (vx, vy) along the block's longest side in image coords.

        Note: OpenCV image coordinates are: +x right, +y down.
        """
        theta = np.deg2rad(angle_deg)
        vx = float(np.cos(theta))
        vy = float(np.sin(theta))
        norm = (vx * vx + vy * vy) ** 0.5
        if norm < 1e-8:
            return 1.0, 0.0
        return vx / norm, vy / norm

    def _direction_away_from_observer_bottom(self, vx, vy):
        """Resolve the +/- ambiguity so the vector points away from an observer at bottom.

        Observer A is at the bottom of the frame, facing the top.
        "Away from A" therefore means "toward the top" in the image plane.
        We choose the sign that maximizes the component in the (0, -1) direction.
        """
        eps = 1e-6
        # Prefer negative y (upwards). If perfectly horizontal, prefer +x for stability.
        if (vy > eps) or (abs(vy) <= eps and vx < 0):
            return -vx, -vy
        return vx, vy
    
    def _perpendicular_anticlockwise(self, vx, vy):
        """Return a vector perpendicular (90° anticlockwise) to the input vector.
        
        For a vector (vx, vy), rotating 90° anticlockwise gives (-vy, vx).
        """
        return vy, -vx
    
    def _line_segment_intersection(self, p1, p2, p3, p4):
        """Find intersection point between two line segments.
        
        Args:
            p1, p2: First line segment endpoints (tuples)
            p3, p4: Second line segment endpoints (tuples)
            
        Returns:
            Intersection point as tuple (x, y) or None if no intersection
        """
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
        """Find where a ray from center intersects the bounding box.
        
        Args:
            center: Starting point (x, y)
            direction_vx, direction_vy: Direction vector (unit vector)
            box_points: 4 corner points of the bounding box
            
        Returns:
            Intersection point (x, y) or None
        """
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
        """Split a (potentially merged) min-area rect into multiple block-sized rects.

        If multiple same-color blocks touch, they can form a single contour.
        Since Jenga blocks have fixed dimensions, we split oversized rectangles into
        multiple equal-breadth rectangles. The number of splits scales with how much
        the breadth/length exceeds the expected single-block size.

        Returns a list of rect_info dicts (same schema as find_aligned_rectangle).
        """
        width = float(rect_info['width'])
        height = float(rect_info['height'])
        if width <= 1e-6 or height <= 1e-6:
            return [rect_info]

        observed_aspect = width / height

        # Choose the expected aspect that best matches this observation to avoid
        # splitting a single block when it is showing the 7x1.5 face.
        expected_aspects = [self.real_block_length / s for s in self.real_block_short_sides]
        expected_aspect = min(expected_aspects, key=lambda a: abs(observed_aspect - a))

        tol = float(self.split_tolerance)

        # Determine how many blocks are merged along each axis.
        # For a single block: observed_aspect ~= expected_aspect.
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

        # Subdivide into a grid of n_major x n_minor rectangles aligned with the same angle.
        angle = float(rect_info['angle'])
        theta = np.deg2rad(angle)
        ux, uy = float(np.cos(theta)), float(np.sin(theta))          # major axis
        vx, vy = -uy, ux                                             # minor axis (perpendicular)

        sub_w = width / n_major
        sub_h = height / n_minor

        cx0, cy0 = float(rect_info['center'][0]), float(rect_info['center'][1])
        out = []

        for i in range(n_major):
            for j in range(n_minor):
                # Offsets are centered so the grid stays centered at the original rect.
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
        """Detect all Jenga blocks in the image"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detected_blocks = []
        
        for color_name in self.color_ranges.keys():
            mask = self.segment_color(hsv, color_name)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by area - Jenga blocks should have reasonable size
                if area < 500:  # Minimum area to filter noise
                    continue
                
                rect_info = self.find_aligned_rectangle(contour)
                # If the contour is a merged blob of multiple same-color blocks, split it
                # into multiple block-sized rectangles based on expected Jenga dimensions.
                rect_infos = self._split_rect_if_merged(rect_info)
                
                # Filter by solidity - block should be mostly filled
                hull = cv2.convexHull(contour)
                hull_area = cv2.contourArea(hull)
                if hull_area > 0:
                    solidity = area / hull_area
                    if solidity < 0.6:  # Less than 60% filled is likely not a block
                        continue

                # Split-aware per-rectangle block creation
                approx_area_each = float(area) / max(1, len(rect_infos))

                for ri in rect_infos:
                    width = float(ri['width'])
                    height = float(ri['height'])
                    if width <= 1e-6 or height <= 1e-6:
                        continue

                    aspect_ratio = width / height
                    # For split rectangles this should fall back into the normal range.
                    if aspect_ratio < 1.2 or aspect_ratio > 6:
                        continue

                    # Calculate distance using the LONGEST side
                    dist = 0.0
                    if self.focal_length is not None:
                        dist = self.calculate_distance(width)

                    # Convert 2D image center to 3D world coordinates
                    center_pixel = ri['center']
                    world_coords = self.pixel_to_3d_world(center_pixel[0], center_pixel[1], dist)
                    
                    # Transform to new coordinate frame if calibration matrix is available
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
                        'world_coords': world_coords,  # 3D coordinates with camera as origin
                        'new_frame_coords': new_frame_coords,  # 3D coordinates in new frame
                        'intersection_point': intersection,  # 2D pixel coordinates of intersection
                        'intersection_world_coords': intersection_world_coords,  # 3D camera frame coords
                        'intersection_new_frame_coords': intersection_new_frame_coords  # 3D new frame coords
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

            # Draw direction arrow along the block's length (away from bottom-of-frame observer)
            vx, vy = self._major_axis_unit_vector(block['angle'])
            vx, vy = self._direction_away_from_observer_bottom(vx, vy)
            arrow_len = int(max(30, min(0.6 * float(block['width']), 180)))
            end_pt = (
                int(round(block['center'][0] + vx * arrow_len)),
                int(round(block['center'][1] + vy * arrow_len)),
            )
            # Make arrow bold/visible: draw a dark outline first, then bright arrow on top
            cv2.arrowedLine(result, center, end_pt, (0, 0, 0), 10, tipLength=0.35)
            cv2.arrowedLine(result, center, end_pt, (255, 255, 0), 5, tipLength=0.35)
            
            # Draw perpendicular vector (90° anticlockwise) - already calculated in detect_blocks
            if block['intersection_point'] is not None:
                intersection = block['intersection_point']
                intersection_pt = (int(round(intersection[0])), int(round(intersection[1])))
                # Draw perpendicular vector from center to intersection
                cv2.arrowedLine(result, center, intersection_pt, (0, 0, 0), 8, tipLength=0.35)
                cv2.arrowedLine(result, center, intersection_pt, (0, 255, 255), 4, tipLength=0.35)  # Cyan color
                # Mark intersection point with a circle
                cv2.circle(result, intersection_pt, 6, (255, 0, 255), -1)  # Magenta circle
            
            # Text information: coordinates, color, and orientation
            text_lines = []

            if block['new_frame_coords'] is not None:
                coords = block['new_frame_coords']
                text_lines.append(f"({coords['x']:.1f},{coords['y']:.1f},{coords['z']:.1f})")
                text_lines.append(f"{block['color'].upper()}")
                text_lines.append(f"{block['angle']:.0f} DEG")
            else:
                # Fallback if calibration/new frame is not available
                text_lines.append("(N/A,N/A,N/A)")
                text_lines.append(f"{block['color'].upper()}")
                text_lines.append(f"{block['angle']:.0f} DEG")

            y_offset = -30
            for line in text_lines:
                cv2.putText(result, line, (center[0] - 80, center[1] + y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                y_offset += 15
        
        return result

# --- MAIN EXECUTION ---
if __name__ == "__main__":
    # Standard Jenga block length is 7.5 cm (Longest side)
    detector = JengaBlockDetector(real_block_length=7)
    
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
        
        # Store camera intrinsics for 3D coordinate conversion
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
        
        # Load calibration matrix for coordinate transformation
        detector.load_calibration_matrix("/home/arduino/Qualcomm-AI-Challenge/calibration/calibration_matrix.npy")
        
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
                    coords = b['world_coords']
                    new_coords = b['new_frame_coords']
                    # coord_str = f"Camera: X={coords['x']:.1f}, Y={coords['y']:.1f}, Z={coords['z']:.1f}cm"
                    # if new_coords is not None:
                    #     coord_str += f" | New Frame: X={new_coords['x']:.1f}, Y={new_coords['y']:.1f}, Z={new_coords['z']:.1f}cm"
                    # print(f"[{b['color']}] Distance: {b['distance']:.1f}cm | Angle: {b['angle']:.1f}° | {coord_str}")
                    
                    # Print intersection point coordinates
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