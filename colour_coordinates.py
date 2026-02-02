"""ColourCoordinates class for capturing camera frames and producing
colour -> coordinates dictionary using the detect_jenga detector.

Class provided:
- `ColourCoordinates` - Main class for coordinate detection

Usage:
    from colour_coordinates import ColourCoordinates
    
    # Initialize the detector
    detector = ColourCoordinates()
    
    # Capture and get all coordinates
    coords = detector.capture()
    
    # Get coordinates for a specific color
    red_coords = detector.get_coordinates_by_color("red")
    
    # Get first coordinate for a color (for end-effector target)
    target = detector.get_target_coordinate("red")

Dictionary format:
{
    'red': [(x1, y1, z1), (x2, y2, z2), ...],
    'blue': [(x3, y3, z3), ...],
    ...
}
Each color appears as a key only once, with all detected blocks of that color
in the associated list.
"""

import os
import time
import importlib.util
from typing import Optional, Dict, List, Tuple
import numpy as np

try:
    import pyrealsense2 as rs
except Exception:
    rs = None


class ColourCoordinates:
    """Class to capture camera frames and detect block coordinates by color.
    
    This class provides methods to:
    - Capture frames from RealSense camera
    - Detect Jenga blocks and their colors
    - Retrieve coordinates for robotic arm end-effector positioning
    """
    
    def __init__(
        self,
        arm: Optional[object] = None,
        roarm_ip: str = "192.168.4.1",
        home_x: float = 40.0,
        home_y: float = 1.0,
        home_z: float = 53.0,
        home_t: float = 3.14,
        speed: float = 0.4,
        rs_timeout: float = 5.0,
        calibration_path: str = "/home/arduino/Qualcomm-AI-Challenge/calibration/calibration_matrix.npy"
    ):
        """Initialize the ColourCoordinates detector.
        
        Args:
            arm: Optional existing RoArm controller instance
            roarm_ip: IP address of the robotic arm
            home_x: Home position X coordinate
            home_y: Home position Y coordinate
            home_z: Home position Z coordinate
            home_t: Home position T (gripper) angle
            speed: Movement speed for arm
            rs_timeout: Timeout for RealSense frame capture
            calibration_path: Path to calibration matrix file
        """
        self.arm = arm
        self.roarm_ip = roarm_ip
        self.home_x = home_x
        self.home_y = home_y
        self.home_z = home_z
        self.home_t = home_t
        self.speed = speed
        self.rs_timeout = rs_timeout
        self.calibration_path = calibration_path
        
        # Cached coordinates from last capture
        self._coordinates: Dict[str, List[Tuple[float, float, float]]] = {}
        
        # Detector and controller classes (lazy loaded)
        self._RoArmController = None
        self._JengaBlockDetector = None
    
    def _load_roarm_controller_class(self):
        """Dynamically loads RoArmController from roarm_helper.py."""
        if self._RoArmController is not None:
            return self._RoArmController
            
        here = os.path.dirname(__file__)
        helper_path = os.path.normpath(os.path.join(here, "roarm_m2", "roarm_helper.py"))

        spec = importlib.util.spec_from_file_location("roarm_helper", helper_path)
        if spec is None or spec.loader is None:
            raise ImportError(f"Cannot load roarm_helper from {helper_path}")

        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self._RoArmController = getattr(module, "RoArmController")
        return self._RoArmController

    def _load_detector_class(self):
        """Dynamically loads JengaBlockDetector from detect_jenga.py."""
        if self._JengaBlockDetector is not None:
            return self._JengaBlockDetector
            
        here = os.path.dirname(__file__)
        detector_path = os.path.normpath(os.path.join(here, "detect_jenga.py"))

        spec = importlib.util.spec_from_file_location("detect_jenga", detector_path)
        if spec is None or spec.loader is None:
            raise ImportError(f"Cannot load detect_jenga from {detector_path}")

        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self._JengaBlockDetector = getattr(module, "JengaBlockDetector")
        return self._JengaBlockDetector

    def move_arm_to_home(self) -> bool:
        """Move the robotic arm to the home position.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            if self.arm is None:
                RoArmController = self._load_roarm_controller_class()
                self.arm = RoArmController(ip_address=self.roarm_ip)

            self.arm.set_torque(True)
            self.arm.move_cartesian(
                x=self.home_x, 
                y=self.home_y, 
                z=self.home_z, 
                t=self.home_t, 
                speed=self.speed, 
                wait=True
            )
            return True
        except Exception as e:
            print(f"[ColourCoordinates] Warning: failed to move arm to home: {e}")
            return False

    def capture(self, move_to_home: bool = True) -> Dict[str, List[Tuple[float, float, float]]]:
        """Capture a frame and detect block coordinates.
        
        Args:
            move_to_home: Whether to move arm to home position before capture
        
        Returns:
            Dictionary mapping color name -> list of (x, y, z) tuples
        """
        # 1) Optionally move arm to home
        if move_to_home:
            self.move_arm_to_home()

        # 2) Check RealSense availability
        if rs is None:
            raise RuntimeError("pyrealsense2 is not available in this environment")

        # 3) Load detector
        try:
            JengaBlockDetector = self._load_detector_class()
            detector = JengaBlockDetector()
        except Exception as e:
            raise RuntimeError(f"Failed to load JengaBlockDetector: {e}")

        # 4) Capture frames using RealSense
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        profile = pipeline.start(config)
        try:
            # Obtain intrinsics and set detector focal length
            color_stream = profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            detector.focal_length = intrinsics.fx
            detector.camera_intrinsics = {
                'fx': intrinsics.fx,
                'fy': intrinsics.fy,
                'ppx': intrinsics.ppx,
                'ppy': intrinsics.ppy,
            }

            # Load calibration matrix to transform to robot frame
            detector.load_calibration_matrix(self.calibration_path)

            # Wait for and collect 60 color frames before running detector
            start = time.time()
            color_frame = None
            frame_count = 0
            while (time.time() - start) < self.rs_timeout and frame_count < 60:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                cf = frames.get_color_frame()
                if cf:
                    color_frame = cf
                    frame_count += 1

            if frame_count < 60:
                raise RuntimeError(f"Failed to capture 60 frames; only captured {frame_count} within timeout")

            frame = np.asanyarray(color_frame.get_data())

            # Run detection
            blocks = detector.detect_blocks(frame)

        finally:
            pipeline.stop()

        # 5) Build dictionary: color -> list of coordinates
        self._coordinates = {}

        for block in blocks:
            color = block.get('color') or 'unknown'

            # Prefer new_frame_coords (calibrated robot frame), fall back to world_coords
            coords = block.get('intersection_new_frame_coords') or block.get('intersection_world_coords')

            if coords is None:
                continue

            # Extract x, y, z safely
            try:
                x = float(coords.get('x', 0))
                y = float(coords.get('y', 0))
                z = float(coords.get('z', 0))
            except (ValueError, TypeError):
                continue

            # Add to dictionary
            self._coordinates.setdefault(color, []).append((x, y, z))

        return self._coordinates

    def get_all_coordinates(self) -> Dict[str, List[Tuple[float, float, float]]]:
        """Get all cached coordinates from the last capture.
        
        Returns:
            Dictionary mapping color name -> list of (x, y, z) tuples
        """
        return self._coordinates.copy()

    def get_coordinates_by_color(self, color: str) -> List[Tuple[float, float, float]]:
        """Get all coordinates for a specific color.
        
        Args:
            color: The color name to retrieve coordinates for
        
        Returns:
            List of (x, y, z) tuples for the specified color, empty list if not found
        """
        return self._coordinates.get(color, []).copy()

    def get_target_coordinate(
        self, 
        color: str, 
        index: int = 0
    ) -> Optional[Tuple[float, float, float]]:
        """Get a specific coordinate for the end-effector target.
        
        Args:
            color: The color name to retrieve coordinate for
            index: Index of the block (0 for first block of that color)
        
        Returns:
            (x, y, z) tuple for the target, or None if not found
        """
        coords = self._coordinates.get(color, [])
        if index < len(coords):
            return coords[index]
        return None

    def get_available_colors(self) -> List[str]:
        """Get list of colors that have detected blocks.
        
        Returns:
            List of color names
        """
        return list(self._coordinates.keys())

    def get_block_count(self, color: Optional[str] = None) -> int:
        """Get the count of detected blocks.
        
        Args:
            color: Optional color to count. If None, returns total count.
        
        Returns:
            Number of blocks detected
        """
        if color is not None:
            return len(self._coordinates.get(color, []))
        return sum(len(coords) for coords in self._coordinates.values())

    def refresh(self) -> Dict[str, List[Tuple[float, float, float]]]:
        """Refresh coordinates by capturing a new frame.
        
        Alias for capture() method.
        
        Returns:
            Dictionary mapping color name -> list of (x, y, z) tuples
        """
        return self.capture()


# Backwards compatibility: standalone function that uses the class
def capture_colour_coordinates(
    arm: Optional[object] = None,
    roarm_ip: str = "192.168.4.1",
    home_x: float = 40.0,
    home_y: float = 1.0,
    home_z: float = 53.0,
    home_t: float = 3.14,
    speed: float = 0.4,
    rs_timeout: float = 5.0,
) -> Dict[str, List[Tuple[float, float, float]]]:
    """Legacy function for backwards compatibility.
    
    Consider using the ColourCoordinates class instead.
    """
    detector = ColourCoordinates(
        arm=arm,
        roarm_ip=roarm_ip,
        home_x=home_x,
        home_y=home_y,
        home_z=home_z,
        home_t=home_t,
        speed=speed,
        rs_timeout=rs_timeout
    )
    return detector.capture()


if __name__ == "__main__":
    print("Capturing colour coordinates...")
    
    # Using the class
    detector = ColourCoordinates()
    coords = detector.capture()
    
    print("\nDetected blocks by colour:")
    for colour in detector.get_available_colors():
        coord_list = detector.get_coordinates_by_color(colour)
        print(f"  {colour}: {len(coord_list)} block(s)")
        for i, (x, y, z) in enumerate(coord_list, 1):
            print(f"    Block {i}: ({x:.1f}, {y:.1f}, {z:.1f})")
    
    print(f"\nTotal blocks detected: {detector.get_block_count()}")
    
    # Example: Get target coordinate for end-effector
    if detector.get_available_colors():
        first_color = detector.get_available_colors()[0]
        target = detector.get_target_coordinate(first_color)
        if target:
            print(f"\nExample target for '{first_color}': {target}")
