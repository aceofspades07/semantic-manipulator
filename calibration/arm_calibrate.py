import cv2
import numpy as np
import pyrealsense2 as rs
import time
from typing import Optional, List, Tuple
import json
import requests

# ==========================================
# 1. ROBOT INTERFACE (Hardware Abstraction)
# ==========================================
def get_current_robot_position(tid: int) -> Tuple[float, float]:
    ip_addr = '192.168.4.1'
    command = {"T": 105}
    json_command = json.dumps(command)
    
    url = f"http://{ip_addr}/js?json={json_command}"
    try:
        try:
            response = requests.get(url, timeout=2)
            response.raise_for_status()
            
            # Parse the JSON response
            # Expected format example: {"T":105, "b":0, "s":0, "e":0, "h":0, "x":150, "y":0, "z":100, ...}
            data = json.loads(response.text)

            
            # Extract Position
            x = float(data.get('x', 0))
            y = float(data.get('y', 0))
            z = float(data.get('z', 0))

        except requests.exceptions.RequestException as err:
            print(f"Connection Error: {err}")
        except json.JSONDecodeError:
            print("Data Error: Could not decode JSON response.")
        
        # Poll every 0.2 seconds (adjust as needed)
        time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nStopped.")

    try:
        print(f"   [INFO] Retrieved position for ID: {tid} = ({x}, {y})")
        return x , y
    except (NameError, ValueError):
        print("   [ERROR] Could not retrieve robot position. Using mock input (0.0,0.0).")
        return 0.0 , 0.0   
           
# ==========================================
# 2. CALIBRATION CLASS
# ==========================================
class InteractiveCalibrator:
    def __init__(
        self,
        *,
        marker_ids: Tuple[int, int, int, int] = (0, 1, 2, 3),
        aruco_dict_name: int = cv2.aruco.DICT_4X4_50,
        save_path: str = "/home/karan/Qualcomm-AI-Challenge/calibration/calibration_matrix.npy",
    ) -> None:
        self.marker_ids = marker_ids
        self.save_path = save_path
        
        # Initialize ArUco detector
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_name)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        
        # This will hold the "Ground Truth" coordinates we teach it
        self.robot_coords: Optional[np.ndarray] = None

    def get_realsense_frame(self) -> np.ndarray:
        """Captures a single frame from the RealSense camera."""
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)

        # Warmup to allow auto-exposure to settle
        for _ in range(10):
            pipeline.wait_for_frames()

        try:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                raise RuntimeError("No color frame received from RealSense")
            image = np.asanyarray(color_frame.get_data())
        finally:
            pipeline.stop()

        return image

    def teach_robot_points(self) -> bool:
        """Interactive loop to query robot positions for each marker."""
        print("\n--- PHASE 1: ROBOT TEACHING ---")
        points_list = []

        for tid in self.marker_ids:
            print(f"\n[ACTION] Move robot tip to CENTER of Marker {tid}")
            input("Press ENTER when robot is in position...")
            
            # Hardware call
            rx, ry = get_current_robot_position(tid)
            
            print(f"   -> Recorded: ID {tid} = ({rx}, {ry})")
            points_list.append([rx, ry])

        self.robot_coords = np.array(points_list, dtype="float32")
        print("\n[SUCCESS] Robot coordinates recorded.")
        return True

    def compute_homography(self, image: np.ndarray) -> Optional[np.ndarray]:
        if self.robot_coords is None:
            raise ValueError("Robot coordinates not set. Run teach_robot_points() first.")

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self.detector.detectMarkers(gray)

        if ids is None or len(ids) < 4:
            print("Error: Less than 4 markers detected in camera view.")
            return None

        ids = ids.flatten()
        image_points: List[List[float]] = []

        # Match camera detections to the order of self.robot_coords (0, 1, 2, 3)
        try:
            for target_id in self.marker_ids:
                if target_id not in ids:
                    print(f"Error: Marker ID {target_id} missing from view.")
                    return None
                
                index = list(ids).index(target_id)
                marker_corners = corners[index][0]
                
                # Calculate center of the marker
                center_x = float(np.mean(marker_corners[:, 0]))
                center_y = float(np.mean(marker_corners[:, 1]))
                
                image_points.append([center_x, center_y])
                print(f"   -> Camera saw ID {target_id} at pixel {center_x:.1f}, {center_y:.1f}")
                
        except ValueError as e:
            print(f"Error processing markers: {e}")
            return None

        image_points_arr = np.array(image_points, dtype="float32")
        
        # Calculate the matrix
        return cv2.getPerspectiveTransform(image_points_arr, self.robot_coords)

    def run(self) -> None:
        # STEP 1: Interactive Teaching
        if not self.teach_robot_points():
            return

        # STEP 2: Clear the workspace
        print("\n--- PHASE 2: CAMERA CAPTURE ---")
        print("Please move the robot arm OUT of the camera's view. \n ALSO ensure that the Aruco markers are in place and clearly visible to the camera.")
        input("Press ENTER when the view is clear...")

        # STEP 3: Capture & Calibrate
        print("Capturing image...")
        image = self.get_realsense_frame()

        matrix = self.compute_homography(image)
        if matrix is None:
            print("Calibration failed.")
            return

        np.save(self.save_path, matrix)
        print(f"\n[SUCCESS] Calibration Matrix saved to '{self.save_path}'")


if __name__ == "__main__":
    # Initialize without hardcoded coords
    calibrator = InteractiveCalibrator()
    calibrator.run()