import gradio as gr
import time
import subprocess
import sys
import os
import requests
import json
import math
import cv2
import numpy as np
import pyrealsense2 as rs
from typing import Optional, Dict, Any, List, Tuple

# Import text classifier and action controller
from text_classifier import ActionClassifier
from roarm_m2.actions.control_action import ActionController
from colour_coordinates import ColourCoordinates

# ==========================================
# ROARM CONTROLLER CLASS
# ==========================================
class RoArmController:
    """
    An efficient controller for the RoArm-M2 that synchronizes Python execution 
    with physical arm movement.
    """

    def __init__(self, ip_address: str, port: int = 80, protocol: str = "http", timeout: int = 10):
        self.base_url = f"{protocol}://{ip_address}:{port}/js?json="
        self.timeout = timeout
        self.last_response = None
        # Tolerance for deciding if the arm has "stopped" (radians/mm change per check)
        self.motion_tolerance = 0.02 
        print(f"[RoArm] Initialized. Endpoint: {self.base_url}")

    def _send_command(self, command_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Sends command and parses the immediate JSON acknowledgement.
        """
        try:
            json_payload = json.dumps(command_dict)
            url = f"{self.base_url}{json_payload}"
            
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            
            try:
                data = response.json()
            except json.JSONDecodeError:
                # Fallback for raw text responses
                data = {"status": "ok", "raw": response.text}
            
            return data
        except Exception as e:
            print(f"[RoArm] Comm Error: {e}")
            return None

    def get_feedback(self) -> Optional[Dict[str, float]]:
        """
        Queries the arm's current status (T:105).
        Returns a dictionary of current joint angles/coordinates.
        """
        cmd = {"T": 105}
        resp = self._send_command(cmd)
        # RoArm usually returns keys like 'b', 's', 'e', 'h', 'x', 'y', 'z' in the response
        return resp

    def wait_for_motion_completion(self, check_interval: float = 0.2, stability_required: int = 3):
        """
        BLOCKS execution until the arm physically stops moving.
        
        Strategy: Poll status repeatedly. If the position hasn't changed 
        significantly for 'stability_required' checks in a row, we assume it stopped.
        """
        print("[RoArm] Waiting for motion to complete...", end="", flush=True)
        
        stable_count = 0
        last_values = {}
        
        start_time = time.time()
        
        while True:
            current_status = self.get_feedback()
            
            if not current_status:
                break # Comm failure, don't block indefinitely

            # Extract relevant movement metrics (joints b, s, e, h)
            # We filter for keys that are likely numeric position data
            current_values = {k: v for k, v in current_status.items() if k in ['b', 's', 'e', 'h', 'x', 'y', 'z'] and isinstance(v, (int, float))}
            
            if not last_values:
                last_values = current_values
                time.sleep(check_interval)
                continue

            # Calculate maximum change across all joints/axes
            max_delta = 0.0
            for key, val in current_values.items():
                prev_val = last_values.get(key, val)
                delta = abs(val - prev_val)
                if delta > max_delta:
                    max_delta = delta
            
            # Check if change is within "stopped" tolerance
            if max_delta < self.motion_tolerance:
                stable_count += 1
            else:
                stable_count = 0 # Reset if we detect movement
                
            # If stable for enough consecutive checks, we are done
            if stable_count >= stability_required:
                print(" Done.")
                break
                
            # Safety timeout (e.g., 15 seconds max wait)
            if time.time() - start_time > 15:
                print(" Timeout (Movement took too long).")
                break

            last_values = current_values
            time.sleep(check_interval)

    def move_cartesian(self, x: float, y: float, z: float, t: float, speed: float = 0.25, wait: bool = True):
        """
        Move to X,Y,Z coords (Inverse Kinematics).
        If wait=True, code blocks until move is finished.
        """
        cmd = {"T": 104, "x": x, "y": y, "z": z, "t": t, "spd": speed}
        print(f"\n[RoArm] Moving Cartesian: {x}, {y}, {z}")
        self._send_command(cmd)
        if wait:
            self.wait_for_motion_completion()

    def set_joint(self, joint_id: int, angle: float, speed: float = 0.25, wait: bool = True):
        """
        Move single joint. 1=Base, 2=Shoulder, 3=Elbow, 4=Hand.
        """
        cmd = {"T": 101, "joint": joint_id, "angle": angle, "spd": speed}
        print(f"\n[RoArm] Moving Joint {joint_id} to {angle}")
        self._send_command(cmd)
        if wait:
            self.wait_for_motion_completion()

    def set_torque(self, enable: bool):
        """Enables/Disables motors."""
        cmd = {"T": 210, "cmd": 1 if enable else 0}
        self._send_command(cmd)
        print(f"[RoArm] Torque set to {enable}")
        time.sleep(0.5) # Small buffer for hardware relay/activation


class RobotMock:
    """Robot object for teleop controls using RoArmController."""
    def __init__(self, ip_address: str = "192.168.4.1"):
        try:
            self.arm = RoArmController(ip_address=ip_address)
            self.arm.set_torque(True)
            self.use_real_arm = True
            print("[Robot] Real arm connected")
        except Exception as e:
            print(f"[Robot] Failed to connect to real arm: {e}. Using mock mode.")
            self.arm = None
            self.use_real_arm = False


# # ==========================================
# # INTERACTIVE CALIBRATION CLASS (for Gradio)
# # ==========================================
# class GradioCalibrator:
#     """
#     Interactive calibrator that works with Gradio UI instead of terminal input.
#     """
#     def __init__(
#         self,
#         marker_ids: Tuple[int, int, int, int] = (0, 1, 2, 3),
#         aruco_dict_name: int = cv2.aruco.DICT_4X4_50,
#         save_path: str = None,
#     ) -> None:
#         self.marker_ids = marker_ids
#         if save_path is None:
#             self.save_path = os.path.join(os.path.dirname(__file__), "calibration", "calibration_matrix.npy")
#         else:
#             self.save_path = save_path
        
#         # Initialize ArUco detector
#         aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_name)
#         parameters = cv2.aruco.DetectorParameters()
#         self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        
#         # Robot IP for getting positions
#         self.ip_addr = '192.168.4.1'
    
#     def get_current_robot_position(self) -> Tuple[float, float]:
#         """Get current robot position from the arm."""
#         command = {"T": 105}
#         json_command = json.dumps(command)
#         url = f"http://{self.ip_addr}/js?json={json_command}"
        
#         try:
#             response = requests.get(url, timeout=2)
#             response.raise_for_status()
#             data = json.loads(response.text)
#             x = float(data.get('x', 0))
#             y = float(data.get('y', 0))
#             return x, y
#         except requests.exceptions.RequestException as err:
#             print(f"Connection Error: {err}")
#             return 0.0, 0.0
#         except json.JSONDecodeError:
#             print("Data Error: Could not decode JSON response.")
#             return 0.0, 0.0
    
#     def get_realsense_frame(self) -> np.ndarray:
#         """Captures a single frame from the RealSense camera."""
#         pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#         pipeline.start(config)

#         # Warmup to allow auto-exposure to settle
#         for _ in range(10):
#             pipeline.wait_for_frames()

#         try:
#             frames = pipeline.wait_for_frames()
#             color_frame = frames.get_color_frame()
#             if not color_frame:
#                 raise RuntimeError("No color frame received from RealSense")
#             image = np.asanyarray(color_frame.get_data())
#         finally:
#             pipeline.stop()

#         return image
    
#     def compute_homography(self, image: np.ndarray, robot_coords: np.ndarray) -> Optional[np.ndarray]:
#         """Compute homography matrix from image and robot coordinates."""
#         gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         corners, ids, _rejected = self.detector.detectMarkers(gray)

#         if ids is None or len(ids) < 4:
#             return None, "Error: Less than 4 markers detected in camera view."

#         ids = ids.flatten()
#         image_points: List[List[float]] = []
#         log_messages = []

#         try:
#             for target_id in self.marker_ids:
#                 if target_id not in ids:
#                     return None, f"Error: Marker ID {target_id} missing from view."
                
#                 index = list(ids).index(target_id)
#                 marker_corners = corners[index][0]
                
#                 center_x = float(np.mean(marker_corners[:, 0]))
#                 center_y = float(np.mean(marker_corners[:, 1]))
                
#                 image_points.append([center_x, center_y])
#                 log_messages.append(f"   -> Camera saw ID {target_id} at pixel ({center_x:.1f}, {center_y:.1f})")
                
#         except ValueError as e:
#             return None, f"Error processing markers: {e}"

#         image_points_arr = np.array(image_points, dtype="float32")
#         matrix = cv2.getPerspectiveTransform(image_points_arr, robot_coords)
        
#         return matrix, "\n".join(log_messages)
    
#     def save_calibration(self, matrix: np.ndarray) -> bool:
#         """Save the calibration matrix to file."""
#         try:
#             # Ensure directory exists
#             os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
#             np.save(self.save_path, matrix)
#             return True
#         except Exception as e:
#             print(f"Error saving calibration: {e}")
#             return False


# # Global calibrator instance
# _gradio_calibrator = None

# def get_calibrator():
#     global _gradio_calibrator
#     if _gradio_calibrator is None:
#         _gradio_calibrator = GradioCalibrator()
#     return _gradio_calibrator


class RobotMockTeleop:
    """Robot object for teleop controls using RoArmController."""
    def __init__(self, ip_address: str = "192.168.4.1"):
        try:
            self.arm = RoArmController(ip_address=ip_address)
            self.arm.set_torque(True)
            self.use_real_arm = True
            print("[Robot] Real arm connected")
        except Exception as e:
            print(f"[Robot] Failed to connect to real arm: {e}. Using mock mode.")
            self.arm = None
            self.use_real_arm = False
    
    def teleop_move(self, direction: str) -> str:
        """Move robot based on direction."""
        if not self.use_real_arm or self.arm is None:
            return f"Moving {direction} (Mock)"
        
        try:
            # Define movement vectors for each direction
            movements = {
                'Forward': (50, 0, 0),
                'Backward': (-50, 0, 0),
                'Left': (0, 50, 0),
                'Right': (0, -50, 0),
                'Up': (0, 0, 50),
                'Down': (0, 0, -50),
            }
            
            if direction in movements:
                # Get current position
                feedback = self.arm.get_feedback()
                if feedback:
                    current_x = float(feedback.get('x', 0))
                    current_y = float(feedback.get('y', 0))
                    current_z = float(feedback.get('z', 0))
                    current_t = float(feedback.get('t', 3.14))
                    
                    # Apply movement
                    dx, dy, dz = movements[direction]
                    new_x = current_x + dx
                    new_y = current_y + dy
                    new_z = current_z + dz
                    
                    # Move arm
                    self.arm.move_cartesian(new_x, new_y, new_z, current_t, wait=False)
                    return f"Moving {direction}"
            
            return f"Invalid direction: {direction}"
        except Exception as e:
            print(f"[Robot] Movement error: {e}")
            return f"Movement failed: {e}"
    
    def drop_block(self) -> str:
        """Drop block by controlling gripper."""
        if not self.use_real_arm or self.arm is None:
            return "Block dropped (Mock)"
        
        try:
            # Joint 4 is the gripper
            # Open gripper (high angle)
            self.arm.set_joint(joint_id=4, angle=3.14, wait=True)
            time.sleep(0.5)
            # Close gripper (low angle)
            self.arm.set_joint(joint_id=4, angle=1.57, wait=True)
            return "Block dropped"
        except Exception as e:
            print(f"[Robot] Drop error: {e}")
            return f"Drop failed: {e}"

def system_logic():
    """
    Main application logic container.
    """
    
    # Define default state: Not calibrated, Not disabled
    # We use a dictionary to allow mutable state passing
    default_state = {
        "calibrated": False, 
        "disabled": False,
        # Calibration state
        "calib_active": False,
        "calib_step": 0,  # 0-3 for markers, 4 for camera capture, 5 for done
        "calib_points": [],  # List of (x, y) tuples
    }
    
    # Initialize robot - will try real arm first, fall back to mock
    robot = RobotMockTeleop(ip_address="192.168.4.1")
    
    # Initialize text classifier for command interpretation
    print("[System] Initializing action classifier...")
    action_classifier = ActionClassifier()
    action_classifier.train()
    print("[System] Action classifier ready")
    
    # Initialize action controller for robot commands
    print("[System] Initializing action controller...")
    action_controller = ActionController(roarm_ip="192.168.4.1")
    print("[System] Action controller ready")
    
    # Initialize color detector for picking objects
    print("[System] Initializing color detector...")
    color_detector = ColourCoordinates()
    print("[System] Color detector ready")
    
    # Map keyboard keys to robot commands
    teleop_commands = {
        'w': lambda: robot.teleop_move('Forward'),
        's': lambda: robot.teleop_move('Backward'),
        'a': lambda: robot.teleop_move('Left'),
        'd': lambda: robot.teleop_move('Right'),
        'u': lambda: robot.teleop_move('Up'),
        'j': lambda: robot.teleop_move('Down'),
        'o': lambda: robot.drop_block(),
    }

    def process_chat(user_message, history, state):
        """
        Handles chat interaction and inference generation.
        Processes user commands through the text classifier and executes robot actions.
        """
        # Ensure history is initialized
        if history is None:
            history = []

        # Normalize older tuple-format histories to messages format
        if len(history) > 0 and isinstance(history[0], (list, tuple)):
            normalized = []
            for user_msg, bot_msg in history:
                normalized.append({"role": "user", "content": user_msg})
                normalized.append({"role": "assistant", "content": bot_msg})
            history = normalized

        if state["disabled"]:
            # If system is disabled, prevent chat and return warning in messages format
            history.append({"role": "user", "content": user_message})
            history.append({"role": "assistant", "content": "SYSTEM DISABLED. MESSAGE REJECTED."})
            return history, "System is offline.", ""

        if not user_message.strip():
            return history, "", ""

        # Step 1: Classify the user message
        classification = action_classifier.predict(user_message)
        action = classification['action']
        color = classification['color']
        confidence = classification['confidence']
        
        # Build inference output
        inference_lines = [
            f"━━━━ COMMAND ANALYSIS ━━━━",
            f"Input: {user_message}",
            f"Action: {action.upper()}",
        ]
        
        if color:
            inference_lines.append(f"Color: {color.upper()}")
        
        inference_lines.append(f"Confidence: {confidence:.1%}")
        inference_lines.append(f"")
        
        # Step 2: Execute the action
        bot_response = ""
        
        if action == "none":
            bot_response = f"I couldn't identify a valid robot action from your message. I can help you with:\n• Pick [color] block\n• Place the block\n• Drop the block"
            inference_lines.append(f"Status: No valid action detected")
            
        elif action == "pick":
            if not color:
                bot_response = "Please specify a color for the pick action (e.g., 'pick the red block')"
                inference_lines.append(f"Status: Missing color parameter")
            else:
                inference_lines.append(f"Status: Detecting {color} objects...")
                
                try:
                    # Get coordinates of objects
                    targets = color_detector.capture()
                    
                    if color not in targets or len(targets[color]) == 0:
                        bot_response = f"No {color} objects detected. Please ensure the object is visible to the camera."
                        inference_lines.append(f"Result: No {color} objects found")
                    else:
                        # Execute pick action
                        inference_lines.append(f"Result: Found {len(targets[color])} {color} object(s)")
                        inference_lines.append(f"Status: Executing pick action...")
                        
                        success, msg = action_controller.execute_action(
                            action="pick",
                            targets=targets,
                            color=color
                        )
                        
                        if success:
                            bot_response = f"✓ Successfully picked {color} block! {msg}"
                            inference_lines.append(f"Execution: SUCCESS")
                        else:
                            bot_response = f"✗ Failed to pick {color} block: {msg}"
                            inference_lines.append(f"Execution: FAILED - {msg}")
                            
                except Exception as e:
                    bot_response = f"Error during pick operation: {str(e)}"
                    inference_lines.append(f"Error: {str(e)}")
                    
        elif action == "place":
            inference_lines.append(f"Status: Executing place action...")
            
            try:
                success, msg = action_controller.execute_action(action="place")
                
                if success:
                    bot_response = f"✓ Successfully placed the block! {msg}"
                    inference_lines.append(f"Execution: SUCCESS")
                else:
                    bot_response = f"✗ Failed to place block: {msg}"
                    inference_lines.append(f"Execution: FAILED - {msg}")
                    
            except Exception as e:
                bot_response = f"Error during place operation: {str(e)}"
                inference_lines.append(f"Error: {str(e)}")
                
        elif action == "drop":
            inference_lines.append(f"Status: Executing drop action...")
            
            try:
                success, msg = action_controller.execute_action(action="drop")
                
                if success:
                    bot_response = f"✓ Successfully dropped! {msg}"
                    inference_lines.append(f"Execution: SUCCESS")
                else:
                    bot_response = f"✗ Failed to drop: {msg}"
                    inference_lines.append(f"Execution: FAILED - {msg}")
                    
            except Exception as e:
                bot_response = f"Error during drop operation: {str(e)}"
                inference_lines.append(f"Error: {str(e)}")

        # Append messages in the dict format expected by newer Gradio versions
        history.append({"role": "user", "content": user_message})
        history.append({"role": "assistant", "content": bot_response})

        inference_data = "\n".join(inference_lines)
        return history, inference_data, ""

#     # ==========================================
#     # INTERACTIVE CALIBRATION FUNCTIONS
#     # ==========================================
    
#     def start_calibration(state):
#         """Start the interactive calibration process."""
#         if state["disabled"]:
#             return (
#                 "System Disabled. Cannot start calibration.",
#                 state,
#                 gr.update(visible=False),  # calib_panel
#                 gr.update(visible=True),   # calibrate_btn
#             )
        
#         # Initialize calibration state
#         state["calib_active"] = True
#         state["calib_step"] = 0
#         state["calib_points"] = []
        
#         prompt = """
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#    INTERACTIVE CALIBRATION - PHASE 1
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# STEP 1/4: Move robot tip to CENTER of Marker 0

# Position the robot arm so its tip is exactly 
#    at the center of ArUco Marker ID 0.

# When ready, click "Confirm Position" below.
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# """
#         return (
#             prompt,
#             state,
#             gr.update(visible=True),   # calib_panel
#             gr.update(visible=False),  # calibrate_btn
#         )
    
#     def confirm_marker_position(state):
#         """Confirm the current marker position and move to next step."""
#         if not state["calib_active"]:
#             return state, "Calibration not active.", gr.update(), gr.update()
        
#         calibrator = get_calibrator()
#         step = state["calib_step"]
#         marker_ids = (0, 1, 2, 3)
        
#         # Get current robot position
#         x, y = calibrator.get_current_robot_position()
#         state["calib_points"].append([x, y])
        
#         recorded_msg = f"Recorded Marker {marker_ids[step]}: X={x:.2f}, Y={y:.2f}\n\n"
        
#         # Move to next step
#         state["calib_step"] = step + 1
        
#         if state["calib_step"] < 4:
#             # More markers to record
#             next_marker = marker_ids[state["calib_step"]]
#             prompt = f"""
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#    INTERACTIVE CALIBRATION - PHASE 1
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# {recorded_msg}
# STEP {state["calib_step"] + 1}/4: Move robot tip to CENTER of Marker {next_marker}

# Position the robot arm so its tip is exactly 
#    at the center of ArUco Marker ID {next_marker}.

# When ready, click "Confirm Position" below.
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# Recorded positions so far:
# """
#             for i, (px, py) in enumerate(state["calib_points"]):
#                 prompt += f"   • Marker {i}: ({px:.2f}, {py:.2f})\n"
            
#             return (
#                 state,
#                 prompt,
#                 gr.update(visible=True, value="✅ Confirm Position"),  # confirm_btn
#                 gr.update(visible=False),  # capture_btn
#             )
#         else:
#             # All markers recorded, move to camera capture phase
#             prompt = f"""
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#    INTERACTIVE CALIBRATION - PHASE 2
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# {recorded_msg}
# All 4 marker positions recorded successfully!

# Recorded positions:
# """
#             for i, (px, py) in enumerate(state["calib_points"]):
#                 prompt += f"   • Marker {i}: ({px:.2f}, {py:.2f})\n"
            
#             prompt += """
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# IMPORTANT: Before capturing the camera image:

#    1. Move the robot arm OUT of the camera's view
#    2. Ensure all 4 ArUco markers are clearly visible
#    3. Make sure lighting is adequate

# When ready, click "Capture & Calibrate" below.
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# """
#             return (
#                 state,
#                 prompt,
#                 gr.update(visible=False),  # confirm_btn
#                 gr.update(visible=True),   # capture_btn
#             )
    
#     def capture_and_calibrate(state):
#         """Capture camera image and compute calibration matrix."""
#         if not state["calib_active"] or len(state["calib_points"]) < 4:
#             return (
#                 state,
#                 "Error: Not enough calibration points recorded.",
#                 gr.update(visible=False),  # calib_panel
#                 gr.update(visible=True),   # calibrate_btn
#             )
        
#         calibrator = get_calibrator()
#         robot_coords = np.array(state["calib_points"], dtype="float32")
        
#         prompt = """
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#    CAPTURING IMAGE...
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# """
        
#         try:
#             # Capture image from RealSense
#             image = calibrator.get_realsense_frame()
#             prompt += "\nImage captured successfully!\n\n"
#             prompt += "Computing homography matrix...\n\n"
            
#             # Compute homography
#             matrix, log_msg = calibrator.compute_homography(image, robot_coords)
            
#             if matrix is None:
#                 # Calibration failed
#                 state["calib_active"] = False
#                 state["calib_step"] = 0
#                 state["calib_points"] = []
                
#                 prompt += f"""
# CALIBRATION FAILED

# {log_msg}

# Please ensure:
# • All 4 ArUco markers (ID 0-3) are visible
# • Markers are not obstructed
# • Lighting is adequate
# • Camera is properly connected

# Click "Send Calibration Signal" to try again.
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# """
#                 return (
#                     state,
#                     prompt,
#                     gr.update(visible=False),  # calib_panel
#                     gr.update(visible=True),   # calibrate_btn
#                 )
            
#             # Save calibration matrix
#             if calibrator.save_calibration(matrix):
#                 state["calibrated"] = True
#                 state["calib_active"] = False
#                 state["calib_step"] = 0
#                 state["calib_points"] = []
                
#                 timestamp = time.strftime("%H:%M:%S")
#                 prompt += f"""
# {log_msg}

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#    CALIBRATION COMPLETE!
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# Homography matrix computed and saved!

# File: calibration/calibration_matrix.npy
# Time: {timestamp}

# The system is now calibrated and ready for use.
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# """
#                 return (
#                     state,
#                     prompt,
#                     gr.update(visible=False),  # calib_panel
#                     gr.update(visible=False, interactive=False),  # calibrate_btn (disabled after success)
#                 )
#             else:
#                 prompt += "\nFailed to save calibration matrix."
                
#         except Exception as e:
#             prompt += f"\nError during calibration: {str(e)}"
#             state["calib_active"] = False
        
#         return (
#             state,
#             prompt,
#             gr.update(visible=False),  # calib_panel
#             gr.update(visible=True),   # calibrate_btn
#         )
    
#     def cancel_calibration(state):
#         """Cancel the ongoing calibration process."""
#         state["calib_active"] = False
#         state["calib_step"] = 0
#         state["calib_points"] = []
        
#         return (
#             state,
#             "Calibration cancelled by user.",
#             gr.update(visible=False),  # calib_panel
#             gr.update(visible=True),   # calibrate_btn
#         )

#     def run_calibration(state):
#         """
#         Runs the external `arm_calibrate.py` script and waits for it to finish.
#         """
#         if state["disabled"]:
#             return "System Disabled. Calibration Failed.", state
#         try:
#             script_path = os.path.join(os.path.dirname(__file__), "calibration", "arm_calibrate.py")
#             # Run the calibration script using the same Python executable and wait for completion
#             result = subprocess.run([sys.executable, script_path], check=True, capture_output=True, text=True)
#             state["calibrated"] = True
#             timestamp = time.strftime("%H:%M:%S")
#             stdout = result.stdout.strip()
#             if stdout:
#                 return f"System Calibrated at {timestamp}\n\n{stdout}", state
#             return f"System Calibrated at {timestamp}", state
#         except subprocess.CalledProcessError as e:
#             err = e.stderr.strip() if e.stderr else str(e)
#             return f"Calibration script failed: {err}", state
#         except Exception as e:
#             return f"Calibration failed: {e}", state

    # def check_calibration_file_exists():
    #     """
    #     Checks if the calibration matrix file exists in the calibration directory.
    #     """
    #     calib_file = os.path.join(os.path.dirname(__file__), "calibration", "calibration_matrix.npy")
    #     return os.path.isfile(calib_file)

    # def handle_signal(signal_code, state):
    #     """
    #     Handles the specific signal to disable functionality (Feature 4).
    #     Specific Signal is: 'STOP'
    #     """
    #     if signal_code.strip().upper() == "STOP":
    #         state["disabled"] = True
            
    #         # Return values to update UI components:
    #         # 1. Status Message
    #         # 2. State
    #         # 3. Chat Input (interactive=False)
    #         # 4. Calibrate Button (interactive=False)
    #         # 5. Signal Button (interactive=False)
    #         return (
    #             "SYSTEM SHUTDOWN SIGNAL RECEIVED. ALL OPERATIONS CEASED.", 
    #             state,
    #             gr.update(interactive=False, placeholder="System Disabled"),
    #             gr.update(interactive=False, value="Disabled"),
    #             gr.update(interactive=False)
    #         )
        
    #     return f"Signal '{signal_code}' ignored.", state, gr.update(), gr.update(), gr.update()

    def execute_teleop_command(key_or_direction):
        """
        Execute teleop command based on key or direction button press.
        """
        if key_or_direction in teleop_commands:
            result = teleop_commands[key_or_direction]()
            return f"{result}"
        return "Invalid command"

    # --- GUI Layout ---
    with gr.Blocks(title="Control Interface") as demo:
        # State variable to hold system status across interactions within a session
        system_state = gr.State(default_state)
        
        gr.Markdown("# System Control Interface")
        
        with gr.Row():
            with gr.Column(scale=2):
                # Feature 1: Chatbot
                # Removed 'type="messages"' to fix TypeError. 
                # This component will now expect [[user_msg, bot_msg], ...] format.
                chatbot = gr.Chatbot(label="Conversation Log", height=400)
                msg_input = gr.Textbox(
                    label="User Input", 
                    placeholder="Type a message...",
                    interactive=True
                )
                clear = gr.ClearButton([msg_input, chatbot])

            with gr.Column(scale=1):
                # Feature 2 & 4: Controls and Status
                gr.Markdown("### System Status")
                # Determine whether calibration file exists; if it does, log and disable the button
                calib_file = os.path.join(os.path.dirname(__file__), "calibration", "calibration_matrix.npy")
                file_exists = os.path.isfile(calib_file)
                if file_exists:
                    print(f"[Calibration] Detected calibration file: {calib_file}")
                    calibrate_btn = gr.Button("Send Calibration Signal", variant="primary", interactive=False)
                else:
                    print(f"[Calibration] No calibration file found; enabling calibration button")
                    calibrate_btn = gr.Button("Send Calibration Signal", variant="primary", interactive=True)
                
                # Feature 3: Inference Display (also used for calibration prompts)
                inference_output = gr.TextArea(
                    label="Inference / Processing Output", 
                    interactive=False,
                    lines=12
                )
                
                # # Interactive Calibration Panel (hidden by default)
                # with gr.Group(visible=False) as calib_panel:
                #     gr.Markdown("### Calibration Controls")
                #     with gr.Row():
                #         confirm_btn = gr.Button("Confirm Position", variant="primary", size="lg")
                #         capture_btn = gr.Button("Capture & Calibrate", variant="primary", visible=False, size="lg")
                #     cancel_btn = gr.Button("Cancel Calibration", variant="stop", size="sm")

                # gr.Markdown("---")
                # gr.Markdown("### Admin Controls")
                
                # # Feature 4: Disable Functionality
                # signal_input = gr.Textbox(
                #     label="Control Signal", 
                #     placeholder="Enter signal code (e.g. STOP)"
                # )
                # signal_btn = gr.Button("Transmit Signal", variant="stop")

        # --- Robot Teleop Control Panel ---
        gr.Markdown("### Robot Teleop Controls")
        gr.Markdown("**Keyboard:** W/A/S/D (Move), U (Up), J (Down), O (Drop)")
        
        with gr.Row():
            teleop_forward = gr.Button("W", size="lg")
            teleop_output = gr.Textbox(label="Command Output", interactive=False, lines=2)
        
        with gr.Row():
            teleop_left = gr.Button("A", size="lg")
            teleop_down = gr.Button("S", size="lg")
            teleop_right = gr.Button("D", size="lg")
        
        with gr.Row():
            teleop_up = gr.Button("U", size="lg")
            teleop_drop = gr.Button("O", size="lg")

        # --- Event Wiring ---

        # 1. Chat Interaction
        msg_input.submit(
            process_chat, 
            inputs=[msg_input, chatbot, system_state], 
            outputs=[chatbot, inference_output, msg_input]
        )

        # # 2. Interactive Calibration - Start
        # calibrate_btn.click(
        #     start_calibration,
        #     inputs=[system_state],
        #     outputs=[inference_output, system_state, calib_panel, calibrate_btn]
        # )
        
        # # 2a. Calibration - Confirm marker position
        # confirm_btn.click(
        #     confirm_marker_position,
        #     inputs=[system_state],
        #     outputs=[system_state, inference_output, confirm_btn, capture_btn]
        # )
        
        # # 2b. Calibration - Capture and compute
        # capture_btn.click(
        #     capture_and_calibrate,
        #     inputs=[system_state],
        #     outputs=[system_state, inference_output, calib_panel, calibrate_btn]
        # )
        
        # # 2c. Calibration - Cancel
        # cancel_btn.click(
        #     cancel_calibration,
        #     inputs=[system_state],
        #     outputs=[system_state, inference_output, calib_panel, calibrate_btn]
        # )

        # # 4. Signal Handling (Disable functionality)
        # signal_btn.click(
        #     handle_signal,
        #     inputs=[signal_input, system_state],
        #     outputs=[
        #         inference_output, # Update status area
        #         system_state,     # Update internal state
        #         msg_input,        # Disable chat input
        #         calibrate_btn,    # Disable calibrate button
        #         signal_btn        # Disable signal button itself
        #     ]
        # )

        # 5. Teleop Button Controls
        teleop_forward.click(execute_teleop_command, inputs=gr.State('w'), outputs=teleop_output)
        teleop_left.click(execute_teleop_command, inputs=gr.State('a'), outputs=teleop_output)
        teleop_down.click(execute_teleop_command, inputs=gr.State('s'), outputs=teleop_output)
        teleop_right.click(execute_teleop_command, inputs=gr.State('d'), outputs=teleop_output)
        teleop_up.click(execute_teleop_command, inputs=gr.State('u'), outputs=teleop_output)
        teleop_drop.click(execute_teleop_command, inputs=gr.State('o'), outputs=teleop_output)

        # 6. Keyboard input handler with JavaScript
        def register_keyboard_handler():
            return """
            <script>
            document.addEventListener('keydown', (e) => {
                const key = e.key.toLowerCase();
                const commands = ['w', 'a', 's', 'd', 'u', 'j', 'o'];
                if (commands.includes(key)) {
                    const buttons = document.querySelectorAll('[data-testid="button"]');
                    const buttonMap = {
                        'w': 0,
                        'a': 1,
                        's': 2,
                        'd': 3,
                        'u': 4,
                        'j': 5,
                        'o': 6
                    };
                    if (buttonMap[key] !== undefined && buttons[buttonMap[key]]) {
                        buttons[buttonMap[key]].click();
                    }
                }
            });
            </script>
            """
        
        demo.load(lambda: None, outputs=gr.HTML(register_keyboard_handler()))

    return demo

if __name__ == "__main__":
    app = system_logic()
    app.launch()