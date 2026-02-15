import requests
import json
import time
import math
from typing import Optional, Dict, Any, Union

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

    # =========================================================================
    # MOVEMENT FUNCTIONS (Now with 'wait' argument)
    # =========================================================================

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

# =============================================================================
# EFFICIENT EXECUTION FLOW
# =============================================================================
if __name__ == "__main__":
    ROARM_IP = "192.168.4.1" 
    
    # 1. Connect
    arm = RoArmController(ip_address=ROARM_IP)

    # 2. Ensure Torque is ON
    arm.set_torque(True)

    # 3. Define a sequence of tasks (The efficient part)
    # Because 'wait=True' is default, these will execute back-to-back perfectly
    # without any manual 'time.sleep()' guessing.
    
    try:
        # Move Home
        arm.move_cartesian(x=200, y=0, z=150, t=3.14, speed=0.5)

        # Move Left
        arm.move_cartesian(x=200, y=100, z=150, t=3.14, speed=0.5)

        # Move Right
        arm.move_cartesian(x=200, y=-100, z=150, t=3.14, speed=0.5)
        
        # Grab Action (Joint 4)
        arm.set_joint(joint_id=4, angle=3.14, wait=True) # Open
        arm.set_joint(joint_id=4, angle=1.57, wait=True) # Close

        # Return Home
        arm.move_cartesian(x=200, y=0, z=150, t=3.14, speed=0.5)

    except KeyboardInterrupt:
        print("\n[RoArm] Stopping...")
        arm.set_torque(False)