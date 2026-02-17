"""RoArm-M2 controller with motion completion synchronization."""

import requests
import json
import time
import math
from typing import Optional, Dict, Any, Union

class RoArmController:
    """Controller for the RoArm-M2 that synchronizes execution with arm movement."""

    def __init__(self, ip_address: str, port: int = 80, protocol: str = "http", timeout: int = 10):
        self.base_url = f"{protocol}://{ip_address}:{port}/js?json="
        self.timeout = timeout
        self.last_response = None
        # Tolerance for motion completion detection
        self.motion_tolerance = 0.02 
        print(f"[RoArm] Initialized. Endpoint: {self.base_url}")

    def _send_command(self, command_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Send command and parse JSON acknowledgement."""
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
        """Query arm status and return current joint angles and coordinates."""
        cmd = {"T": 105}
        resp = self._send_command(cmd)
        return resp

    def wait_for_motion_completion(self, check_interval: float = 0.2, stability_required: int = 3):
        """Block until arm stops moving by polling position stability."""
        print("[RoArm] Waiting for motion to complete...", end="", flush=True)
        
        stable_count = 0
        last_values = {}
        
        start_time = time.time()
        
        while True:
            current_status = self.get_feedback()
            
            if not current_status:
                break

            # Extract position metrics
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
            
            # Check if stable
            if max_delta < self.motion_tolerance:
                stable_count += 1
            else:
                stable_count = 0
                
            # Done if stable for enough checks
            if stable_count >= stability_required:
                print(" Done.")
                break
                
            # Safety timeout
            if time.time() - start_time > 15:
                print(" Timeout (Movement took too long).")
                break

            last_values = current_values
            time.sleep(check_interval)


    def move_cartesian(self, x: float, y: float, z: float, t: float, speed: float = 0.25, wait: bool = True):
        """Move to X,Y,Z coords using inverse kinematics."""
        cmd = {"T": 104, "x": x, "y": y, "z": z, "t": t, "spd": speed}
        print(f"\n[RoArm] Moving Cartesian: {x}, {y}, {z}")
        self._send_command(cmd)
        if wait:
            self.wait_for_motion_completion()

    def set_joint(self, joint_id: int, angle: float, speed: float = 0.25, wait: bool = True):
        """Move single joint. 1=Base, 2=Shoulder, 3=Elbow, 4=Hand."""
        cmd = {"T": 101, "joint": joint_id, "angle": angle, "spd": speed}
        print(f"\n[RoArm] Moving Joint {joint_id} to {angle}")
        self._send_command(cmd)
        if wait:
            self.wait_for_motion_completion()

    def set_torque(self, enable: bool):
        """Enable or disable motors."""
        cmd = {"T": 210, "cmd": 1 if enable else 0}
        self._send_command(cmd)
        print(f"[RoArm] Torque set to {enable}")
        time.sleep(0.5)


if __name__ == "__main__":
    ROARM_IP = "192.168.4.1" 
    
    # Connect
    arm = RoArmController(ip_address=ROARM_IP)

    # Enable torque
    arm.set_torque(True)

    # Execute movement sequence
    try:
        # Move Home
        arm.move_cartesian(x=200, y=0, z=150, t=3.14, speed=0.5)

        # Move Left
        arm.move_cartesian(x=200, y=100, z=150, t=3.14, speed=0.5)

        # Move Right
        arm.move_cartesian(x=200, y=-100, z=150, t=3.14, speed=0.5)
        
        # Grab action
        arm.set_joint(joint_id=4, angle=3.14, wait=True)
        arm.set_joint(joint_id=4, angle=1.57, wait=True)

        # Return home
        arm.move_cartesian(x=200, y=0, z=150, t=3.14, speed=0.5)

    except KeyboardInterrupt:
        print("\n[RoArm] Stopping...")
        arm.set_torque(False)