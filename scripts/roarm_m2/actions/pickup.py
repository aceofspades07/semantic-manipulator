"""Pickup action for RoArm-M2 (Jenga Block Logic).

Modified to:
1. Open gripper.
2. Move to coordinates.
3. Close gripper (pickup).
4. Return to home position (while holding the object).
"""

import os
import time
import importlib.util
from typing import Dict, Iterable, List, Optional, Tuple

TABLE_HEIGHT = -120
BLOCK_HEIGHT = 25

def _load_roarm_controller_class():
    """Dynamically loads RoArmController from ../roarm_helper.py."""
    here = os.path.dirname(__file__)
    helper_path = os.path.normpath(os.path.join(here, "..", "roarm_helper.py"))
    
    spec = importlib.util.spec_from_file_location("roarm_helper", helper_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load roarm_helper from {helper_path}")
    
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return getattr(module, "RoArmController")


def _ensure_coordinate(coord: Iterable[float]) -> Tuple[float, float, Optional[float]]:
    """Normalize a coordinate iterable to (x, y, z_or_None)."""
    lst = list(coord)
    if len(lst) < 2:
        raise ValueError("Coordinate must contain at least x and y")
    x = float(lst[0])
    y = float(lst[1])
    z = float(lst[2]) if len(lst) >= 3 else None
    return x, y, z


def pickup(targets: Dict[str, List[Iterable[float]]],
           color: Optional[str] = None,
           arm: Optional[object] = None,
           roarm_ip: str = "192.168.4.1",
           open_angle: float = 1.57,
           close_angle: float = 3.14,
           approach_z_default: float = -110.0,
           grasp_z_default: float = -120.0,
           home_coords: Tuple[float, float, float] = (200.0, 0.0, 150.0)) -> Tuple[bool, str]:
    """Perform a pickup sequence and return home.

    Args:
        targets: Dictionary of coordinates.
        color: Specific color key to pick.
        arm: Optional existing controller instance.
        roarm_ip: IP address if arm needs to be initialized.
        open_angle: Servo angle for open gripper.
        close_angle: Servo angle for closed gripper.
        approach_z_default: Height to hover before lowering.
        grasp_z_default: Height to grab the object.
        home_coords: (x, y, z) to return to after picking up.
    """
    if not isinstance(targets, dict) or not targets:
        return False, "Targets must be a non-empty dict"

    # 1. Select Color
    if color is None:
        color = next(iter(targets.keys()))
    
    if color not in targets:
        return False, f"Color {color!r} not found in targets"

    coords_list = targets[color]
    if not coords_list:
        return False, f"No coordinates found for color {color!r}"

    # 2. Parse Coordinate
    try:
        x, y, z = _ensure_coordinate(coords_list[0])
    except Exception as e:
        return False, f"Invalid coordinate: {e}"

    # 3. Initialize Controller
    try:
        if arm is None:
            RoArmController = _load_roarm_controller_class()
            arm = RoArmController(ip_address=roarm_ip)
        
        # Ensure motors are enabled (Torque On)
        arm.set_torque(True)
    except Exception as e:
        return False, f"Failed to initialize arm: {e}"

    # Calculate Heights
    approach_z = (z + 10.0) if (z is not None) else approach_z_default
    grasp_z = (z - BLOCK_HEIGHT/2) if (z is not None) else grasp_z_default

    # --- ACTION SEQUENCE ---

    # Step 1: Open Gripper
    try:
        arm.set_joint(joint_id=4, angle=open_angle, wait=True)
    except Exception as e:
        return False, f"Failed to open gripper: {e}"

    # Step 2: Move to Approach (High)
    try:
        arm.move_cartesian(
            x=x, y=y, z=approach_z, 
            t=open_angle,  # Keep gripper open
            speed=0.4, wait=True
        )
    except Exception as e:
        return False, f"Failed approach move: {e}"

    time.sleep(0.1)

    # Step 3: Lower to Grasp (Low)
    try:
        arm.move_cartesian(
            x=x, y=y, z=grasp_z, 
            t=open_angle, # Keep gripper open while descending
            speed=0.2, wait=True
        )
    except Exception as e:
        return False, f"Failed to lower arm: {e}"

    # Step 4: Close Gripper (Grasp)
    try:
        arm.set_joint(joint_id=4, angle=close_angle, wait=True)
    except Exception as e:
        return False, f"Failed to close gripper: {e}"

    time.sleep(1.5) # Allow slightly more time for secure Jenga grip

    # Step 5: Vertical Safety Lift
    # We lift straight up first to avoid knocking over other blocks
    try:
        arm.move_cartesian(
            x=x, y=y, z=grasp_z, 
            t=close_angle, # KEEP gripper closed
            speed=0.3, wait=True
        )
    except Exception as e:
        return False, f"Failed to perform safety lift: {e}"
    
    time.sleep(1)

    # Step 6: Return to Home
    # Move to the defined home coordinates carrying the object
    try:
        hx, hy, hz = home_coords
        arm.move_cartesian(
            x=hx, y=hy, z=hz,
            t=close_angle, # CRITICAL: Maintain closed grip during home move
            speed=0.4, wait=True
        )
    except Exception as e:
        return False, f"Failed to return home: {e}"

    return True, f"Picked {color!r} and returned to home ({home_coords})"


if __name__ == "__main__":
    # Test configuration
    # Example Jenga block coordinate
    test_targets = {"jenga_block": [(200.0,0.0,-120.0)]} 
    
    print("Starting pickup test...")
    # Passing a specific home position (High and Center)
    success, message = pickup(test_targets, home_coords=(180, 0, 150))
    print(f"Result: {success}")
    print(f"Message: {message}")