"""Pickup action for RoArm-M2 to grab Jenga blocks and return to home."""

import os
import time
import importlib.util
from typing import Dict, Iterable, List, Optional, Tuple

TABLE_HEIGHT = -120
BLOCK_HEIGHT = 25

def _load_roarm_controller_class():
    """Dynamically loads RoArmController from roarm_helper.py."""
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
    """Perform a pickup sequence and return home."""
    if not isinstance(targets, dict) or not targets:
        return False, "Targets must be a non-empty dict"

    # Select color
    if color is None:
        color = next(iter(targets.keys()))
    
    if color not in targets:
        return False, f"Color {color!r} not found in targets"

    coords_list = targets[color]
    if not coords_list:
        return False, f"No coordinates found for color {color!r}"

    # Parse coordinate
    try:
        x, y, z = _ensure_coordinate(coords_list[0])
    except Exception as e:
        return False, f"Invalid coordinate: {e}"

    # Initialize controller
    try:
        if arm is None:
            RoArmController = _load_roarm_controller_class()
            arm = RoArmController(ip_address=roarm_ip)
        
        arm.set_torque(True)
    except Exception as e:
        return False, f"Failed to initialize arm: {e}"

    # Calculate heights
    approach_z = (z + 10.0) if (z is not None) else approach_z_default
    grasp_z = (z - BLOCK_HEIGHT/2) if (z is not None) else grasp_z_default

    # Open gripper
    try:
        arm.set_joint(joint_id=4, angle=open_angle, wait=True)
    except Exception as e:
        return False, f"Failed to open gripper: {e}"

    # Move to approach height
    try:
        arm.move_cartesian(
            x=x, y=y, z=approach_z, 
            t=open_angle,
            speed=0.4, wait=True
        )
    except Exception as e:
        return False, f"Failed approach move: {e}"

    time.sleep(0.1)

    # Lower to grasp height
    try:
        arm.move_cartesian(
            x=x, y=y, z=grasp_z, 
            t=open_angle,
            speed=0.2, wait=True
        )
    except Exception as e:
        return False, f"Failed to lower arm: {e}"

    # Close gripper to grasp
    try:
        arm.set_joint(joint_id=4, angle=close_angle, wait=True)
    except Exception as e:
        return False, f"Failed to close gripper: {e}"

    time.sleep(1.5)

    # Vertical safety lift to avoid knocking other blocks
    try:
        arm.move_cartesian(
            x=x, y=y, z=grasp_z, 
            t=close_angle,
            speed=0.3, wait=True
        )
    except Exception as e:
        return False, f"Failed to perform safety lift: {e}"
    
    time.sleep(1)

    # Return to home while holding the object
    try:
        hx, hy, hz = home_coords
        arm.move_cartesian(
            x=hx, y=hy, z=hz,
            t=close_angle,
            speed=0.4, wait=True
        )
    except Exception as e:
        return False, f"Failed to return home: {e}"

    return True, f"Picked {color!r} and returned to home ({home_coords})"


if __name__ == "__main__":
    test_targets = {"jenga_block": [(200.0,0.0,-120.0)]} 
    
    print("Starting pickup test...")
    success, message = pickup(test_targets, home_coords=(180, 0, 150))
    print(f"Result: {success}")
    print(f"Message: {message}")