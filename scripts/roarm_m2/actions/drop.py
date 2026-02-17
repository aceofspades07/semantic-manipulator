"""Drop action for RoArm-M2 to open gripper and return to home."""

import os
import time
import importlib.util
from typing import Optional, Tuple


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


def drop(arm: Optional[object] = None,
         roarm_ip: str = "192.168.4.1",
         open_angle: float = 3.14,
         home_x: float = 40.0,
         home_y: float = 1.0,
         home_z: float = 53.0,
         home_t: float = 3.14,
         speed: float = 0.4) -> Tuple[bool, str]:
    """Open the gripper and move to home position."""
    try:
        if arm is None:
            RoArmController = _load_roarm_controller_class()
            arm = RoArmController(ip_address=roarm_ip)

        arm.set_torque(True)
    except Exception as e:
        return False, f"Failed to initialize arm: {e}"

    # Open gripper
    try:
        arm.set_joint(joint_id=4, angle=open_angle, wait=True)
    except Exception as e:
        return False, f"Failed to open gripper: {e}"

    time.sleep(1)

    # Move to home
    try:
        arm.move_cartesian(x=home_x, y=home_y, z=home_z, t=open_angle, speed=speed, wait=True)
    except Exception as e:
        return False, f"Failed to move to home: {e}"

    return True, "Dropped and moved to home"


if __name__ == "__main__":
    print("Running drop test...")
    success, msg = drop()
    print(f"Result: {success} - {msg}")
