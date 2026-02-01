"""Place action for RoArm-M2.

Translates the arm to the middle position, lowers to -120mm,
opens the gripper, and returns to home without closing the gripper.
"""

import os
import time
import importlib.util
from typing import Optional, Tuple


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


def place(arm: Optional[object] = None,
          roarm_ip: str = "192.168.4.1",
          open_angle: float = 3.14,
          middle_x: float = 200.0,
          middle_y: float = 0.0,
          middle_z: float = 150.0,
          place_z: float = -120.0,
          place_t: float = 3.14,
          home_x: float = 200.0,
          home_y: float = 0.0,
          home_z: float = 150.0,
          home_t: float = 3.14,
          speed: float = 0.4) -> Tuple[bool, str]:
    """Place the object at middle position then return to home.

    Sequence:
    1. Move to middle position (approach height)
    2. Lower arm to -120mm (z coordinate)
    3. Open gripper
    4. Return to home position without closing gripper

    Returns (success, message).
    """
    try:
        if arm is None:
            RoArmController = _load_roarm_controller_class()
            arm = RoArmController(ip_address=roarm_ip)

        # Ensure motors are enabled
        arm.set_torque(True)
    except Exception as e:
        return False, f"Failed to initialize arm: {e}"

    # Step 1: Move to middle position (approach height)
    try:
        arm.move_cartesian(x=middle_x, y=middle_y, z=middle_z, t=place_t, speed=speed, wait=True)
    except Exception as e:
        return False, f"Failed to move to middle position: {e}"

    time.sleep(0.1)

    # Step 2: Lower the arm to -120mm
    try:
        arm.move_cartesian(x=middle_x, y=middle_y, z=place_z, t=place_t, speed=0.2, wait=True)
    except Exception as e:
        return False, f"Failed to lower arm: {e}"

    time.sleep(0.2)

    # Step 3: Open the gripper
    try:
        arm.set_joint(joint_id=4, angle=open_angle, wait=True)
    except Exception as e:
        return False, f"Failed to open gripper: {e}"

    time.sleep(0.1)

    # Step 4: Return to home position without closing gripper
    try:
        arm.move_cartesian(x=home_x, y=home_y, z=home_z, t=open_angle, speed=speed, wait=True)
    except Exception as e:
        return False, f"Failed to return to home: {e}"

    return True, "Object placed successfully"


if __name__ == "__main__":
    print("Running place test...")
    success, msg = place()
    print(f"Result: {success} - {msg}")
