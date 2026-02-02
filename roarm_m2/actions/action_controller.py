"""Action Controller for RoArm-M2.

Coordinates pick/place/drop actions with FSM state validation.
Receives a dictionary of coordinates and executes requested actions.
"""

import os
import sys
from typing import Dict, List, Iterable, Tuple, Optional

# Add parent directories to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, parent_dir)

# Import action modules
from roarm_m2.actions.pickup import pickup
from roarm_m2.actions.place import place
from roarm_m2.actions.drop import drop

# Import FSM controller
from fsm_controller import fsm_controller


class ActionController:
    """Manages robotic arm actions with state validation."""
    
    def __init__(self, arm: Optional[object] = None, roarm_ip: str = "192.168.4.1"):
        """Initialize the action controller.
        
        Args:
            arm: Optional existing RoArm controller instance
            roarm_ip: IP address of the robotic arm
        """
        self.arm = arm
        self.roarm_ip = roarm_ip
        self.current_state = "doesnot_have_block"  # Initial state
    
    def execute_action(self, 
                      action: str, 
                      targets: Dict[str, List[Iterable[float]]], 
                      color: Optional[str] = None) -> Tuple[bool, str]:
        """Execute the requested action with FSM validation.
        
        Args:
            action: Action to perform - 'pick', 'place', or 'drop'
            targets: Dictionary with colors as keys and coordinate lists as values
            color: Specific color key to operate on (optional for place/drop)
        
        Returns:
            Tuple of (success: bool, message: str)
        """
        if not isinstance(action, str):
            return False, "Action must be a string"
        
        action = action.strip().lower()
        
        if action not in {"pick", "place", "drop"}:
            return False, f"Invalid action: {action}. Must be 'pick', 'place', or 'drop'"
        
        # Validate action with FSM controller
        try:
            new_state, fsm_message = fsm_controller(action, self.current_state)
        except Exception as e:
            return False, f"FSM validation error: {e}"
        
        # Check if FSM returned a no-op
        if "no-op" in fsm_message.lower():
            return False, f"Action not allowed: {fsm_message}"
        
        # FSM approved the action, proceed with execution
        success = False
        result_message = ""
        
        try:
            if action == "pick":
                # Pickup requires targets dictionary and color
                if not targets:
                    return False, "Pick action requires non-empty targets dictionary"
                success, result_message = pickup(
                    targets=targets,
                    color=color,
                    arm=self.arm,
                    roarm_ip=self.roarm_ip
                )
            
            elif action == "place":
                # Place action
                success, result_message = place(
                    arm=self.arm,
                    roarm_ip=self.roarm_ip
                )
            
            elif action == "drop":
                # Drop action
                success, result_message = drop(
                    arm=self.arm,
                    roarm_ip=self.roarm_ip
                )
            
            # Update state only if action succeeded
            if success:
                self.current_state = new_state
                return True, f"{fsm_message} | {result_message}"
            else:
                return False, f"Action failed: {result_message}"
        
        except Exception as e:
            return False, f"Error executing {action}: {e}"
    
    def get_current_state(self) -> str:
        """Get the current state of the robotic arm.
        
        Returns:
            Current state string
        """
        return self.current_state
    
    def set_state(self, state: str) -> bool:
        """Manually set the current state (use with caution).
        
        Args:
            state: New state - 'have_block' or 'doesnot_have_block'
        
        Returns:
            True if state was set successfully
        """
        normalized = state.strip().lower().replace(" ", "_")
        if normalized in {"have_block", "haveblock", "has_block"}:
            self.current_state = "have_block"
            return True
        elif normalized in {"doesnot_have_block", "does_not_have_block", "empty", "no_block"}:
            self.current_state = "doesnot_have_block"
            return True
        else:
            return False


def main():
    """Test the action controller with sample data."""
    # Sample test data
    test_targets = {
        "red": [(100.0, 50.0, -120.0)],
        "blue": [(150.0, -30.0, -115.0)],
        "green": [(200.0, 0.0, -120.0)]
    }
    
    # Initialize controller
    controller = ActionController()
    
    print(f"Initial state: {controller.get_current_state()}\n")
    
    # Test 1: Pick action
    print("Test 1: Pick red block")
    success, message = controller.execute_action("pick", test_targets, "green")
    print(f"Success: {success}")
    print(f"Message: {message}")
    print(f"New state: {controller.get_current_state()}\n")
    
    # Test 2: Try to pick again (should fail - no-op)
    print("Test 2: Try to pick again (should fail)")
    success, message = controller.execute_action("pick", test_targets, "blue")
    print(f"Success: {success}")
    print(f"Message: {message}")
    print(f"State: {controller.get_current_state()}\n")
    
    # Test 3: Place action
    print("Test 3: Place block")
    success, message = controller.execute_action("place", test_targets)
    print(f"Success: {success}")
    print(f"Message: {message}")
    print(f"New state: {controller.get_current_state()}\n")
    
    # Test 4: Drop when no block (should fail - no-op)
    print("Test 4: Try to drop when no block (should fail)")
    success, message = controller.execute_action("drop", test_targets)
    print(f"Success: {success}")
    print(f"Message: {message}")
    print(f"State: {controller.get_current_state()}\n")


if __name__ == "__main__":
    main()
