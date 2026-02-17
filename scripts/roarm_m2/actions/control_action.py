"""Action controller for RoArm-M2 robotic arm with FSM validation."""

import sys
import os
from typing import Dict, List, Iterable, Optional, Tuple

# Add parent directory for imports
_parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)

from fsm_controller import fsm_controller
from roarm_m2.actions.pickup import pickup
from roarm_m2.actions.place import place
from roarm_m2.actions.drop import drop


class ActionController:
    """Manages robotic arm actions with FSM-based state validation."""
    
    def __init__(self, roarm_ip: str = "192.168.4.1", initial_state: str = "doesnot_have_block"):
        self.current_state = initial_state
        self.roarm_ip = roarm_ip
    
    def execute_action(self, 
                      action: str, 
                      targets: Optional[Dict[str, List[Iterable[float]]]] = None,
                      color: Optional[str] = None,
                      **kwargs) -> Tuple[bool, str]:
        """Execute a robotic arm action with FSM validation."""
        action_lower = action.strip().lower()
        
        # Validate action with FSM controller
        try:
            new_state, fsm_msg = fsm_controller(action_lower, self.current_state)
        except ValueError as e:
            return False, f"FSM validation error: {e}"
        
        # Check for rejected action
        if "no-op" in fsm_msg:
            return False, f"Action rejected by FSM: {fsm_msg}"
        
        # Execute the action
        success = False
        result_msg = ""
        
        if action_lower == "pick":
            success, result_msg = self._perform_pick(targets, color, **kwargs)
        elif action_lower == "place":
            success, result_msg = self._perform_place(**kwargs)
        elif action_lower == "drop":
            success, result_msg = self._perform_drop(**kwargs)
        else:
            return False, f"Unknown action: {action}"
        
        # Update state on success
        if success:
            self.current_state = new_state
            return True, f"{result_msg} [FSM: {fsm_msg}]"
        else:
            return False, f"Action execution failed: {result_msg}"
    
    def _perform_pick(self, 
                     targets: Optional[Dict[str, List[Iterable[float]]]], 
                     color: Optional[str],
                     **kwargs) -> Tuple[bool, str]:
        """Execute pickup action."""
        if targets is None:
            return False, "No targets dictionary provided for pick action"
        
        if color is None:
            return False, "No color specified for pick action"
        
        try:
            success, msg = pickup(
                targets=targets,
                color=color,
                roarm_ip=self.roarm_ip,
                **kwargs
            )
            return success, msg
        except Exception as e:
            return False, f"Pick execution error: {e}"
    
    def _perform_place(self, **kwargs) -> Tuple[bool, str]:
        """Execute place action."""
        try:
            success, msg = place(
                roarm_ip=self.roarm_ip,
                **kwargs
            )
            return success, msg
        except Exception as e:
            return False, f"Place execution error: {e}"
    
    def _perform_drop(self, **kwargs) -> Tuple[bool, str]:
        """Execute drop action."""
        try:
            success, msg = drop(
                roarm_ip=self.roarm_ip,
                **kwargs
            )
            return success, msg
        except Exception as e:
            return False, f"Drop execution error: {e}"
    
    def get_current_state(self) -> str:
        """Get the current state of the robotic arm."""
        return self.current_state
    
    def reset_state(self, new_state: str = "doesnot_have_block") -> None:
        """Manually reset the controller state."""
        self.current_state = new_state
    
    def is_holding_block(self) -> bool:
        """Check if the arm is currently holding a block."""
        return self.current_state == "have_block"


if __name__ == "__main__":
    print("ActionController Test Suite")
    
    # Create controller
    controller = ActionController()
    print(f"Initial state: {controller.get_current_state()}")
    print(f"Holding block: {controller.is_holding_block()}")
    print()
    
    # Test place without holding block
    print("Test 1: Place without holding block")
    success, msg = controller.execute_action("place")
    print(f"Result: {msg}")
    print(f"Current state: {controller.get_current_state()}\n")
    
    # Test pick without arguments
    print("Test 2: Pick without targets")
    success, msg = controller.execute_action("pick")
    print(f"Result: {msg}")
    print(f"Current state: {controller.get_current_state()}\n")
    
    # Test pick with mock coordinates
    print("Test 3: Pick with mock coordinates")
    mock_coords = {"red": [(200.0, 0.0, -120.0)]}
    success, msg = controller.execute_action("pick", targets=mock_coords, color="red")
    print(f"Result: {msg}")
    print(f"Current state: {controller.get_current_state()}\n")
    
    print("Note: Full integration testing requires physical arm connection")
