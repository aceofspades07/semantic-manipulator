"""Action Controller for RoArm-M2 robotic arm.

This module provides a centralized controller for managing pick, place, and drop
actions on Jenga blocks. It integrates with the FSM controller to validate actions
based on the arm's current state (holding a block or not).

Usage:
    from roarm_m2.actions.control_action import ActionController
    from colour_coordinates import ColourCoordinates
    
    # Initialize controller
    controller = ActionController()
    
    # Get block coordinates
    detector = ColourCoordinates()
    coords = detector.capture()
    
    # Execute actions
    success, msg = controller.execute_action("pick", targets=coords, color="red")
    success, msg = controller.execute_action("place")
    success, msg = controller.execute_action("drop")
"""

import sys
import os
from typing import Dict, List, Iterable, Optional, Tuple

# Add parent directory to path for fsm_controller import
_parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)

from fsm_controller import fsm_controller
from roarm_m2.actions.pickup import pickup
from roarm_m2.actions.place import place
from roarm_m2.actions.drop import drop


class ActionController:
    """Manages robotic arm actions with FSM-based state validation.
    
    This controller ensures that actions are only executed when valid:
    - Pick: Only when not holding a block
    - Place: Only when holding a block
    - Drop: Only when holding a block or for full reset
    
    Attributes:
        current_state (str): Current state of the arm
            - "doesnot_have_block": Arm is not holding anything
            - "have_block": Arm is holding a block
        roarm_ip (str): IP address of the robotic arm
    """
    
    def __init__(self, roarm_ip: str = "192.168.4.1", initial_state: str = "doesnot_have_block"):
        """Initialize the ActionController.
        
        Args:
            roarm_ip (str): IP address of the RoArm-M2. Default: "192.168.4.1"
            initial_state (str): Initial state of the arm. Default: "doesnot_have_block"
        """
        self.current_state = initial_state
        self.roarm_ip = roarm_ip
    
    def execute_action(self, 
                      action: str, 
                      targets: Optional[Dict[str, List[Iterable[float]]]] = None,
                      color: Optional[str] = None,
                      **kwargs) -> Tuple[bool, str]:
        """Execute a robotic arm action with FSM validation.
        
        This method validates the requested action against the current state using
        the FSM controller, then executes it if valid.
        
        Args:
            action (str): Action to execute - "pick", "place", or "drop"
            targets (Dict[str, List[Iterable[float]]], optional): 
                Dictionary mapping colors to coordinate lists. Required for "pick" action.
                Format: {"red": [(x, y, z), ...], "blue": [(x, y, z), ...], ...}
            color (str, optional): Color of block to pick. Required for "pick" action.
            **kwargs: Additional arguments passed to the underlying action functions
        
        Returns:
            Tuple[bool, str]: (success, message)
                - success (bool): True if action executed successfully, False otherwise
                - message (str): Status message describing the result
        
        Examples:
            >>> controller = ActionController()
            >>> coords = {"red": [(200.0, 0.0, -120.0)]}
            >>> 
            >>> # Pick a red block
            >>> success, msg = controller.execute_action("pick", targets=coords, color="red")
            >>> print(msg)  # "Picked 'red' and returned to home (200.0, 0.0, 150.0)"
            >>> 
            >>> # Place the block
            >>> success, msg = controller.execute_action("place")
            >>> print(msg)  # "Object placed successfully"
            >>> 
            >>> # Drop/reset
            >>> success, msg = controller.execute_action("drop")
            >>> print(msg)  # "Gripper opened and arm returned to home"
        """
        action_lower = action.strip().lower()
        
        # Step 1: Validate action with FSM controller
        try:
            new_state, fsm_msg = fsm_controller(action_lower, self.current_state)
        except ValueError as e:
            return False, f"FSM validation error: {e}"
        
        # Step 2: Check if action was rejected (no-op)
        if "no-op" in fsm_msg:
            return False, f"Action rejected by FSM: {fsm_msg}"
        
        # Step 3: Execute the actual action based on type
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
        
        # Step 4: Update state only if execution succeeded
        if success:
            self.current_state = new_state
            return True, f"{result_msg} [FSM: {fsm_msg}]"
        else:
            # Action failed - state remains unchanged
            return False, f"Action execution failed: {result_msg}"
    
    def _perform_pick(self, 
                     targets: Optional[Dict[str, List[Iterable[float]]]], 
                     color: Optional[str],
                     **kwargs) -> Tuple[bool, str]:
        """Execute pickup action.
        
        Args:
            targets (Dict): Dictionary of color->coordinates mappings
            color (str): Color of block to pick
            **kwargs: Additional arguments for pickup function
        
        Returns:
            Tuple[bool, str]: Success status and message
        """
        if targets is None:
            return False, "No targets dictionary provided for pick action"
        
        if color is None:
            return False, "No color specified for pick action"
        
        # Call the pickup module
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
        """Execute place action.
        
        Args:
            **kwargs: Additional arguments for place function
        
        Returns:
            Tuple[bool, str]: Success status and message
        """
        try:
            success, msg = place(
                roarm_ip=self.roarm_ip,
                **kwargs
            )
            return success, msg
        except Exception as e:
            return False, f"Place execution error: {e}"
    
    def _perform_drop(self, **kwargs) -> Tuple[bool, str]:
        """Execute drop action.
        
        Args:
            **kwargs: Additional arguments for drop function
        
        Returns:
            Tuple[bool, str]: Success status and message
        """
        try:
            success, msg = drop(
                roarm_ip=self.roarm_ip,
                **kwargs
            )
            return success, msg
        except Exception as e:
            return False, f"Drop execution error: {e}"
    
    def get_current_state(self) -> str:
        """Get the current state of the robotic arm.
        
        Returns:
            str: Current state - "have_block" or "doesnot_have_block"
        """
        return self.current_state
    
    def reset_state(self, new_state: str = "doesnot_have_block") -> None:
        """Manually reset the controller's state.
        
        Use this if you need to synchronize the controller state with the
        physical arm state (e.g., after manual intervention or recovery).
        
        Args:
            new_state (str): New state to set. Default: "doesnot_have_block"
        """
        self.current_state = new_state
    
    def is_holding_block(self) -> bool:
        """Check if the arm is currently holding a block.
        
        Returns:
            bool: True if holding a block, False otherwise
        """
        return self.current_state == "have_block"


# Example usage and testing
if __name__ == "__main__":
    print("ActionController Test Suite")
    print("=" * 50)
    
    # Create controller instance
    controller = ActionController()
    print(f"Initial state: {controller.get_current_state()}")
    print(f"Holding block: {controller.is_holding_block()}")
    print()
    
    # Test 1: Try to place without picking (should fail)
    print("Test 1: Place without holding block")
    success, msg = controller.execute_action("place")
    print(f"Result: {msg}")
    print(f"Current state: {controller.get_current_state()}\n")
    
    # Test 2: Pick without proper arguments (should fail)
    print("Test 2: Pick without targets")
    success, msg = controller.execute_action("pick")
    print(f"Result: {msg}")
    print(f"Current state: {controller.get_current_state()}\n")
    
    # Test 3: Valid pick sequence (simulated - would need real coordinates)
    print("Test 3: Pick with mock coordinates")
    mock_coords = {"red": [(200.0, 0.0, -120.0)]}
    # This will fail at execution due to no real arm, but FSM validation will pass
    success, msg = controller.execute_action("pick", targets=mock_coords, color="red")
    print(f"Result: {msg}")
    print(f"Current state: {controller.get_current_state()}\n")
    
    print("=" * 50)
    print("Note: Full integration testing requires physical arm connection")
