#!/usr/bin/env python3
"""Test script for ActionController and ColourCoordinates integration.

This script tests the robustness of the pick, place, and drop actions
on real Jenga blocks detected by the RealSense camera.

The script:
1. Detects blocks using ColourCoordinates
2. Executes various action sequences to test FSM logic
3. Re-detects blocks after each action to maintain accurate state
"""

import sys
import os
import time
import random

# Add parent directories to path for imports
_project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from colour_coordinates import ColourCoordinates
from roarm_m2.actions.control_action import ActionController


# Configurable delay range (seconds) between actions for block re-detection
MIN_DELAY = 4.0
MAX_DELAY = 5.0


def get_random_delay() -> float:
    """Generate a random delay between MIN_DELAY and MAX_DELAY."""
    return random.uniform(MIN_DELAY, MAX_DELAY)


def print_separator(title: str = "") -> None:
    """Print a visual separator for logging."""
    print("\n" + "=" * 60)
    if title:
        print(f"  {title}")
        print("=" * 60)


def print_detected_blocks(coords: dict) -> None:
    """Print detected blocks in a readable format."""
    if not coords:
        print("  [No blocks detected]")
        return
    
    total = sum(len(positions) for positions in coords.values())
    print(f"  Total blocks detected: {total}")
    for color, positions in coords.items():
        print(f"    {color}: {len(positions)} block(s)")
        for idx, (x, y, z) in enumerate(positions):
            print(f"      [{idx}] ({x:.1f}, {y:.1f}, {z:.1f})")


def execute_and_log(controller: ActionController, action: str, 
                    targets: dict = None, color: str = None, detector = ColourCoordinates()) -> bool:
    """Execute an action and log the result.
    
    Returns:
        bool: True if action succeeded, False otherwise
    """
    print(f"\n>>> Executing: {action.upper()}", end="")
    if color:
        print(f" (color={color})", end="")
    print()

    success, msg = controller.execute_action(action, targets=targets, color=color)
    
    status = "SUCCESS" if success else "FAILED"
    print(f"    [{status}] {msg}")
    print(f"    Current state: {controller.get_current_state()}")
    
    return success


def refresh_blocks(detector: ColourCoordinates, delay: float = None) -> dict:
    """Wait, then refresh block detection.
    
    Args:
        detector: ColourCoordinates instance
        delay: Delay in seconds before refreshing (random if None)
    
    Returns:
        Updated coordinates dictionary
    """
    wait_time = delay if delay is not None else get_random_delay()
    print(f"\n... Waiting {wait_time:.1f}s before refreshing block detection ...")
    time.sleep(wait_time)
    
    print("... Refreshing block detection ...")
    coords = detector.refresh()
    print_detected_blocks(coords)
    
    return coords


def main():
    """Main test execution."""
    print_separator("ROBOTIC ARM ACTION CONTROLLER TEST")
    print(f"  Project: Jenga Block Pick-Place-Drop Testing")
    print(f"  Delay range: {MIN_DELAY:.1f}s - {MAX_DELAY:.1f}s")
    
    # Initialize components
    print("\n[Init] Initializing ColourCoordinates detector...")
    detector = ColourCoordinates()
    
    print("[Init] Initializing ActionController...")
    controller = ActionController()
    
    # Initial block detection
    print_separator("INITIAL BLOCK DETECTION")
    print("Capturing initial block positions...")
    coords = detector.capture(move_to_home=True)
    print_detected_blocks(coords)
    
    if not coords:
        print("\n[ERROR] No blocks detected! Please place blocks on the table and retry.")
        return
    
    available_colors = detector.get_available_colors()
    print(f"\nAvailable colors: {available_colors}")
    
    # === TEST SUITE: Individual Actions ===
    
    # Action 1: Try place without holding anything (should fail - FSM rejection)
    print_separator("ACTION 1: PLACE without holding block")
    execute_and_log(controller, "place")
    # coords = refresh_blocks(detector)
    
    # Action 2: Try drop without holding anything (should fail - FSM rejection)
    print_separator("ACTION 2: DROP without holding block")
    execute_and_log(controller, "drop")
    coords = refresh_blocks(detector)
    
    # Action 3: Pick first available color
    available_colors = list(coords.keys())
    if available_colors:
        print_separator(f"ACTION 3: PICK {available_colors[0]}")
        execute_and_log(controller, "pick", targets=coords, color=available_colors[0], detector = detector)
        coords = refresh_blocks(detector)
    
    # Action 4: Try to pick again while holding (should fail - FSM rejection)
    available_colors = list(coords.keys())
    if available_colors:
        print_separator(f"ACTION 4: PICK {available_colors[0]} while holding")
        execute_and_log(controller, "pick", targets=coords, color=available_colors[0], detector = detector)
        # coords = refresh_blocks(detector)
    
    # Action 5: Place the held block (should succeed if holding)
    print_separator("ACTION 5: PLACE held block")
    execute_and_log(controller, "place")
    coords = refresh_blocks(detector)
    
    # Action 6: Pick another color if available
    available_colors = list(coords.keys())
    if len(available_colors) > 1:
        print_separator(f"ACTION 6: PICK {available_colors[1]}")
        execute_and_log(controller, "pick", targets=coords, color=available_colors[1], detector = detector)
        # coords = refresh_blocks(detector)
    elif available_colors:
        print_separator(f"ACTION 6: PICK {available_colors[0]}")
        execute_and_log(controller, "pick", targets=coords, color=available_colors[0], detector = detector)
        # coords = refresh_blocks(detector)
    
    # Action 7: Drop the held block (should succeed if holding)
    print_separator("ACTION 7: DROP held block")
    execute_and_log(controller, "drop")
    # coords = refresh_blocks(detector)
    
    # Action 8: Pick third color or first available
    available_colors = list(coords.keys())
    if len(available_colors) > 2:
        print_separator(f"ACTION 8: PICK {available_colors[2]}")
        execute_and_log(controller, "pick", targets=coords, color=available_colors[2], detector = detector)
        # coords = refresh_blocks(detector)
    elif available_colors:
        print_separator(f"ACTION 8: PICK {available_colors[0]}")
        execute_and_log(controller, "pick", targets=coords, color=available_colors[0], detector = detector)
        # coords = refresh_blocks(detector)
    
    # Action 9: Try place while holding (should succeed)
    print_separator("ACTION 9: PLACE held block")
    execute_and_log(controller, "place")
    # coords =  refresh_blocks(detector)
    
    # Action 10: Try drop after place (should fail - no longer holding)
    print_separator("ACTION 10: DROP after PLACE (not holding)")
    execute_and_log(controller, "drop")
    coords =  refresh_blocks(detector)
    
    # Action 11: Pick first available color again
    available_colors = list(coords.keys())
    if available_colors:
        print_separator(f"ACTION 11: PICK {available_colors[0]}")
        execute_and_log(controller, "pick", targets=coords, color=available_colors[0], detector = detector)
        # coords =  refresh_blocks(detector)
    
    # Action 12: Drop to finish
    print_separator("ACTION 12: DROP to finish")
    execute_and_log(controller, "drop")
    # coords =  refresh_blocks(detector)
    
    # Action 13: Pick another block if still available
    available_colors = list(coords.keys())
    if available_colors:
        pick_color = available_colors[1] if len(available_colors) > 1 else available_colors[0]
        print_separator(f"ACTION 13: PICK {pick_color}")
        execute_and_log(controller, "pick", targets=coords, color=pick_color, detector = detector)
        # coords =  refresh_blocks(detector)
    
    # Action 14: Place
    print_separator("ACTION 14: PLACE held block")
    execute_and_log(controller, "place")
    # coords =  refresh_blocks(detector)
    
    # === FINAL SUMMARY ===
    print_separator("TEST COMPLETE")
    print(f"Final controller state: {controller.get_current_state()}")
    print(f"Holding block: {controller.is_holding_block()}")
    
    # Final detection
    print("\nFinal block detection:")
    # coords =  detector.refresh()
    print_detected_blocks(coords)
    
    print("\n" + "=" * 60)
    print("  All tests completed!")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
