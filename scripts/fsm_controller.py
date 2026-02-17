"""Simple FSM controller for pick, place, and drop actions."""

from typing import Tuple

_VALID_ACTIONS = {"pick", "drop", "place"}


def _normalize_state(s: str) -> str:
    """Normalize state string to canonical form."""
	if not isinstance(s, str):
		raise ValueError("current_state must be a string")
	t = s.strip().lower().replace(" ", "_")
	# Accept common forms and normalize to canonical states
	if t in {"have_block", "haveblock", "has_block", "hasblock"}:
		return "have_block"
	if t in {"doesnot_have_block", "doesnot_haveblock", "does_not_have_block", "does_not_have_block", "doesnot_have_block", "doesnot_have_block", "doesnot_have_block"}:
		return "doesnot_have_block"
	if t in {"doesnot_have_block", "doesnot_have_block", "does_not_have_block", "does not have block", "doesnot have block"}:
		return "doesnot_have_block"
	if t in {"empty", "no_block", "no-block", "no_block_present"}:
		return "doesnot_have_block"
	raise ValueError(f"unknown current_state: {s!r}")


def pick() -> str:
    """Stub pick action. Replace with real robot implementation."""
	return "picked"


def drop() -> str:
    """Stub drop action. Replace with real robot implementation."""
	return "dropped"


def place() -> str:
    """Stub place action. Replace with real robot implementation."""
	return "placed"


def fsm_controller(action_name: str, current_state: str) -> Tuple[str, str]:
    """Execute pick, place, or drop depending on current state."""
	if not isinstance(action_name, str):
		raise ValueError("action_name must be a string")
	action = action_name.strip().lower()
	if action not in _VALID_ACTIONS:
		raise ValueError(f"unknown action: {action_name!r}")

	state = _normalize_state(current_state)

	if action == "pick":
		if state == "doesnot_have_block":
			result = pick()
			return "have_block", f"pick: {result}"
		else:
			return state, "no-op: already have block"

	if action == "drop":
		if state == "have_block":
			result = drop()
			return "doesnot_have_block", f"drop: {result}"
		else:
			return state, "no-op: no block to drop"

	if action == "place":
		if state == "have_block":
			result = place()
			return "doesnot_have_block", f"place: {result}"
		else:
			return state, "no-op: no block to place"


if __name__ == "__main__":
	# Test demo
	print(fsm_controller("pick", "does not have block"))
	print(fsm_controller("pick", "have_block"))
	print(fsm_controller("drop", "have_block"))
	print(fsm_controller("drop", "does not have block"))
	print(fsm_controller("place", "have_block"))
	print(fsm_controller("place", "does not have block"))

