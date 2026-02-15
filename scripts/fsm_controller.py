"""Simple FSM controller for pick/drop actions.

Function: `fsm_controller(action_name, current_state)`
- `action_name` should be 'pick' or 'drop' (case-insensitive)
- `current_state` can be provided in several common forms; it's normalized.

Returns a tuple: `(new_state, message)` where `new_state` is one of
  'have_block' or 'doesnot_have_block'
and `message` is a short status string.
"""

from typing import Tuple

_VALID_ACTIONS = {"pick", "drop", "place"}


def _normalize_state(s: str) -> str:
	if not isinstance(s, str):
		raise ValueError("current_state must be a string")
	t = s.strip().lower().replace(" ", "_")
	# Accept several common forms and normalize to two canonical states
	if t in {"have_block", "haveblock", "has_block", "hasblock"}:
		return "have_block"
	if t in {"doesnot_have_block", "doesnot_haveblock", "does_not_have_block", "does_not_have_block", "doesnot_have_block", "doesnot_have_block", "doesnot_have_block"}:
		return "doesnot_have_block"
	# common English variant
	if t in {"doesnot_have_block", "doesnot_have_block", "does_not_have_block", "does not have block", "doesnot have block"}:
		return "doesnot_have_block"
	# fallbacks
	if t in {"empty", "no_block", "no-block", "no_block_present"}:
		return "doesnot_have_block"
	raise ValueError(f"unknown current_state: {s!r}")


def pick() -> str:
	"""Stub pick action.

	Replace or monkey-patch this with the real robot pick implementation.
	"""
	# perform pick action here
	return "picked"


def drop() -> str:
	"""Stub drop action.

	Replace or monkey-patch this with the real robot drop implementation.
	"""
	# perform drop action here
	return "dropped"


def place() -> str:
	"""Stub place action.

	Replace or monkey-patch this with the real robot place implementation.
	"""
	# perform place action here
	return "placed"


def fsm_controller(action_name: str, current_state: str) -> Tuple[str, str]:
	"""Execute `pick` or `drop` depending on `current_state`.

	Behavior:
	- If action is 'pick' and current state indicates no block -> call `pick()`
	  and transition to 'have_block'.
	- If action is 'drop' and current state indicates have block -> call `drop()`
	  and transition to 'doesnot_have_block'.
	- If action would be a no-op (e.g., pick when already have block), no action
	  is executed and state is returned unchanged with an explanatory message.

	Returns:
	  (new_state, message)
	"""
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
	# tiny manual test/demo
	print(fsm_controller("pick", "does not have block"))
	print(fsm_controller("pick", "have_block"))
	print(fsm_controller("drop", "have_block"))
	print(fsm_controller("drop", "does not have block"))
	print(fsm_controller("place", "have_block"))
	print(fsm_controller("place", "does not have block"))

