"""13-State Deterministic FDIR Executive State Machine.

Operational Modes:
    STARTUP, SELF_TEST, IDLE, PERCEPTION, LOCALIZATION, PLANNING,
    EXECUTION, REPLANNING, SAFE_STOP, RECOVERY, MISSION_COMPLETE,
    MISSION_ABORT, FAULT.
"""
from __future__ import annotations
from typing import Dict, Set, Tuple, List, Optional
from trl9_autonomy import OperatingState


class MissionStateMachine:
    """Deterministic finite state machine with transition guard rules."""

    # Explicit allowed state transition map
    TRANSITION_TABLE: Dict[OperatingState, Set[OperatingState]] = {
        OperatingState.STARTUP: {OperatingState.SELF_TEST, OperatingState.FAULT},
        OperatingState.SELF_TEST: {OperatingState.IDLE, OperatingState.FAULT},
        OperatingState.IDLE: {OperatingState.PERCEPTION, OperatingState.SAFE_STOP, OperatingState.FAULT},
        OperatingState.PERCEPTION: {OperatingState.LOCALIZATION, OperatingState.SAFE_STOP, OperatingState.FAULT},
        OperatingState.LOCALIZATION: {OperatingState.PLANNING, OperatingState.SAFE_STOP, OperatingState.FAULT},
        OperatingState.PLANNING: {OperatingState.EXECUTION, OperatingState.REPLANNING, OperatingState.SAFE_STOP, OperatingState.FAULT},
        OperatingState.EXECUTION: {
            OperatingState.PERCEPTION,
            OperatingState.REPLANNING,
            OperatingState.SAFE_STOP,
            OperatingState.MISSION_COMPLETE,
            OperatingState.FAULT,
        },
        OperatingState.REPLANNING: {OperatingState.PLANNING, OperatingState.RECOVERY, OperatingState.SAFE_STOP, OperatingState.FAULT},
        OperatingState.RECOVERY: {OperatingState.PERCEPTION, OperatingState.SAFE_STOP, OperatingState.FAULT},
        OperatingState.SAFE_STOP: {OperatingState.IDLE, OperatingState.MISSION_ABORT, OperatingState.FAULT},
        OperatingState.MISSION_COMPLETE: {OperatingState.IDLE},
        OperatingState.MISSION_ABORT: set(),  # Terminal state
        OperatingState.FAULT: {OperatingState.SAFE_STOP, OperatingState.MISSION_ABORT},
    }

    def __init__(self) -> None:
        self.current_state: OperatingState = OperatingState.STARTUP
        self.state_history: List[Tuple[float, OperatingState, str]] = []
        self.transition_count: int = 0

    def transition_to(self, next_state: OperatingState, timestamp: float, reason: str = "") -> bool:
        """Transitions state machine to next_state if transition is permitted."""
        allowed_next = self.TRANSITION_TABLE.get(self.current_state, set())

        # If transition is illegal, force fail-safe transition to FAULT
        if next_state not in allowed_next and next_state != OperatingState.FAULT:
            self.state_history.append((timestamp, OperatingState.FAULT, f"ILLEGAL_TRANSITION_ATTEMPT: {self.current_state} -> {next_state}"))
            self.current_state = OperatingState.FAULT
            self.transition_count += 1
            return False

        self.current_state = next_state
        self.state_history.append((timestamp, next_state, reason))
        self.transition_count += 1
        return True

    def is_safe_or_halted(self) -> bool:
        """Returns True if vehicle is in a non-actuating safe state."""
        return self.current_state in {
            OperatingState.IDLE,
            OperatingState.SAFE_STOP,
            OperatingState.MISSION_COMPLETE,
            OperatingState.MISSION_ABORT,
            OperatingState.FAULT,
        }
