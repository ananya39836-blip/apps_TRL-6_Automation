"""Real-Time Navigation Cycle Execution Watchdog.

Monitors navigation cycle execution latencies to prevent timing overruns
exceeding the strict 10 Hz / 100 ms RTOS deadline budget.
"""
from __future__ import annotations
import time
from typing import List, Tuple


class CycleExecutionWatchdog:
    """Monitors cycle execution latency and escalates on repeated deadline misses."""

    def __init__(self, max_cycle_latency_ms: float = 100.0, max_consecutive_misses: int = 3) -> None:
        self.max_latency_ms = max_cycle_latency_ms
        self.max_misses = max_consecutive_misses

        self.consecutive_misses: int = 0
        self.total_cycles: int = 0
        self.total_misses: int = 0
        self.cycle_latencies_ms: List[float] = []

    def evaluate_cycle(self, cycle_latency_ms: float) -> Tuple[bool, bool]:
        """Evaluates cycle execution time.

        Returns:
            Tuple of (is_deadline_met: bool, trigger_safe_stop: bool).
        """
        self.total_cycles += 1
        self.cycle_latencies_ms.append(cycle_latency_ms)

        if cycle_latency_ms <= self.max_latency_ms:
            self.consecutive_misses = 0
            return True, False
        else:
            self.total_misses += 1
            self.consecutive_misses += 1
            trigger_stop = self.consecutive_misses >= self.max_misses
            return False, trigger_stop

    def get_latency_percentiles(self) -> dict:
        """Returns mean, 95th, and 99th percentile execution latencies."""
        if not self.cycle_latencies_ms:
            return {"mean_ms": 0.0, "p95_ms": 0.0, "p99_ms": 0.0, "max_ms": 0.0}

        import numpy as np
        arr = np.array(self.cycle_latencies_ms)
        return {
            "mean_ms": float(np.mean(arr)),
            "p95_ms": float(np.percentile(arr, 95)),
            "p99_ms": float(np.percentile(arr, 99)),
            "max_ms": float(np.max(arr)),
            "miss_rate_pct": float((self.total_misses / max(1, self.total_cycles)) * 100.0),
        }
