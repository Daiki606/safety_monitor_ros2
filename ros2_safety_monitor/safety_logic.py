#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

import math


WARNING_RADIUS = 6.0
DANGER_RADIUS = 10.0


def calculate_distance(x: float, y: float) -> float:
    """Compute Euclidean distance from (0,0)."""
    return math.sqrt(x * x + y * y)


def get_safety_status(x: float, y: float) -> str:
    """
    Determine safety status based on distance from origin.
    Returns: "safe", "warning", or "danger".
    """
    distance = calculate_distance(x, y)

    if distance < WARNING_RADIUS:
        return "safe"
    elif WARNING_RADIUS <= distance < DANGER_RADIUS:
        return "warning"
    else:
        return "danger"
