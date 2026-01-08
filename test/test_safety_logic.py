# SPDX-License-Identifier: MIT

from ros2_safety_monitor.safety_logic import get_safety_status


def test_safe():
    assert get_safety_status(2.0, 3.0) == "safe"


def test_warning():
    assert get_safety_status(6.0, 0.0) == "warning"
    assert get_safety_status(7.0, 1.0) == "warning"


def test_danger():
    assert get_safety_status(10.0, 0.0) == "danger"
    assert get_safety_status(8.0, 8.0) == "danger"


def test_boundary_safe_warning():
    assert get_safety_status(5.99, 0.0) == "safe"
    assert get_safety_status(6.01, 0.0) == "warning"


def test_boundary_warning_danger():
    assert get_safety_status(9.99, 0.0) == "warning"
    assert get_safety_status(10.01, 0.0) == "danger"
