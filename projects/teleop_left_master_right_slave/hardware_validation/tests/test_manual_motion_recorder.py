"""Static + pure-logic tests for record_manual_motion.py (2026-09-10, H3/H4
recorder tooling task).

Requires the ROS 2 workspace to be sourced (record_manual_motion.py imports
touch_haptics.msg and bridge_master_slave.py at module scope) -- same
requirement as every other test in this project's scripts/ and
hardware_validation/ directories.
"""
from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

THIS_DIR = Path(__file__).resolve().parent
HV_DIR = THIS_DIR.parent
RECORDER_PATH = HV_DIR / "record_manual_motion.py"

sys.path.insert(0, str(HV_DIR))


# ---------------------------------------------------------------------------
# Section 4 (absolute requirement): static source assertion that the
# recorder never calls create_publisher(). This is a BUILD-TIME guard, not
# a runtime behavioral test -- it catches an accidental publisher addition
# even if no test happens to exercise that code path.
# ---------------------------------------------------------------------------

def test_recorder_source_contains_no_create_publisher_call():
    src = RECORDER_PATH.read_text()
    tree = ast.parse(src, filename=str(RECORDER_PATH))
    calls = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            name = None
            if isinstance(func, ast.Attribute):
                name = func.attr
            elif isinstance(func, ast.Name):
                name = func.id
            if name == "create_publisher":
                calls.append(node.lineno)
    assert not calls, (
        f"record_manual_motion.py must NEVER call create_publisher() -- found "
        f"call(s) at line(s) {calls}. The recorder is a measurement instrument "
        "only (section 4 of the H3/H4 recorder task); if a publisher is "
        "genuinely needed, it does not belong in this file."
    )


def test_recorder_source_never_imports_publisher_message_types_for_commands():
    """Secondary guard: the recorder should not import PoseStamped, WrenchStamped,
    or the Bool type used for /enable -- their presence would suggest a
    publisher is about to be built even if create_publisher() itself isn't
    called yet."""
    src = RECORDER_PATH.read_text()
    forbidden = ["PoseStamped", "WrenchStamped"]
    found = [name for name in forbidden if name in src]
    assert not found, (
        f"record_manual_motion.py references command message type(s) {found} -- "
        "this recorder must stay subscribe-only; these types are only used "
        "to PUBLISH target_pose/force_cmd elsewhere in this project."
    )


def test_recorder_module_imports_cleanly():
    """Import-only smoke test (requires the ROS env). Confirms
    record_manual_motion.py's module-level code (imports, constant
    definitions) has no syntax/import errors, without starting rclpy."""
    pytest.importorskip("rclpy")
    import importlib.util
    spec = importlib.util.spec_from_file_location("record_manual_motion", RECORDER_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert hasattr(mod, "RecorderNode")
    assert hasattr(mod, "LIVE_HARDWARE_NODE_NAMES")
    assert "teleop_master_left" in mod.LIVE_HARDWARE_NODE_NAMES
    assert "teleop_slave_right" in mod.LIVE_HARDWARE_NODE_NAMES


def test_recorder_node_creates_no_publishers_at_runtime():
    """Runtime confirmation (belt-and-suspenders alongside the static
    check above): after constructing RecorderNode, rclpy's own publisher
    count for that node must be exactly zero."""
    pytest.importorskip("rclpy")
    import rclpy
    import importlib.util
    spec = importlib.util.spec_from_file_location("record_manual_motion", RECORDER_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    class _Args:
        left_telemetry_topic = "/test_recorder_left_telem"
        right_telemetry_topic = "/test_recorder_right_telem"
        bridge_diagnostics_topic = "/test_recorder_bridge_diag"
        bilateral_status_topic = "/test_recorder_bilateral_status"
        left_dob_topic = "/test_recorder_left_dob"
        right_dob_topic = "/test_recorder_right_dob"
        auto_anchor = True

    if not rclpy.ok():
        rclpy.init()
    node = mod.RecorderNode(_Args())
    try:
        pubs = node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace())
        # rclpy gives every Node a /rosout logging publisher and a
        # /parameter_events publisher automatically -- neither is a command
        # publisher this recorder created, so they're excluded from the
        # "zero publishers" check rather than the check being dropped.
        rclpy_infra_topics = {"/rosout", "/parameter_events"}
        app_pubs = [p for p in pubs if p[0] not in rclpy_infra_topics]
        assert app_pubs == [], f"RecorderNode must create zero application publishers, found: {app_pubs}"
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Section 17: live-hardware refusal logic, tested against a fake node
# (no real ROS graph needed) -- confirms the exact node-name match this
# check relies on, independent of whatever is/isn't actually running on
# this machine's own ROS domain right now.
# ---------------------------------------------------------------------------

class _FakeSpinlessNode:
    """Enough of rclpy.node.Node's interface for
    _refuse_if_live_hardware_attached() -- spin_once/get_node_names_and_namespaces
    only, no real rclpy context needed."""

    def __init__(self, node_names):
        self._node_names = node_names

    def get_node_names_and_namespaces(self):
        return [(n, "/") for n in self._node_names]


def test_refuses_when_teleop_master_left_present():
    pytest.importorskip("rclpy")
    import importlib.util
    spec = importlib.util.spec_from_file_location("record_manual_motion", RECORDER_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    orig_spin_once = mod.rclpy.spin_once
    mod.rclpy.spin_once = lambda node, timeout_sec=0.0: None
    try:
        fake = _FakeSpinlessNode(["some_sim_node", "teleop_master_left"])
        reason = mod._refuse_if_live_hardware_attached(fake)
        assert reason is not None
        assert "teleop_master_left" in reason
    finally:
        mod.rclpy.spin_once = orig_spin_once


def test_refuses_when_teleop_slave_right_present():
    pytest.importorskip("rclpy")
    import importlib.util
    spec = importlib.util.spec_from_file_location("record_manual_motion", RECORDER_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    orig_spin_once = mod.rclpy.spin_once
    mod.rclpy.spin_once = lambda node, timeout_sec=0.0: None
    try:
        fake = _FakeSpinlessNode(["teleop_slave_right"])
        reason = mod._refuse_if_live_hardware_attached(fake)
        assert reason is not None
        assert "teleop_slave_right" in reason
    finally:
        mod.rclpy.spin_once = orig_spin_once


def test_does_not_refuse_on_an_unrelated_sim_graph():
    pytest.importorskip("rclpy")
    import importlib.util
    spec = importlib.util.spec_from_file_location("record_manual_motion", RECORDER_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    orig_spin_once = mod.rclpy.spin_once
    mod.rclpy.spin_once = lambda node, timeout_sec=0.0: None
    try:
        fake = _FakeSpinlessNode(["ci_m_1234abcd", "ci_s_5678efgh", "touch_master_slave_bridge"])
        reason = mod._refuse_if_live_hardware_attached(fake)
        assert reason is None
    finally:
        mod.rclpy.spin_once = orig_spin_once


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
