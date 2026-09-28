"""2026-09-10 real-hardware finding regression test (Task 4/5 item 6): a
PRE_ARM_FAIL run must still produce a complete part1_meta.json, and
analyze_h2_gravity_settle.py must report a structured FAIL/PRE_ARM_FAILED
result rather than crashing with FileNotFoundError.

Real-hardware evidence: the second 2026-09-10 H2 attempt correctly failed
pre-arm (bridge oscillating READY<->GRAVITY_RAMP_DOWN) but then
analyze_h2_gravity_settle.py crashed because part1_meta.json did not
exist at all.

Pure Python -- _write_pre_arm_fail_meta() only writes JSON/CSV files and
shells out to `git` (read-only); no ROS/rclpy needed. Requires the ROS 2
workspace sourced only because part1_gravity_settle_capture.py imports
touch_haptics.msg at module scope, same as every other test in this
directory.
"""
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parent))
from part1_gravity_settle_capture import _write_pre_arm_fail_meta  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from analyze_h2_gravity_settle import analyze  # noqa: E402


def _args(session_id="test_session"):
    return SimpleNamespace(session_id=session_id, observation_window_s=10.0)


def test_pre_arm_fail_writes_complete_meta_json():
    with tempfile.TemporaryDirectory() as td:
        out_dir = Path(td)
        pre_arm = {"status": "PRE_ARM_FAIL", "reasons": ["synthetic test reason"]}
        ownership = {"ok": True}
        _write_pre_arm_fail_meta(out_dir, _args(), Path("/home/nural/HIL-RL"), ["fake", "bridge", "command"],
                                 pre_arm, ownership, "READY")
        meta_path = out_dir / "part1_meta.json"
        assert meta_path.exists()
        meta = json.loads(meta_path.read_text())
        assert meta["aborted"] is True
        assert meta["abort_reason"].startswith("PRE_ARM_FAILED")
        assert meta["arm_sent"] is False
        assert meta["gate_pass_t"] is None
        assert meta["pre_arm_evaluation"] == pre_arm
        # Empty-but-present CSVs too.
        for fname in ("part1_gravity_settle_master.csv", "part1_gravity_settle_slave.csv", "part1_bridge_diag.csv"):
            assert (out_dir / fname).exists()


def test_pre_arm_fail_artifact_analyzes_as_structured_fail_not_crash():
    with tempfile.TemporaryDirectory() as td:
        out_dir = Path(td)
        pre_arm = {"status": "PRE_ARM_FAIL", "reasons": ["bridge_state='GRAVITY_RAMP_DOWN' is not a valid "
                                                          "pre-arm state"]}
        ownership = {"ok": True}
        _write_pre_arm_fail_meta(out_dir, _args(), Path("/home/nural/HIL-RL"), ["fake", "bridge", "command"],
                                 pre_arm, ownership, "GRAVITY_RAMP_DOWN")
        result = analyze(out_dir)  # must not raise
        assert result["validation"]["overall_result"] == "PRE_ARM_FAILED"
        assert (out_dir / "h2_validation.json").exists()


def test_missing_meta_json_entirely_analyzes_gracefully():
    """Defense-in-depth guard in analyze() itself (belt-and-suspenders
    alongside the part1-side fix above) -- if part1_meta.json is somehow
    STILL missing, the analyzer must report ANALYSIS_INCOMPLETE, not
    raise FileNotFoundError."""
    with tempfile.TemporaryDirectory() as td:
        out_dir = Path(td)
        result = analyze(out_dir)  # must not raise
        assert result["validation"]["overall_result"] == "ANALYSIS_INCOMPLETE"


def test_malformed_meta_json_analyzes_gracefully_via_main():
    """main()'s top-level try/except must convert an unexpected exception
    (e.g. a truncated/corrupt file) into a structured result rather than a
    raw traceback."""
    import subprocess
    with tempfile.TemporaryDirectory() as td:
        out_dir = Path(td)
        (out_dir / "part1_meta.json").write_text("{not valid json")
        analyzer_path = Path(__file__).parent / "analyze_h2_gravity_settle.py"
        r = subprocess.run([sys.executable, str(analyzer_path), str(out_dir)],
                           capture_output=True, text=True, timeout=15)
        assert r.returncode != 0
        assert "Traceback" not in r.stderr, f"raw traceback leaked instead of a structured result:\n{r.stderr}"
        validation_path = out_dir / "h2_validation.json"
        assert validation_path.exists()
        validation = json.loads(validation_path.read_text())
        assert validation["overall_result"] == "ANALYSIS_INCOMPLETE"


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
