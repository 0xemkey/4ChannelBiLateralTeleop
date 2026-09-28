"""Adversarial tests for the hardened qualification gate (2026-09-10,
full4ch pre-tuning closure Task 2). Pure file-based, no ROS/rclpy
dependency -- write_qualification_artifact.py only reads CSV/JSON.

Covers the task's explicit adversarial list: empty artifact, malformed
artifact, wrong schema, wrong channel, zero force evidence, wrong sign,
missing bilateral evidence, and (where provenance permits) an
insufficient-excitation-style rejection.
"""
from __future__ import annotations

import csv
import json
import subprocess
import sys
from pathlib import Path

import pytest

HV_DIR = Path(__file__).resolve().parents[1]
WRITER = HV_DIR / "write_qualification_artifact.py"

sys.path.insert(0, str(HV_DIR))
import write_qualification_artifact as wqa  # noqa: E402

JOINTS = ("waist", "shoulder", "elbow")


def _write_bilateral_csv(path: Path, n: int, source_vals, command_vals, source_key="tau_e_hat_nm",
                         command_key="master_torque_cmd_nm"):
    fields = ["t"] + [f"{source_key}_{j}" for j in JOINTS] + [f"{command_key}_{j}" for j in JOINTS]
    other_source = "tau_h_hat_nm" if source_key == "tau_e_hat_nm" else "tau_e_hat_nm"
    other_command = "slave_torque_cmd_nm" if command_key == "master_torque_cmd_nm" else "master_torque_cmd_nm"
    fields += [f"{other_source}_{j}" for j in JOINTS] + [f"{other_command}_{j}" for j in JOINTS]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for i in range(n):
            row = {"t": i}
            for j in JOINTS:
                row[f"{source_key}_{j}"] = source_vals(i, j)
                row[f"{command_key}_{j}"] = command_vals(i, j)
                row[f"{other_source}_{j}"] = 0.0
                row[f"{other_command}_{j}"] = 0.0
            w.writerow(row)


def _write_receiving_csv(path: Path, n: int, relay_force, relay_valid=True, relay_fresh=True,
                         relay_applied=True):
    fields = ["t", "relay_force_n_x", "relay_force_n_y", "relay_force_n_z",
              "relay_valid", "relay_fresh", "relay_applied",
              "applied_force_n_x", "applied_force_n_y", "applied_force_n_z"]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for i in range(n):
            fy = relay_force(i)
            w.writerow({"t": i, "relay_force_n_x": 0.0, "relay_force_n_y": fy, "relay_force_n_z": 0.0,
                       "relay_valid": str(relay_valid), "relay_fresh": str(relay_fresh),
                       "relay_applied": str(relay_applied),
                       "applied_force_n_x": 0.0, "applied_force_n_y": fy, "applied_force_n_z": 0.0})


def _real_c1_like_trial(tmp_path: Path) -> Path:
    """A trial shaped like the genuine C1 evidence this project already has:
    real excitation, correctly-signed (negative) correlation, real relay force."""
    trial = tmp_path / "trial"
    trial.mkdir()
    n = 50
    _write_bilateral_csv(
        trial / "raw_bilateral.csv", n,
        source_vals=lambda i, j: 0.05 * ((-1) ** (i // 5)) + (0.01 if j == "shoulder" else 0.0),
        command_vals=lambda i, j: -0.03 * ((-1) ** (i // 5)),
    )
    _write_receiving_csv(trial / "raw_master.csv", n, relay_force=lambda i: 0.3 * ((-1) ** (i // 5)))
    (trial / "manifest.json").write_text(json.dumps({
        "expected_channel_configuration": {"c1": True, "c2": False, "c3": False, "c4": False},
        "git_head": "deadbeef", "git_dirty": False, "git_branch": "test", "config_fingerprint": "abc123",
    }))
    (trial / "validation.json").write_text(json.dumps({
        "overall_result": "VALID",
        "channel_isolation": {"live_channel_active_available": True, "missing_expected_activation": None},
    }))
    (trial / "metrics.json").write_text(json.dumps({"analysis_direction": "c1"}))
    return trial


def test_real_looking_c1_trial_passes():
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        trial = _real_c1_like_trial(Path(d))
        ev = wqa.compute_force_evidence(trial, "c1", disabled_joints=set())
        assert ev["passed"], ev["problems"]


def test_zero_force_evidence_fails():
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        tmp_path = Path(d)
        trial = tmp_path / "trial"
        trial.mkdir()
        n = 50
        _write_bilateral_csv(trial / "raw_bilateral.csv", n,
                             source_vals=lambda i, j: 0.0, command_vals=lambda i, j: 0.0)
        _write_receiving_csv(trial / "raw_master.csv", n, relay_force=lambda i: 0.0,
                             relay_applied=False)
        ev = wqa.compute_force_evidence(trial, "c1", disabled_joints=set())
        assert not ev["passed"]
        assert any("source disturbance" in p or "commanded relay torque" in p for p in ev["problems"])


def test_wrong_sign_fails():
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        tmp_path = Path(d)
        trial = tmp_path / "trial"
        trial.mkdir()
        n = 50
        # C1 expects NEGATIVE correlation; make it POSITIVE instead.
        _write_bilateral_csv(
            trial / "raw_bilateral.csv", n,
            source_vals=lambda i, j: 0.05 * ((-1) ** (i // 5)),
            command_vals=lambda i, j: 0.03 * ((-1) ** (i // 5)),  # same sign as source -> wrong for C1
        )
        _write_receiving_csv(trial / "raw_master.csv", n, relay_force=lambda i: 0.3 * ((-1) ** (i // 5)))
        ev = wqa.compute_force_evidence(trial, "c1", disabled_joints=set())
        assert not ev["passed"]
        assert any("does not meet expected" in p for p in ev["problems"])


def test_stale_relay_fails():
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        tmp_path = Path(d)
        trial = tmp_path / "trial"
        trial.mkdir()
        n = 50
        _write_bilateral_csv(
            trial / "raw_bilateral.csv", n,
            source_vals=lambda i, j: 0.05 * ((-1) ** (i // 5)),
            command_vals=lambda i, j: -0.03 * ((-1) ** (i // 5)),
        )
        _write_receiving_csv(trial / "raw_master.csv", n, relay_force=lambda i: 0.3 * ((-1) ** (i // 5)),
                             relay_fresh=False)
        ev = wqa.compute_force_evidence(trial, "c1", disabled_joints=set())
        assert not ev["passed"]
        assert any("relay_fresh_duty" in p for p in ev["problems"])


def test_relay_never_applied_fails():
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        tmp_path = Path(d)
        trial = tmp_path / "trial"
        trial.mkdir()
        n = 50
        _write_bilateral_csv(
            trial / "raw_bilateral.csv", n,
            source_vals=lambda i, j: 0.05 * ((-1) ** (i // 5)),
            command_vals=lambda i, j: -0.03 * ((-1) ** (i // 5)),
        )
        _write_receiving_csv(trial / "raw_master.csv", n, relay_force=lambda i: 0.3 * ((-1) ** (i // 5)),
                             relay_applied=False)
        ev = wqa.compute_force_evidence(trial, "c1", disabled_joints=set())
        assert not ev["passed"]
        assert any("relay_applied_duty" in p for p in ev["problems"])


def test_masked_joint_excluded_from_sign_requirement():
    """A masked joint (e.g. shoulder) must not be required to show a
    correlation -- it has no relay contribution by construction."""
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        tmp_path = Path(d)
        trial = tmp_path / "trial"
        trial.mkdir()
        n = 50

        def cmd(i, j):
            if j == "shoulder":
                return 0.0  # masked: no command at all
            return -0.03 * ((-1) ** (i // 5))

        _write_bilateral_csv(trial / "raw_bilateral.csv", n,
                             source_vals=lambda i, j: 0.05 * ((-1) ** (i // 5)), command_vals=cmd)
        _write_receiving_csv(trial / "raw_master.csv", n, relay_force=lambda i: 0.3 * ((-1) ** (i // 5)))
        ev = wqa.compute_force_evidence(trial, "c1", disabled_joints={"shoulder"})
        assert ev["passed"], ev["problems"]
        assert ev["per_joint"]["shoulder"]["masked"] is True
        assert "shoulder" not in ev["required_joints"]


def _run_main(trial_dir: Path, channel: str, session: str, out_dir: Path, disabled_joints: str = ""):
    return subprocess.run(
        [sys.executable, str(WRITER), "--trial-dir", str(trial_dir), "--channel", channel,
         "--session", session, "--out-dir", str(out_dir), "--disabled-joints", disabled_joints],
        capture_output=True, text=True,
    )


def test_missing_manifest_fails(tmp_path):
    trial = tmp_path / "trial"
    trial.mkdir()
    r = _run_main(trial, "c1", "sess", tmp_path / "out")
    assert r.returncode != 0
    assert not (tmp_path / "out" / "C1_QUALIFIED.json").exists()


def test_malformed_manifest_fails(tmp_path):
    trial = tmp_path / "trial"
    trial.mkdir()
    (trial / "manifest.json").write_text("{not valid json")
    r = _run_main(trial, "c1", "sess", tmp_path / "out")
    assert r.returncode != 0


def test_wrong_channel_expected_fails(tmp_path):
    trial = _real_c1_like_trial(tmp_path)
    (trial / "manifest.json").write_text(json.dumps({
        "expected_channel_configuration": {"c1": False, "c2": False, "c3": False, "c4": False},
    }))
    r = _run_main(trial, "c1", "sess", tmp_path / "out")
    assert r.returncode != 0
    assert not (tmp_path / "out" / "C1_QUALIFIED.json").exists()


def test_missing_bilateral_status_evidence_fails(tmp_path):
    trial = _real_c1_like_trial(tmp_path)
    (trial / "validation.json").write_text(json.dumps({
        "overall_result": "VALID",
        "channel_isolation": {"live_channel_active_available": False, "missing_expected_activation": None},
    }))
    r = _run_main(trial, "c1", "sess", tmp_path / "out")
    assert r.returncode != 0
    assert "bilateral_status" in (r.stderr + r.stdout).lower() or "channel-active" in (r.stderr + r.stdout)


def test_missing_expected_activation_fails(tmp_path):
    trial = _real_c1_like_trial(tmp_path)
    (trial / "validation.json").write_text(json.dumps({
        "overall_result": "VALID",
        "channel_isolation": {"live_channel_active_available": True,
                              "missing_expected_activation": [{"channel": "c1"}]},
    }))
    r = _run_main(trial, "c1", "sess", tmp_path / "out")
    assert r.returncode != 0


def test_analyzer_invalid_result_fails(tmp_path):
    trial = _real_c1_like_trial(tmp_path)
    (trial / "validation.json").write_text(json.dumps({
        "overall_result": "INVALID_TRIAL",
        "channel_isolation": {"live_channel_active_available": True, "missing_expected_activation": None},
    }))
    r = _run_main(trial, "c1", "sess", tmp_path / "out")
    assert r.returncode != 0


def test_zero_force_trial_fails_end_to_end(tmp_path):
    trial = tmp_path / "trial"
    trial.mkdir()
    n = 50
    _write_bilateral_csv(trial / "raw_bilateral.csv", n,
                         source_vals=lambda i, j: 0.0, command_vals=lambda i, j: 0.0)
    _write_receiving_csv(trial / "raw_master.csv", n, relay_force=lambda i: 0.0, relay_applied=False)
    (trial / "manifest.json").write_text(json.dumps({
        "expected_channel_configuration": {"c1": True, "c2": False, "c3": False, "c4": False},
    }))
    (trial / "validation.json").write_text(json.dumps({
        "overall_result": "VALID",
        "channel_isolation": {"live_channel_active_available": True, "missing_expected_activation": None},
    }))
    (trial / "metrics.json").write_text(json.dumps({}))
    r = _run_main(trial, "c1", "sess", tmp_path / "out")
    assert r.returncode != 0
    assert not (tmp_path / "out" / "C1_QUALIFIED.json").exists()


def test_real_looking_trial_passes_end_to_end_and_carries_provenance(tmp_path):
    trial = _real_c1_like_trial(tmp_path)
    out_dir = tmp_path / "out"
    r = _run_main(trial, "c1", "sess123", out_dir)
    assert r.returncode == 0, r.stdout + r.stderr
    artifact = json.loads((out_dir / "C1_QUALIFIED.json").read_text())
    assert artifact["schema_version"] == "2.0.0"
    assert artifact["git_head"] == "deadbeef"
    assert artifact["config_fingerprint"] == "abc123"
    assert artifact["force_evidence"]["passed"] is True
    assert artifact["verdict"] == "QUALIFIED"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
