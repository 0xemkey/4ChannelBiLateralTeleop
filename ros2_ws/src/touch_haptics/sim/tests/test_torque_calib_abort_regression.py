"""Standing regression test for tools/hdapi_torque_calib.cpp's missing-SIGINT
fix (0.2, 2026-08-25). Before that fix this tool had NO signal handler at
all: Ctrl-C terminated the process with a force still latched, leaving
whether the device went passive up to the driver's watchdog. This was
verified once by hand against a stub harness (93 force writes after SIGINT,
zero non-zero); this test makes that a standing check against the real tool,
compiled against hd_sim so it needs no hardware.

Compiles tools/hdapi_torque_calib.cpp fresh (it is a standalone g++ build,
not part of the CMake package -- see its own BUILD comment) linking against
libhd_sim.so instead of the real -lHD/-lHDU, runs it against a simulated
device, sends SIGINT partway through the sweep, and checks hd_sim's own
TOUCH_SIM_TRUTH_CSV ground truth: every commanded torque sample AFTER the
signal must be exactly zero, and at least one sample BEFORE it must be
nonzero (proving the test actually exercised a live force, not a no-op).
"""
from __future__ import annotations

import glob
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import read_truth_csv, unique_ns  # noqa: E402

PKG_ROOT = Path(__file__).resolve().parents[1]  # .../touch_haptics
TOOL_SRC = PKG_ROOT / "tools" / "hdapi_torque_calib.cpp"
INSTALL_LIB = PKG_ROOT.parent.parent / "install" / "touch_haptics" / "lib"


def _find_hd_sim_lib() -> Path:
    for candidate in (
        INSTALL_LIB / "libhd_sim.so",
        PKG_ROOT.parent.parent / "build" / "touch_haptics" / "libhd_sim.so",
    ):
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        "libhd_sim.so not found -- build touch_haptics with "
        "TOUCH_HAPTICS_BUILD_SIM=ON first (colcon build --packages-select touch_haptics)"
    )


@pytest.fixture(scope="module")
def compiled_tool(tmp_path_factory):
    if not TOOL_SRC.exists():
        pytest.skip(f"{TOOL_SRC} not found")
    gxx = shutil.which("g++")
    if not gxx:
        pytest.skip("no g++ available to compile the standalone tool")

    hd_sim_lib = _find_hd_sim_lib()
    out_dir = tmp_path_factory.mktemp("torque_calib_build")
    out_bin = out_dir / "hdapi_torque_calib_sim"

    include_dir = PKG_ROOT / "sim" / "src"  # HD/hd.h fallback if no real headers
    real_include = None
    for cand in ("/usr/include", "/usr/local/include"):
        if (Path(cand) / "HD" / "hd.h").exists():
            real_include = cand
            break

    cmd = ["g++", "-std=gnu++17", "-O2", str(TOOL_SRC), "-o", str(out_bin)]
    if real_include:
        cmd += ["-I", real_include]
    else:
        cmd += ["-I", str(PKG_ROOT / "sim" / "include")]
    cmd += [f"-L{hd_sim_lib.parent}", "-lhd_sim", "-lpthread",
           f"-Wl,-rpath,{hd_sim_lib.parent}"]

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        pytest.fail(f"compile failed:\n{result.stdout}\n{result.stderr}")
    return out_bin


def test_abort_zeroes_force_within_the_run(compiled_tool):
    ns = unique_ns("torquecalib")
    truth_prefix = f"/tmp/{ns}_truth"
    env = dict(os.environ)
    env["TOUCH_SIM_TRUTH_CSV"] = truth_prefix

    proc = subprocess.Popen(
        [str(compiled_tool), "Left"], env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    try:
        # Let it get partway into the level sweep (each level holds ~0.5s,
        # four levels -- 1.2s lands inside a nonzero level) before aborting.
        time.sleep(1.2)
        abort_wall_time = time.time()
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=5.0)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()

    matches = glob.glob(truth_prefix + "*.csv")
    assert matches, f"no truth CSV at {truth_prefix}*.csv -- did the tool even start?"
    rows = read_truth_csv(matches[0])
    assert len(rows) > 50, f"truth CSV too short: {len(rows)} rows"

    # t_s is sim time from process start, at ~1000 Hz -- abort happened at
    # roughly abort_wall_time - proc_start, but simplest and robust: split the
    # row list itself, since t_s is monotonic and SIGINT landed close to the
    # 1.2s mark by construction.
    before = [r for r in rows if r["t_s"] < 1.15]
    after = [r for r in rows if r["t_s"] > 1.3]

    assert before, "no samples before the abort window"
    assert any(
        abs(r["tau_cmd1"]) + abs(r["tau_cmd2"]) + abs(r["tau_cmd3"]) > 1e-6
        for r in before
    ), "no nonzero commanded torque before the abort -- the sweep never armed force"

    assert after, "no samples after the abort window (process may have died too fast)"
    nonzero_after = [
        r for r in after
        if abs(r["tau_cmd1"]) + abs(r["tau_cmd2"]) + abs(r["tau_cmd3"]) > 1e-9
    ]
    assert not nonzero_after, (
        f"{len(nonzero_after)}/{len(after)} samples AFTER the abort window still "
        f"carry nonzero commanded torque -- the SIGINT handler regressed"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
