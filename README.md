# 4-Channel Bilateral Teleoperation — Portable Project Copy

Self-contained copy of the Touch/Omni master-slave bilateral teleoperation
system, assembled 2026-09-04 so this project can be built and run on a
different computer (same physical Touch/Omni devices) without depending on
anything else in the original `HIL-RL` repository.

**Status at time of copy:** all four channels (C1 slave-force→master, C2
slave-position→master, C3 master-position→slave, C4 master-force→slave)
confirmed working simultaneously on real hardware 2026-09-03. Phase 1 bug
fixes (see `projects/teleop_left_master_right_slave/docs/PHASE1_VERIFICATION_REPORT.md`)
completed 2026-09-04 under sim/code-review-only verification — the real-hardware
retest of item 1.4 (a one-off anomalous reading, investigated but not
reproduced in sim) is the first thing to run once hardware is available
again, using the script already written for it (see below).

**Read `projects/teleop_left_master_right_slave/docs/SYSTEM_ARCHITECTURE.pdf`
first** if you want to understand *how* the system works (channel
mechanism, DOB math, identified coefficients, diagrams) before *running*
it. This README is about getting it built and running; that PDF is the
reference for what it's actually doing.

---

## 1. Directory structure

```
4 channel bilateral teleop/
├── README.md                          <- you are here
├── ros2_ws/
│   └── src/
│       └── touch_haptics/             <- the ROS2 package (C++ nodes, sim, msgs)
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── include/touch_haptics/ <- touch_dob.hpp (DOB math), touch_force_estimate.hpp, dob_json_loader.hpp
│           ├── src/                   <- touch_slave.cpp (the main node, real hardware), touch_publisher*.cpp
│           ├── msg/                   <- ForceEstimate, DobDiagnostics, BilateralStatus
│           ├── sim/                   <- hd_sim_api.cpp + arm_model.hpp (hardware-free simulator), sim/tests/ (pytest suite)
│           ├── tools/                 <- standalone calibration/identification utilities (.cpp sources, not auto-built)
│           ├── identification/        <- Python fitting scripts (gravity/inertia/friction) + README
│           └── tests/                 <- framework-free C++ unit tests (test_touch_dob.cpp)
├── projects/
│   └── teleop_left_master_right_slave/
│       ├── README.md                  <- original project overview + force-model background
│       ├── RUNNING_SCRIPTS.txt        <- detailed runbook (terminal-by-terminal, all tuning knobs)
│       ├── scripts/                   <- bilateral_4ch.py, bridge_master_slave.py, run_*.sh launch scripts, monitors
│       └── docs/                      <- FOUR_CHANNEL_TODO.md (full dev log), SYSTEM_ARCHITECTURE.pdf,
│                                          PHASE1_VERIFICATION_REPORT.md, this session's other writeups
└── dob_logs/
    ├── results/                       <- identified DOB coefficients (gravity/inertia/friction fits, both arms)
    └── teleop_calibration.json        <- master/slave anchor-pose calibration
```

This mirrors the original repo's relative layout on purpose
(`ros2_ws/` and `projects/teleop_left_master_right_slave/` as siblings)
— every launch script computes its own workspace path relative to itself
(`REPO_ROOT="$(cd "${PROJECT_ROOT}/../.." && pwd)"`), so **no script had to
be edited** to make this folder self-contained. Verified directly: placing
this folder anywhere and running `run_master_left.sh` from inside it
resolves `TOUCH_WS` to `<this folder>/ros2_ws` automatically.

---

## 2. External dependencies (NOT bundled — install these first)

| dependency | why | typical location |
|---|---|---|
| **ROS2 Humble** | `ament_cmake`, `rclcpp`, `geometry_msgs`, `std_msgs`, `rosidl` | `/opt/ros/humble` |
| **OpenHaptics SDK** (proprietary, 3D Systems) | `HD`/`HDU` headers + libraries — required to build the **real hardware** nodes, and ALSO required to build the **simulator** (`hd_sim`), since this package borrows OpenHaptics' own `HD/hd.h` enum/struct definitions for bit-identical semantics rather than shipping a vendored fallback header set. There is no way to build anything in this package, sim included, without OpenHaptics headers present. | `/opt/OpenHaptics` (or wherever `hdapi`/`hduVector` install to) |
| **C++17 compiler** | the whole package targets `CMAKE_CXX_STANDARD 17` | gcc/g++ or clang |
| **Python 3** + `numpy`, `scipy` | `identification/*.py` fitting scripts only (gravity/inertia/friction regression). Not needed just to run the teleop system itself. | — |
| **colcon** | ROS2 build tool | usually ships with ROS2 desktop install |

Not bundled and not needed to run this system: `pytest` is only required
if you want to run the sim regression suite (`pip install pytest`,
already present on the original machine).

---

## 3. Calibration data — read this before running anything

Two files this system needs at runtime are **NOT resolved relative to
this folder** — they default to paths under `$HOME` on whatever machine
runs the nodes, because that's how the original scripts and
`touch_slave.cpp`'s own ROS parameter defaults were written
(`getenv("HOME") + "/dob_logs/..."`):

| file (bundled here) | default runtime location the code looks for |
|---|---|
| `dob_logs/results/gravity_left_minimal.json` | `$HOME/dob_logs/results/gravity_left_minimal.json` |
| `dob_logs/results/inertia_left.json` | `$HOME/dob_logs/results/inertia_left.json` |
| `dob_logs/results/gravity_right_minimal.json` | `$HOME/dob_logs/results/gravity_right_minimal.json` |
| `dob_logs/results/inertia_right.json` | `$HOME/dob_logs/results/inertia_right.json` |
| `dob_logs/teleop_calibration.json` | `$HOME/dob_logs/teleop_calibration.json` |

**Two ways to satisfy this on a new machine**, pick one:

1. **Simplest — copy the bundled folder to `$HOME`:**
   ```bash
   cp -r "dob_logs" ~/dob_logs
   ```
   Every script and node then finds everything at its default path with
   zero configuration, matching the original rig's layout exactly.

2. **Or — point the env vars at this folder instead**, if you don't want
   anything written outside this directory (every launch script and node
   parameter accepts an override):
   ```bash
   export DOB_GRAVITY_JSON="$(pwd)/dob_logs/results/gravity_left_minimal.json"   # Terminal 1
   export DOB_INERTIA_JSON="$(pwd)/dob_logs/results/inertia_left.json"           # Terminal 1
   # (repeat with _right_ filenames for Terminal 2, and see run_*.sh for the exact var names)
   export CALIBRATION_FILE="$(pwd)/dob_logs/teleop_calibration.json"             # Terminal 2 (bridge)
   ```

**What's deliberately NOT bundled:** the raw identification captures
(`dyn_Left.csv`, `dyn_right_merged.csv`, `sweep_*.csv` — tens of MB each,
`$HOME/dob_logs/*.csv` on the original machine) that the JSON files above
were *fitted from*. Those are only needed to re-run identification
(`identification/fit_gravity.py` etc.), not to run the teleop system, and
were left out to keep this folder small. If a full re-identification is
ever needed on new hardware, start from `identification/README.md` — the
fitting pipeline is fully bundled, only its historical raw input data
isn't.

**`teleop_calibration.json` is rig-specific, not computer-specific.** It
records the geometric offset between where the left and right Touch
devices sit relative to each other on THIS physical rig. It travels
correctly to a new computer driving the *same* two physical devices in the
*same* physical arrangement. If the devices are ever repositioned, or this
is used with a different pair of devices, delete it and let
`bridge_master_slave.py` recapture a fresh one on next launch (see
`RUNNING_SCRIPTS.txt`'s "Calibration file" section).

---

## 4. Build

```bash
cd "ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --base-paths src --cmake-args -DOPENHAPTICS_ROOT=/opt/OpenHaptics
```

Adjust `-DOPENHAPTICS_ROOT` to wherever the SDK is actually installed on
this machine (CMake also searches `/usr`, `/usr/local` automatically —
omit the flag entirely if it's on the default system paths).

**Build options** (pass as additional `--cmake-args`, all defined in
`ros2_ws/src/touch_haptics/CMakeLists.txt`):

| option | default | effect |
|---|---|---|
| `TOUCH_HAPTICS_BUILD_SIM` | `ON` | builds `touch_slave_sim` + the simulator library — works with no real devices attached |
| `TOUCH_HAPTICS_BUILD_HW` | `ON` | builds the real-hardware `touch_slave`/`touch_publisher` — requires OpenHaptics to actually be found, else this silently drops to `OFF` with a warning |
| `TOUCH_HAPTICS_ENABLE_REAL_TORQUE_WRITE` | `OFF` | **safety gate.** Only when explicitly `ON` can the real `touch_slave` binary write joint torque (C1/C4's relay path) to real motors via `joint_space_control:=true`. Leave `OFF` unless you specifically intend to test that path, and confirm `dob_torque_f_max_n` is set low with an e-stop ready first — see the CMake warning it prints when enabled. |

After building:
```bash
source "ros2_ws/install/setup.bash"
```
(needed in every terminal before running any node or script below).

---

## 5. Run — four terminals

Full knob-by-knob detail lives in
`projects/teleop_left_master_right_slave/RUNNING_SCRIPTS.txt`. Quick
version (all commands run from inside this folder):

```bash
# Terminal 1 — master (left)
projects/teleop_left_master_right_slave/scripts/run_master_left.sh

# Terminal 2 — bridge (position channels C2/C3,+ optional force reflection)
SCALE=1.0 AXIS_MAP=x,y,z FORCE_REFLECT=true REFLECT_GAIN=25.0 REFLECT_MAX_N=1.5 REPORT=true \
  projects/teleop_left_master_right_slave/scripts/run_bridge.sh

# Terminal 3 — slave (right)
STIFFNESS_N_PER_M=35.0 DAMPING_N_S_PER_M=0.20 MAX_FORCE_N=1.5 \
  projects/teleop_left_master_right_slave/scripts/run_slave_right.sh

# Terminal 4 — force relay (C1/C4), only after 1-3 are confirmed healthy
python3 projects/teleop_left_master_right_slave/scripts/bilateral_4ch.py \
  --architecture custom --w1 0.4 --w2 0 --w3 0 --w4 0.4
```

Order matters for terminals 1-3: **master, then bridge, then slave** (the
bridge captures each side's calibration anchor as it first sees that
device's pose — see §3 above). Terminal 4 is independent of that ordering
but needs 1 and 3's `force_ext_hat` topics already publishing.

**`RUNNING_SCRIPTS.txt`'s example commands use `$HOME/HIL-RL/...`
paths from the original repo layout — replace that prefix with wherever
you placed this folder** (or just run from inside this folder as shown
above, using relative paths).

---

## 6. Verify without touching hardware

```bash
cd "ros2_ws"
source /opt/ros/humble/setup.bash && source install/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1   # works around an anyio/pytest plugin conflict
python3 -m pytest src/touch_haptics/sim/tests/ -v
```
33 tests should pass, 1 skip (pre-existing, unrelated to this project —
missing external tool binary). Two hardware-free `touch_slave_sim`
instances stand in for the real master/slave throughout.

---

## 7. First thing to do once real hardware is available again

`projects/teleop_left_master_right_slave/scripts/repro_dob_restart_anomaly.py`
— written and syntax-checked, **never executed** (no hardware access when
it was written). Restarts only the master node ~25 times with nothing
touching either arm, checking whether a single 2.79 N·m anomalous reading
seen once on 2026-09-03 (investigated in Phase 1, sim-ruled-out as
filter-state or DOB-math related, real-hardware-specific device re-init
still an open hypothesis) reproduces from a restart alone:

```bash
python3 projects/teleop_left_master_right_slave/scripts/repro_dob_restart_anomaly.py \
  --runs 25 --out-dir ~/dob_logs/restart_repro_$(date +%Y%m%d_%H%M%S)
```

Prerequisite: slave (Terminal 3 above) and `bilateral_4ch.py` (Terminal 4)
already running — this script only restarts the master. Full context in
`docs/PHASE1_VERIFICATION_REPORT.md` §1.4.

---

## 8. Where the fuller documentation lives

| file | what it's for |
|---|---|
| `projects/teleop_left_master_right_slave/docs/SYSTEM_ARCHITECTURE.pdf` | mechanism, math, diagrams, verification proofs — the "how does this work" reference |
| `projects/teleop_left_master_right_slave/docs/FOUR_CHANNEL_TODO.md` | full chronological development log — every bug found, every decision made, with dates |
| `projects/teleop_left_master_right_slave/docs/PHASE1_VERIFICATION_REPORT.md` | 2026-09-04 bug-fix verification report (PASS/FAIL per item) |
| `projects/teleop_left_master_right_slave/README.md` | original project overview, force-model background |
| `projects/teleop_left_master_right_slave/RUNNING_SCRIPTS.txt` | the detailed runbook — every environment variable, every tuning knob, troubleshooting |
| `ros2_ws/src/touch_haptics/identification/README.md` | how the gravity/inertia/friction fits were produced, if re-identification is ever needed |

---

## 9. Update policy for this copy

This folder is a **point-in-time snapshot**, taken 2026-09-04 after Phase
1's sim/code-review-only verification. It is not a live symlink or a git
submodule of the original `HIL-RL` repository — changes made in one do not
automatically appear in the other. Per the plan agreed with the project
owner: after the next real-hardware session (Phase 1.4's real repro
attempt, then Phases 2-3 of the fine-tuning task), **both this folder and
the original `HIL-RL/projects/teleop_left_master_right_slave` /
`HIL-RL/ros2_ws` tree will be updated together**, so they do not silently
drift apart. If you're reading this later and only one of the two has
been updated, treat the more recently modified one as authoritative and
re-sync the other before trusting either blindly.
