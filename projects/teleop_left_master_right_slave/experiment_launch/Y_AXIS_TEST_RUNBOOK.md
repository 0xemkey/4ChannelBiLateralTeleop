# Y-axis diagnosis — operator runbook

Purpose: isolate the Y-axis position tracking problem, per the task in
conversation on 2026-09-08. C1=C2=C4 off, C3 only. Run every command below
yourself — starting the slave needs an interactive `sudo` password, and the
motion steps need a hand on the master stylus, neither of which I can do.

Pick one session id and reuse it in every command below:

```
SESSION=y_axis_diag_$(date +%Y%m%d)
```

## Stage 0 — bring up the passive rig (zero force, no bridge yet)

Terminal 1:
```bash
cd ~/HIL-RL
projects/teleop_left_master_right_slave/experiment_launch/00_preflight.sh \
  --session "$SESSION" --notes "Y-axis position diagnosis, C3 only"
```

Terminal 2:
```bash
cd ~/HIL-RL
projects/teleop_left_master_right_slave/experiment_launch/01_start_master.sh --session "$SESSION"
```

Terminal 3 (will prompt for your sudo password — this is the ttyACM1→ttyACM0
mount-bind, expected):
```bash
cd ~/HIL-RL
projects/teleop_left_master_right_slave/experiment_launch/02_start_slave.sh --session "$SESSION"
```

Terminal 4 — sanity check before arming anything:
```bash
cd ~/HIL-RL
projects/teleop_left_master_right_slave/experiment_launch/05_show_status.sh --session "$SESSION"
```
Confirm: both nodes running, pose/DOB topics fresh, `joint_space_control=false`,
no bridge yet, `applied_force` ≈ 0 on both sides.

## Stage 1 — gravity-comp A/B (task step 3)

**Test A (gravity comp ON — the real production default, 0.55 N on Y):**

Terminal 4:
```bash
projects/teleop_left_master_right_slave/experiment_launch/03_start_position_bridge.sh \
  --session "$SESSION" --arm-position
```
This is C3=1, C1=C2=C4=0, exactly as required. Keep clear of both workspaces
before running it.

Terminal 5 — capture while you slowly move the MASTER along Y only (small
motion, a few cm, a couple of slow back-and-forth cycles is enough for this
comparison — Ctrl-C to stop):
```bash
cd ~/HIL-RL/projects/teleop_left_master_right_slave/scripts
python3 log_phase2_scenario.py --scenario single_axis_slow --axis y \
  --repeat 1 --out-dir ~/dob_logs/y_axis_diag/gravity_on
```

**Stop only the bridge (keep master/slave running) before Test B:**

Terminal 4:
```bash
SESSION_DIR=~/HIL-RL/experiment_runs/teleop_setup/$SESSION
kill -TERM "$(cat "$SESSION_DIR/pids/bridge.pid")"
sleep 2
kill -0 "$(cat "$SESSION_DIR/pids/bridge.pid")" 2>/dev/null && echo "STILL ALIVE — stop here, tell me" || echo "bridge stopped cleanly"
rm -f "$SESSION_DIR/pids/bridge.pid" "$SESSION_DIR/pids/bridge.pgid"
```

**Test B (gravity comp OFF — short diagnostic only, reuses the SAME
calibration file so anchors match Test A):**

Terminal 4:
```bash
cd ~/HIL-RL/projects/teleop_left_master_right_slave/scripts
SESSION_DIR=~/HIL-RL/experiment_runs/teleop_setup/$SESSION
ENABLE_C2=false FORCE_REFLECT=false SCALE=1.0 AXIS_MAP=x,y,z AXIS_SCALE=1.0,1.0,1.0 \
MAX_OFFSET_XYZ=0.12,0.19,0.15 MAX_SPEED_MPS=0.5 RATE_HZ=100 \
SLAVE_GRAVITY_COMP_N=0 SLAVE_GRAVITY_AXIS=1 \
CALIBRATION_FILE="$SESSION_DIR/teleop_calibration.json" \
./run_bridge.sh
```
This is not session-tracked (deliberately, it's a short diagnostic per the
task) — stop it yourself with Ctrl-C in this terminal when done. Keep clear
of the workspace before running it, same as Test A.

Terminal 5 — same capture, same motion pattern as Test A (repeat the same
physical motion as closely as you can, so the two are comparable):
```bash
cd ~/HIL-RL/projects/teleop_left_master_right_slave/scripts
python3 log_phase2_scenario.py --scenario single_axis_slow --axis y \
  --repeat 1 --out-dir ~/dob_logs/y_axis_diag/gravity_off
```

After Test B: Ctrl-C the Terminal 4 `run_bridge.sh` process, confirm it's
dead (`ps aux | grep bridge_master_slave`), then continue to Stage 2.

## Stage 2 — small-amplitude Y-only sweep (task step 5, production gravity-comp ON)

Re-arm the bridge properly through the tracked launcher (now that Test B's
manual process is confirmed stopped):
```bash
projects/teleop_left_master_right_slave/experiment_launch/03_start_position_bridge.sh \
  --session "$SESSION" --arm-position
```

For each amplitude, move the MASTER slowly along Y only, roughly that many mm
peak-to-peak, a few slow cycles, avoiding any X/Z motion:

```bash
cd ~/HIL-RL/projects/teleop_left_master_right_slave/scripts
python3 log_phase2_scenario.py --scenario single_axis_slow --axis y --repeat 1 \
  --out-dir ~/dob_logs/y_axis_diag/amp_5mm
python3 log_phase2_scenario.py --scenario single_axis_slow --axis y --repeat 1 \
  --out-dir ~/dob_logs/y_axis_diag/amp_10mm
python3 log_phase2_scenario.py --scenario single_axis_slow --axis y --repeat 1 \
  --out-dir ~/dob_logs/y_axis_diag/amp_20mm
```
One rep each is enough for the first pass; add `--repeat 2`/`3` (same
`--out-dir`) later if we need to confirm reproducibility.

## Shutdown (any time)

```bash
projects/teleop_left_master_right_slave/experiment_launch/99_stop_setup.sh --session "$SESSION"
```

## When you're done

Tell me the session id and the `~/dob_logs/y_axis_diag/...` directory — I'll
read the CSVs, compute the metrics, generate the required plots, and write
`docs/Y_AXIS_DIAGNOSIS.md`.
