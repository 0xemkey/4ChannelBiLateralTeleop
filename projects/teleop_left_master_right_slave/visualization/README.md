# Four-channel visualization

Observer tools for synthetic data, isolated simulation telemetry, and existing
file replay. Every file in this directory belongs to visualization. No controller,
hardware process, command publisher, recorder, or experiment is launched here.
Hardware observation is a future extension only; it is not an executable mode.

## Launch

From the repository root:

```bash
bash projects/teleop_left_master_right_slave/visualization/run_4ch_sim_ui.sh --source synthetic
```

This opens a Matplotlib dashboard and 3D scene without ROS. Add `--rviz` to
also publish display-only MarkerArray messages and open RViz. Source your
existing ROS installation before using ROS modes; these tools do not build or
modify shared packages. Dependencies: Python, NumPy, Matplotlib, PyYAML, Pillow;
ROS modes additionally need rclpy, rosidl_runtime_py, visualization_msgs and the
existing touch_haptics message package. GUI mode needs a Matplotlib GUI backend.

```bash
bash projects/teleop_left_master_right_slave/visualization/run_4ch_sim_ui.sh --source simulation --rviz
bash projects/teleop_left_master_right_slave/visualization/run_4ch_sim_ui.sh --source simulation --rviz --dry-run
bash projects/teleop_left_master_right_slave/visualization/run_4ch_replay_ui.sh projects/teleop_left_master_right_slave/visualization/artifacts/synthetic.json
```

Simulation mode observes an already running, independently owned simulator.
It does not start touch_slave_sim, bridges, trajectories, or control processes.
Configure its existing telemetry topic names in `config/visualizer_topics.yaml`;
all topics must remain under `/teleop_4ch_sim/`. Launchers enforce domain 42
and localhost discovery. Direct ROS entry points require ROS_DOMAIN_ID=42 and
reject every other domain before initializing ROS. ROS arguments/remappings
are not accepted. Detected root live-node names stop the observer in domain 42.
No ROS domain 0 discovery or subscriptions are performed.

`demo_telemetry_publisher.py` is an optional normalized JSON display publisher,
restricted to `/teleop_4ch_sim/visualization/demo`. The built-in synthetic UI
generates the same deterministic data locally and does not require that publisher.
Only that topic and `/teleop_4ch_sim/visualization/markers` can be published by
these tools. RViz markers are the sole publication made by the visualizer.

## Signal contract

Normalized records contain `t` (original seconds), a 3x3 `mapping`, and `signals`.
Each signal contains `value`, original `t`, and `status` (OK, N/A, BLOCKED or
INVALID). Data older than 0.5 seconds is STALE. Missing values are plot gaps,
never implicit zeros. The adapter maps existing message fields via YAML and
retains per-signal timestamps. Source labels are chosen by the executable mode,
never trusted from incoming data. The configuration hash identifies the observer
mapping, not the controller's production configuration.

The default slave adapter directly reads spring_force_n, damping_force_n,
auxiliary_force_n, force_before_final_clamp_n, force_after_primary_clamp_n,
applied_force_n, and final_force_clamped. Applied Cartesian force is BLOCKED
when cartesian_write_active is false. Auxiliary force is not relabeled gravity.
C2/C3 status and relative positions come from BridgeDiagnostics. C2/C3 are
position channels; no dedicated force decomposition is inferred for them.
C1/C4 force vectors and status default to BLOCKED; final command, absolute
latched anchors and joint DOB default to N/A unless mapped to authoritative
existing telemetry. No Cartesian DOB conversion is implemented. The simulation
adapter deliberately ignores Cartesian DOB mappings. The synthetic source includes
explicitly generated arm geometry and a synthetic Cartesian DOB prediction so the
applied-versus-predicted UI can be inspected. Synthetic C1/C4 and DOB values
demonstrate appearance only and are not control validation evidence.

Scenes use anchor-relative coordinates. Master/slave relative origins coincide;
absolute anchor values appear in RViz text only when supplied. Targets are mapped
relative master positions. The dashboard separately reports the controller's
actual target. The contact plane is shown only for synthetic data; no environment
geometry is fabricated in simulation or replay. Arrows use the configured m/N
scale and display-length cap; numeric vectors retain their true values.

Position error is slave_delta minus mapped_master_delta. Absolute fallback
requires both actual anchors. Metrics report per-axis RMSE, peak norm, fitted
slave-to-mapped gain, delay search within +/-0.5 seconds, Y positive/negative
motion RMSE, force peak and clamp duty. Delay estimation interpolates timestamps;
constant axes report N/A. Loss/jitter refer to normalized observation samples
against configured expected_hz, not DDS packet-loss measurements. UI history is
limited to 600 observations. Metrics describe that history, not a whole session.

Replay accepts normalized JSON arrays, JSONL, or CSV with `t`, `mapping` and
`signals` columns (the latter two JSON-encoded). Timestamps must be increasing;
playback preserves them and holds the final frame. Arbitrary legacy CSV schemas
and rosbag are not imported automatically; an explicit read-only conversion to
the normalized contract is required. No replay telemetry is republished as
controller messages. Replay labels remain REPLAY, even for synthetic fixtures.

## Headless verification

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLCONFIGDIR=/tmp/teleop-viz-mpl python3 -m pytest -o cache_dir=projects/teleop_left_master_right_slave/visualization/.pytest_cache projects/teleop_left_master_right_slave/visualization/tests -q
MPLCONFIGDIR=/tmp/teleop-viz-mpl python3 projects/teleop_left_master_right_slave/visualization/teleop_4ch_dashboard.py --source synthetic --artifacts projects/teleop_left_master_right_slave/visualization/artifacts
```

Artifacts include dashboard_synthetic.png, scene_synthetic.png,
synthetic_sequence.gif, synthetic.json and metrics.json. The scene screenshot
uses the Matplotlib 3D renderer, not an RViz capture. The synthetic sequence
shows READY, ramps, settling, +Y/hold/return, contact, deadman release and disable.
None of these artifacts constitutes evidence of real four-channel control.
