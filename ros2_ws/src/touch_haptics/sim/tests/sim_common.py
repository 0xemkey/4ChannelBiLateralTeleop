"""Shared helpers for sim/tests/*.py.

Plain pytest + subprocess, matching this repo's established rejection of
launch_testing (brittle, slow, hard to debug -- see identification/ and
projects/teleop_left_master_right_slave/scripts/ for the same convention).
Every test here launches touch_slave_sim (linked against hd_sim, never real
hardware) as a subprocess and talks to it over ordinary ROS topics.

Requires a sourced ROS 2 workspace with touch_haptics built
(TOUCH_HAPTICS_BUILD_SIM=ON, the default) -- these tests do not build anything
themselves. Run as: pytest sim/tests/ (from ros2_ws, after `source
install/setup.bash`), or via `colcon test`.
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
import uuid
from typing import Callable, Dict, List, Optional, Sequence, Type

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool

STARTUP_TIMEOUT_S = 15.0
SHUTDOWN_TIMEOUT_S = 5.0


def unique_ns(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex[:8]}"


def _resolve_touch_slave_sim_exe() -> str:
    """Path to the compiled touch_slave_sim binary, found via ament_index
    rather than shelling out through `ros2 run`.

    `ros2 run` spawns the target executable as its OWN child subprocess
    (confirmed by inspecting the process tree: two distinct PIDs, parent
    `ros2 run` python3 process and child touch_slave_sim, not an exec()
    replacement) -- so a SIGINT sent to `ros2 run`'s PID does not reliably
    reach the actual binary, which then never shuts down. Every SimNode-based
    test was leaving an orphaned touch_slave_sim process running indefinitely
    as a result (confirmed directly: one such orphan ran for 10+ minutes
    after its test's `with SimNode(...)` block had already exited), and
    read_truth_csv() intermittently failed on a still-being-written file
    racing against that orphan. Launching the binary directly makes
    `self.proc` the real process, so `send_signal`/`wait`/`kill` all target
    the thing that's actually running the servo loop.
    """
    prefix = get_package_prefix("touch_haptics")
    exe = os.path.join(prefix, "lib", "touch_haptics", "touch_slave_sim")
    if not os.path.isfile(exe):
        raise FileNotFoundError(
            f"touch_slave_sim not found at {exe} -- build touch_haptics "
            "with TOUCH_HAPTICS_BUILD_SIM=ON (the default) first"
        )
    return exe


class SimNode:
    """One touch_slave_sim subprocess, namespaced so parallel tests don't collide.

    `device` selects which side of hd_sim_api.cpp's is_right split applies:
    any name containing "Default" or "Right" gets the (deliberately different)
    right-arm mismatch parameters; anything else (e.g. "Left") gets the left-
    arm ones. This mirrors touch_slave.cpp's own device parameter mapping.
    """

    def __init__(self, namespace: str, device: str = "Left",
                params: Optional[Dict[str, str]] = None,
                sim_env: Optional[Dict[str, str]] = None) -> None:
        self.namespace = namespace
        self.device = device
        params = dict(params or {})
        params.setdefault("device", device)
        params.setdefault("publish_telemetry", "true")

        args = [
            _resolve_touch_slave_sim_exe(),
            "--ros-args",
            "-r", f"__ns:=/{namespace}",
        ]
        for k, v in params.items():
            args += ["-p", f"{k}:={v}"]

        env = dict(os.environ)
        for k, v in (sim_env or {}).items():
            env[str(k)] = str(v)

        self._log_path = f"/tmp/{namespace}.log"
        self._log = open(self._log_path, "w")
        self.proc = subprocess.Popen(
            args, env=env, stdout=self._log, stderr=subprocess.STDOUT, text=True)

    def topic(self, name: str) -> str:
        return f"/{self.namespace}/{name}"

    def wait_started(self, timeout: float = STARTUP_TIMEOUT_S) -> None:
        """Poll the log for the node's own startup line rather than sleeping a
        fixed guess -- configureDob() can take a few hundred ms to load and
        validate the identified JSON files."""
        deadline = time.time() + timeout
        marker = "started with max_force"
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError(
                    f"{self.namespace}: touch_slave_sim exited early (code "
                    f"{self.proc.returncode}); see {self._log_path}")
            try:
                with open(self._log_path) as f:
                    if marker in f.read():
                        return
            except FileNotFoundError:
                pass
            time.sleep(0.1)
        raise TimeoutError(
            f"{self.namespace}: touch_slave_sim did not report startup within "
            f"{timeout}s; see {self._log_path}")

    def log_text(self) -> str:
        try:
            with open(self._log_path) as f:
                return f.read()
        except FileNotFoundError:
            return ""

    def stop(self) -> None:
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=SHUTDOWN_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self._log.close()

    def __enter__(self) -> "SimNode":
        self.wait_started()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()


class ScriptProcess:
    """One Python script (bridge_master_slave.py, bilateral_4ch.py, ...)
    launched directly with sys.executable, same reasoning as
    _resolve_touch_slave_sim_exe() above: a plain subprocess.Popen is
    already the real process (no `ros2 run`-style wrapper to lose SIGINT
    through), so this needs none of that function's indirection -- but it
    mirrors SimNode's log-file/wait-for-marker/stop shape so tests that use
    both look the same.
    """

    def __init__(self, args: Sequence[str], name: str,
                env: Optional[Dict[str, str]] = None) -> None:
        self.name = name
        self._log_path = f"/tmp/{name}.log"
        self._log = open(self._log_path, "w")
        full_env = dict(os.environ)
        for k, v in (env or {}).items():
            full_env[str(k)] = str(v)
        self.proc = subprocess.Popen(
            [sys.executable, *args], env=full_env,
            stdout=self._log, stderr=subprocess.STDOUT, text=True)

    def wait_for_marker(self, marker: str, timeout: float = STARTUP_TIMEOUT_S) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError(
                    f"{self.name}: exited early (code {self.proc.returncode}); "
                    f"see {self._log_path}")
            try:
                with open(self._log_path) as f:
                    if marker in f.read():
                        return
            except FileNotFoundError:
                pass
            time.sleep(0.1)
        raise TimeoutError(
            f"{self.name}: did not see {marker!r} within {timeout}s; "
            f"see {self._log_path}")

    def log_text(self) -> str:
        try:
            with open(self._log_path) as f:
                return f.read()
        except FileNotFoundError:
            return ""

    def stop(self) -> None:
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=SHUTDOWN_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self._log.close()

    def __enter__(self) -> "ScriptProcess":
        return self

    def __exit__(self, *exc) -> None:
        self.stop()


class LiveNode:
    """One rclpy node combining arbitrary watched subscriptions and
    force-publishers, so a test can push a command and read a correlated
    response from the SAME live time series.

    Earlier ad hoc versions of these tests (2026-09-02 session) took two
    separate before/after snapshots with a plain ros2 topic echo --once,
    timed apart -- in an actively-changing system that produces misleading
    deltas (the two reads land at different moments, not the same one).
    Watching both sides continuously on one spin loop is what actually
    fixes that, not a smarter snapshot.
    """

    def __init__(self, name_prefix: str = "live") -> None:
        if not rclpy.ok():
            rclpy.init()
        self._node = Node(f"{name_prefix}_{uuid.uuid4().hex[:8]}")
        self.latest: Dict[str, object] = {}
        self._force_pubs: Dict[str, "rclpy.publisher.Publisher"] = {}
        self._enable_pubs: Dict[str, "rclpy.publisher.Publisher"] = {}

    def watch(self, key: str, topic: str, msg_type: Type,
             extract: Optional[Callable[[object], object]] = None) -> None:
        """Subscribes `topic`; self.latest[key] holds the newest message (or
        extract(message) if given), updated every time one arrives."""
        def _cb(msg: object, k: str = key, ex=extract) -> None:
            self.latest[k] = ex(msg) if ex is not None else msg
        self._node.create_subscription(msg_type, topic, _cb, 50)

    def push_force(self, force_topic: str, enable_topic: str,
                  fx: float, fy: float, fz: float) -> None:
        """Publishes one WrenchStamped + Bool(enable) sample. Call this every
        loop iteration inside spin_for's caller -- there is no real hand in
        sim, so a commanded force stands in for one, and it must be
        republished continuously the same way a real teleop loop does."""
        if force_topic not in self._force_pubs:
            self._force_pubs[force_topic] = self._node.create_publisher(
                WrenchStamped, force_topic, 10)
            self._enable_pubs[force_topic] = self._node.create_publisher(
                Bool, enable_topic, 10)
        m = WrenchStamped()
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = float(fx), float(fy), float(fz)
        self._force_pubs[force_topic].publish(m)
        e = Bool()
        e.data = True
        self._enable_pubs[force_topic].publish(e)

    def spin_once(self, timeout_sec: float = 0.01) -> None:
        rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    def spin_for(self, duration_s: float, timeout_sec: float = 0.01) -> None:
        deadline = time.time() + duration_s
        while time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    def close(self) -> None:
        self._node.destroy_node()

    def __enter__(self) -> "LiveNode":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class Collector:
    """Subscribes to one topic on a private rclpy node and buffers messages.

    A fresh rclpy context per collector rather than a shared long-lived node,
    so one test's leftover subscriptions cannot leak into the next -- pytest
    runs these in a single process by default.
    """

    def __init__(self, topic: str, msg_type: Type) -> None:
        if not rclpy.ok():
            rclpy.init()
        self._node = Node(f"collector_{uuid.uuid4().hex[:8]}")
        self.messages: List = []
        self._sub = self._node.create_subscription(
            msg_type, topic, self.messages.append, 50)

    def spin_for(self, duration_s: float) -> None:
        deadline = time.time() + duration_s
        while time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)

    def wait_for(self, count: int, timeout: float = 10.0) -> None:
        deadline = time.time() + timeout
        while len(self.messages) < count and time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
        if len(self.messages) < count:
            raise TimeoutError(
                f"only {len(self.messages)}/{count} messages on "
                f"{self._sub.topic_name} within {timeout}s")

    def close(self) -> None:
        self._node.destroy_node()


def read_truth_csv(path: str) -> List[dict]:
    """hd_sim_api.cpp's TOUCH_SIM_TRUTH_CSV output, as a list of dict rows."""
    import csv
    with open(path) as f:
        return [
            {k: float(v) for k, v in row.items()}
            for row in csv.DictReader(f)
        ]
