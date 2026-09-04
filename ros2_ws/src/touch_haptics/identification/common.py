"""Shared kinematics, log loading, and unit policy for DOB parameter fitting.

PROVENANCE POLICY
-----------------
Every number this suite produces comes from data logged off THIS device by the
C++ tools in ../tools/. Nothing is read from the legacy MATLAB material
(SetUp.m, initiatesystems.m, omni10.mat, dob.mat). Those files are mentioned
here only to record that they are deliberately excluded: their provenance was
never established, one of them contained a non-physical inertia (I2 = 0), and a
fit that silently inherits an unverified constant is worse than no fit.

UNITS -- read this before interpreting any output
-------------------------------------------------
There are two torque quantities in the logs and they are NOT in the same units:

  tau_cmd = J^T(q) @ F_cmd      REAL N*m.  F_cmd is a Cartesian force we chose,
                                in newtons, exactly known. So this is in true
                                SI units with no calibration whatsoever.

  raw_lastjt (HD_LAST_JOINT_TORQUE)
                                UNCALIBRATED raw driver units. The scale factor
                                to N*m is unknown until hdapi_torque_calib.cpp
                                has been run.

This suite fits everything from tau_cmd, so M_n comes out in real kg*m^2 and
gravity/friction in real N*m WITHOUT requiring the torque-scale calibration.
raw_lastjt is carried through only as an independent cross-check: dividing
tau_cmd by raw_lastjt gives a per-joint scale estimate that can be compared
against hdapi_torque_calib.cpp's answer from completely different data.

THE DEPENDENCY THIS DOES HAVE
-----------------------------
tau_cmd is only correct if J(q) is correct -- both the Jacobian algebra and the
assumption that HDAPI's joint-angle convention matches the one the Jacobian was
written in. That is exactly what hdapi_fk_residual.cpp tests. Run it and get a
PASS before trusting any number here. If it fails, every N*m figure in this
suite is wrong by an unknown rotation, and no amount of curve fitting fixes it.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field

import numpy as np

# Mirrored from ../include/touch_haptics/touch_dob.hpp. If those constants
# change, change them here too -- there is no shared source of truth between
# the C++ and Python sides, which is a known and accepted duplication.
L1_METERS = 0.135
L2_METERS = 0.135

JOINT_NAMES = ("waist", "shoulder", "elbow")


def jacobian(q, l1: float = L1_METERS, l2: float = L2_METERS) -> np.ndarray:
    """J(q), Jarillo-Silva Eq. (15).

    Byte-for-byte the same algebra as touch_dob.hpp::jacobian. Row i is the
    Cartesian axis, column j the joint, so tau = J.T @ F.
    """
    c1, s1 = math.cos(q[0]), math.sin(q[0])
    c2, s2 = math.cos(q[1]), math.sin(q[1])
    c3, s3 = math.cos(q[2]), math.sin(q[2])
    return np.array([
        [-(l1 * c1 * c2 + l2 * s3 * c1),  l1 * s1 * s2,  -l2 * c3 * s1],
        [0.0,                             l1 * c2,        l2 * s3],
        [-l1 * c2 * s1 - l2 * s3 * s1,   -l1 * s2 * c1,   l2 * c3 * c1],
    ])


def fk(q, l1: float = L1_METERS, l2: float = L2_METERS) -> np.ndarray:
    """FK implied by the above Jacobian (touch_dob.hpp::fkFromJacobianConvention)."""
    r = l1 * math.cos(q[1]) + l2 * math.sin(q[2])
    return np.array([-math.sin(q[0]) * r,
                     l1 * math.sin(q[1]) - l2 * math.cos(q[2]),
                     math.cos(q[0]) * r])


def tau_from_force(q, f_cartesian) -> np.ndarray:
    """tau = J^T(q) @ F, in REAL N*m when F is in newtons."""
    return jacobian(q).T @ np.asarray(f_cartesian, dtype=float)


def singularity_metrics(q, l1: float = L1_METERS, l2: float = L2_METERS):
    """Mirrors touch_dob.hpp::singularityMetrics. Returns (radial_r, align_cos, near)."""
    radial_r = l1 * math.cos(q[1]) + l2 * math.sin(q[2])
    align_cos = math.cos(q[1] - q[2])
    near = abs(radial_r) < 0.02 or abs(align_cos) < 0.20
    return radial_r, align_cos, near


# ---------------------------------------------------------------------------
# Log loading
# ---------------------------------------------------------------------------

def _rows(path: str, tool: str) -> list[dict]:
    out = []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            if row.get("tool") == tool:
                out.append(row)
    if not out:
        raise SystemExit(f"{path}: no rows with tool=={tool!r}. Wrong log file?")
    return out


@dataclass
class SweepRow:
    """One ramp attempt from hdapi_static_sweep --log."""
    pose_id: int
    direction: str            # "+X", "-X", ...
    broke: bool
    break_force_n: float
    q0: np.ndarray
    q_break: np.ndarray
    tau_nm: np.ndarray        # REAL N*m, J^T F at breakaway, RAMP FORCE ONLY
    raw_lastjt: np.ndarray    # uncalibrated units
    slipped_joint: str
    max_dq_rad: float
    # Constant known support force held during the ramp, newtons, Cartesian.
    # Zero for logs written before the --bias-n option existed, which is the
    # correct reading: those runs had no bias.
    #
    # It exists because this arm only self-holds where gravity torque is nearly
    # zero. A commanded bias lets the sagging poses be measured; because its
    # value is known exactly it cancels from friction and offsets gravity by a
    # subtractable J^T(q0) @ bias -- unlike a supporting hand, whose force is
    # unknown and therefore lands in the gravity estimate as an unquantifiable
    # error. See fit_gravity.decompose, which does the subtraction.
    bias_n: np.ndarray = field(default_factory=lambda: np.zeros(3))


def load_static_sweep(path: str) -> list[SweepRow]:
    out = []
    for r in _rows(path, "static_sweep"):
        out.append(SweepRow(
            pose_id=int(r["pose_id"]),
            direction=r["dir"],
            broke=r["broke"] == "1",
            break_force_n=float(r["break_force_n"]),
            q0=np.array([float(r[f"q0_{i}"]) for i in (1, 2, 3)]),
            q_break=np.array([float(r[f"qbreak_{i}"]) for i in (1, 2, 3)]),
            tau_nm=np.array([float(r[f"tau_nm_{i}"]) for i in (1, 2, 3)]),
            raw_lastjt=np.array([float(r[f"raw_lastjt_{i}"]) for i in (1, 2, 3)]),
            slipped_joint=r["slipped_joint"],
            max_dq_rad=float(r["max_dq_rad"]),
            bias_n=np.array([float(r.get(f"bias_{i}", 0.0) or 0.0) for i in (1, 2, 3)]),
        ))
    return out


@dataclass
class Capture:
    """One chirp capture from hdapi_dynamic_excitation --log."""
    pose_id: int
    axis: str
    amp_n: float
    f0_hz: float
    f1_hz: float
    dur_s: float
    t: np.ndarray = field(default_factory=lambda: np.empty(0))
    cmd: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    q: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    raw_lastjt: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    pos_mm: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    # Constant support force held frozen through this capture, newtons.
    # NOTE: cmd already INCLUDES this -- cmd is the total commanded force. The
    # bias is recorded separately only so its contribution can be inspected. Do
    # not subtract it from cmd: the whole commanded force is what accelerates
    # the arm, and removing the part that balances gravity would break the
    # regression rather than clean it up.
    bias_n: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # Frames where bias+chirp hit the force clamp. Non-zero means the chirp was
    # DISTORTED, not merely attenuated, so the capture is biased toward higher
    # apparent inertia. fit_inertia drops such captures by default.
    clipped: int = 0

    @property
    def dt(self) -> float:
        """Median sample interval. Median, not mean: a single scheduling hiccup
        would drag the mean and silently bias every derivative taken from it."""
        return float(np.median(np.diff(self.t))) if len(self.t) > 1 else float("nan")


def load_dynamic(path: str) -> list[Capture]:
    """Group dynamic-excitation rows into one Capture per (pose_id, axis, unix_time)."""
    groups: dict[tuple, list[dict]] = {}
    for r in _rows(path, "dynamic_excitation"):
        groups.setdefault((int(r["pose_id"]), r["axis"], r["unix_time"]), []).append(r)

    caps = []
    for (pose_id, axis, _ts), rs in sorted(groups.items()):
        rs.sort(key=lambda r: int(r["sample_idx"]))
        c = Capture(pose_id=pose_id, axis=axis,
                    amp_n=float(rs[0]["amp_n"]), f0_hz=float(rs[0]["f0_hz"]),
                    f1_hz=float(rs[0]["f1_hz"]), dur_s=float(rs[0]["dur_s"]))
        c.t = np.array([float(r["t_s"]) for r in rs])
        c.cmd = np.array([[float(r[f"cmd_{i}"]) for i in (1, 2, 3)] for r in rs])
        c.q = np.array([[float(r[f"q_{i}"]) for i in (1, 2, 3)] for r in rs])
        c.raw_lastjt = np.array([[float(r[f"raw_lastjt_{i}"]) for i in (1, 2, 3)] for r in rs])
        c.pos_mm = np.array([[float(r[f"pos_{a}_mm"]) for a in ("x", "y", "z")] for r in rs])
        c.bias_n = np.array([float(rs[0].get(f"bias_{i}", 0.0) or 0.0) for i in (1, 2, 3)])
        c.clipped = int(float(rs[0].get("clipped", 0) or 0))
        caps.append(c)
    return caps


CAVEAT_CARTESIAN_COUPLING = """\
CAVEAT -- Cartesian excitation, not per-joint excitation.
  These tools command a CARTESIAN force, so every excitation loads all three
  joints at once through J^T. Per-joint excitation would need joint-space
  commanding, which is gated off in touch_dob.hpp. Consequences:
    * Friction estimates are biased toward whichever joint actually slipped
      first; they are NOT clean single-joint values.
    * Inertia estimates are correlated between joints. A joint that barely
      moves in a given capture has its M_n poorly determined by that capture,
      which is why per-joint condition numbers are reported alongside.
  Do not quote any per-joint number from this suite without this caveat."""
