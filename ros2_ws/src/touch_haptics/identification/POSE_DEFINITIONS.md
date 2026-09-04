# Pose definitions — static sweep for gravity map `G_n(q)`

Device: **Left**. Session started 2026-08-24.
Log: `~/dob_logs/sweep_left.csv` (one row per ramp, keyed on `pose_id`).

---

## Why these poses, and not others

Gravity torque cannot load the **waist (q1)**: it rotates about the vertical
axis, and rotation about vertical cannot change potential energy. So `G_n(q)`
is a function of the **shoulder (q2)** and **elbow (q3)** only.

The fitted basis is therefore

```
tau_g(q) = a*cos(q2) + b*sin(q2) + c*cos(q3) + d*sin(q3)
```

Four free coefficients per joint. If q2 and q3 stay in a narrow band those four
terms become nearly collinear: the fit still matches the sampled poses with a
small residual, and then extrapolates nonsense at poses the observer actually
runs at. This was reproduced deliberately on synthetic data — a 7-term basis on
10 clustered poses reached design condition 4.9e4 with coefficients of +/-0.55
fitting data whose true coefficients were ~0.02, at a tiny residual throughout.

Hence the grid below: q2 and q3 are varied **independently** so their separate
contributions can be told apart.

The waist may be left wherever is comfortable for each pose. It is not
constrained and does not affect the gravity fit.

## Reachable range on this arm

Measured during the FK-residual run (8 poses, same device, 2026-08-24):

| Joint | Observed range |
|---|---|
| q1 waist | -0.565 .. +0.849 rad |
| q2 shoulder | -0.126 .. +0.761 rad |
| q3 elbow | +0.495 .. +2.300 rad |

The targets below sit inside ranges this arm is known to reach. They are not
hypothetical.

---

## The 12 poses

Poses 1-9 form a 3x3 grid in (q2, q3). Poses 10-12 extend past the grid so the
fit is pinned at the edges rather than extrapolating there.

| # | q2 target | q3 target | Physical description (Turkish) | Physical description (English) |
|---|---|---|---|---|
| 1 | ~0.00 | ~0.6 | Kol alcakta, dirsek toplanmis — uc govdeye yakin | Arm low, elbow tucked — tip close to base |
| 2 | ~0.00 | ~1.2 | Kol alcakta, dirsek yari acik | Arm low, elbow half open |
| 3 | ~0.00 | ~1.9 | Kol alcakta, dirsek iyice acik — alcak ve uzak | Arm low, elbow well open — low and far |
| 4 | ~0.35 | ~0.6 | Orta yukseklik, dirsek toplanmis | Mid height, elbow tucked |
| 5 | ~0.35 | ~1.2 | Notr / merkez poz | Neutral / centre pose |
| 6 | ~0.35 | ~1.9 | Orta yukseklik, ileri uzanmis | Mid height, reaching forward |
| 7 | ~0.70 | ~0.6 | Kol yukarida, dirsek toplanmis | Arm high, elbow tucked |
| 8 | ~0.70 | ~1.2 | Kol yukarida, dirsek yari acik | Arm high, elbow half open |
| 9 | ~0.70 | ~1.9 | Kol yukarida ve uzak | Arm high and far |
| 10 | ~-0.10 | ~1.0 | Omuz notrun altinda | Shoulder below neutral |
| 11 | ~0.55 | ~2.2 | Dirsek uc bolgede (cos q3 isaret degistirir) | Elbow in the far region (cos q3 changes sign) |
| 12 | ~0.20 | ~0.5 | Dirsek en kapali konumda | Elbow at its most closed |

**Pose 11 is load-bearing for the fit.** Past q3 = pi/2 the term `cos(q3)`
changes sign, so its coefficient is genuinely tested there rather than being
inferred from one side only.

---

## What happens at each pose

Six force ramps, one per Cartesian direction: `+X -X +Y -Y +Z -Z`.

Each ramp raises commanded force from 0 to a cap of **0.30 N** in 0.004 N steps
(~1.5 s to cap) and records the force at which the arm first moves
(**breakaway**, 0.015 rad of joint deviation).

The +/- pairing is what separates the two unknowns:

```
tau_plus  =  c - g          (friction resists the + direction)
tau_minus = -c - g          (friction resists the - direction)
  =>  g = -(tau_plus + tau_minus) / 2     gravity torque
      c =  (tau_plus - tau_minus) / 2     Coulomb magnitude
```

Torque is computed as `tau = J^T(q) * F`, with `F` in newtons and exactly known,
so **every figure is in real N*m** with no dependence on the uncalibrated
`HD_LAST_JOINT_TORQUE` scale factor. `J(q)` was confirmed against this device by
`hdapi_fk_residual` on 2026-08-24: RMS 1.674 mm over a 407 mm span, PASS, with
the nearest alternative convention 35x worse.

### Known limitation of the +/- decomposition

For a single axis pair, `tau_plus = J^T u * F_plus` and
`tau_minus = -J^T u * F_minus`, so both the recovered `g` and `c` from that pair
are **parallel to `J^T u`**. One axis pair constrains gravity only along its own
direction. The three pairs therefore give three different 1-D projections, not
three independent estimates of one free 3-vector. The "spread across axis pairs"
printed by the tool is a useful smell test but is comparing projections onto
different directions — treat a small spread as necessary, not sufficient.

---

## Operating rules

1. **Do not grip the stylus during a ramp.** Position it, release, then let the
   ramp run. A hand's impedance is far larger than the arm's; gripping measures
   the operator, not the device.
2. **Restore the same pose for all six directions.** The +/- pairing assumes
   both members share a configuration. The tool warns and drops the pair if the
   pose drifted more than 0.05 rad between them.
3. **The pose must hold itself.** If the arm sags when released there is no
   static equilibrium to break and the decomposition is meaningless. The tool
   checks for 800 ms and skips the direction if drift exceeds 0.010 rad.
4. **Ctrl-C is safe at any moment.** Force is zeroed inside the servo loop
   within one frame. Measured on a stub harness: zero at +0.644 ms after signal
   delivery, with exactly one in-flight non-zero write.
5. **"No breakaway" is data, not failure.** A direction that does not move by
   the 0.30 N cap is recorded with `broke=0` and excluded from the pair.

---

## Log format

`~/dob_logs/sweep_left.csv`, one row per ramp attempt:

| Column | Meaning |
|---|---|
| `tool`, `unix_time` | provenance |
| `pose_id` | 1-12, matching the table above |
| `dir` | `+X -X +Y -Y +Z -Z` |
| `broke` | 1 if breakaway occurred, 0 if the cap was reached |
| `break_force_n` | commanded force at breakaway, N |
| `q0_1..3` | joint angles of the held pose, rad |
| `qbreak_1..3` | joint angles at breakaway, rad |
| `tau_nm_1..3` | `J^T(q0) * F` at breakaway, **real N*m** |
| `raw_lastjt_1..3` | `HD_LAST_JOINT_TORQUE`, uncalibrated units, cross-check only |
| `slipped_joint` | which joint moved most |
| `max_dq_rad` | how far it moved |

---

## Achieved angles

Filled in as the session proceeds. The targets above are intent; these are what
the device actually reported, and they are what the fit uses.

| # | q1 achieved | q2 achieved | q3 achieved | Valid pairs | Notes |
|---|---|---|---|---|---|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |
| 4 | | | | | |
| 5 | | | | | |
| 6 | | | | | |
| 7 | | | | | |
| 8 | | | | | |
| 9 | | | | | |
| 10 | | | | | |
| 11 | | | | | |
| 12 | | | | | |
