# DOB parameter identification — Python suite

Fits the three things the disturbance observer still needs — the gravity map
`G_n(q)`, the friction floor, and the nominal inertia `M_n` — from data logged
off **this** device by the C++ tools in `../tools/`.

No MATLAB. No `.mat` files. Nothing from the legacy projects (`SetUp.m`,
`initiatesystems.m`, `omni10.mat`, `dob.mat`) is read as input by anything here.
They are named only to record that they are deliberately excluded: their
provenance was never established and one of them contained a non-physical
inertia (`I2 = 0`).

---

## Units — the short version

Everything this suite reports is in **real SI**: `M_n` in kg·m², gravity and
friction in N·m.

That is possible **without** running `hdapi_torque_calib.cpp`, because joint
torque is computed as `τ = Jᵀ(q) · F_cmd` where `F_cmd` is a Cartesian force we
chose, in newtons, exactly known. The uncalibrated `HD_LAST_JOINT_TORQUE` scale
factor never enters the fit. It is logged anyway, purely as an independent
cross-check on that calibration when you do get to it.

**The dependency this does have is `J(q)`.** Every N·m figure assumes both the
Jacobian algebra and that HDAPI's joint-angle convention matches the one the
Jacobian was written in. `hdapi_fk_residual` is the test for exactly that. Get a
PASS from it first. If it fails, everything here is wrong by an unknown
rotation and no amount of fitting repairs it.

---

## Runbook — at the hardware, operator present

Operator must be physically at the device for every step. One OpenHaptics
process owns a device at a time, so run these one at a time.

### Step 0 — prerequisite, passive

```bash
cd ros2_ws/src/touch_haptics/tools
./hdapi_fk_residual "Left"
```

Six poses, spread widely; poses 5 and 6 approach the singular configurations.
**Requires a PASS before continuing.** Never enables force output.

### Step 1 — static sweep: gravity + friction

```bash
./hdapi_static_sweep "Left" --log ~/dob_logs/sweep_left.csv
```

Per pose you are prompted six times (±X, ±Y, ±Z). Restore the same pose each
time — the ± pairing assumes both members share a configuration, and the tool
warns if it drifted.

**Do not grip the stylus during a ramp.** Position it, release, then let it ramp.
A hand adds its own resistance and you would be measuring your arm.

Aim for **10+ poses**, spread as widely in q2/q3 as the workspace allows. The
gravity fit is a 4-coefficient basis; a narrow pose set makes it collinear and
it will extrapolate badly. Ramp cap 0.30 N. Ctrl-C is safe at any time.

### Step 2 — dynamic excitation: inertia

```bash
./hdapi_dynamic_excitation "Left" --log ~/dob_logs/dyn_left.csv
# defaults: 0.5->6 Hz chirp, 8 s, 0.25 N peak, axes xyz
```

Per pose, each of X/Y/Z is chirped in turn. Same rule: position, **release**,
then start. Use the **same poses** as Step 1 where practical, so the gravity map
is being evaluated where it was measured rather than extrapolated.

If a capture reports `barely moved`, raise `--amp` (ceiling 0.30 N) or `--dur`.

### Step 3 — fitting, away from the hardware

```bash
cd ../identification
python3 fit_gravity.py  ~/dob_logs/sweep_left.csv --out gravity_left.json
python3 fit_friction.py ~/dob_logs/sweep_left.csv --out friction_left.json
python3 fit_inertia.py  ~/dob_logs/dyn_left.csv \
        --gravity gravity_left.json --friction friction_left.json \
        --out inertia_left.json
```

Order matters: `fit_inertia` subtracts the gravity fit before regressing, and
cross-checks its Coulomb term against the static one.

### Step 4 — optional, for the raw-unit scale factor only

```bash
./hdapi_torque_calib "Left" x --log ~/dob_logs/calib_left.csv
```

Only needed to convert `HD_LAST_JOINT_TORQUE` into N·m. **Not** a prerequisite
for any fit above.

> ⚠ `hdapi_torque_calib.cpp` currently has **no SIGINT/SIGTERM handler** while it
> commands up to 0.30 N. Ctrl-C mid-sweep can leave force latched — the same gap
> that was already closed in `hdapi_static_sweep.cpp`. Left as-is deliberately
> (this task was not to change safety behaviour). Worth fixing before use.

---

## What each script reports, and how to read it

### `fit_gravity.py`
Per-pose decomposition, then a fit over `{cos q2, sin q2, cos q3, sin q3}`.

- **`--basis minimal`** (default, 4 terms) is what the kinematics implies: with
  `y = L1·sin q2 − L2·cos q3` the two links' heights depend on q2 and q3
  separately, so there is no cross term. `--basis full` adds a constant and
  `cos/sin(q2−q3)`; on a 10-pose set that reached a design condition of ~5·10⁴
  with coefficients of ±0.55 fitting data whose true coefficients were ~0.02.
  Low residuals do **not** vindicate it — collinear terms always fit the
  sampled points and then extrapolate nonsense.
- **Watch `design cond`.** Above 1000 the script says so explicitly.
- **Physical check:** gravity cannot load the waist, which rotates about the
  vertical. If `max |g_waist|` is not ≈0, suspect the J(q) convention or a
  non-level base before believing anything else.
- **`R²` for the waist is meaningless** — the true signal there is zero, so
  you are computing R² against noise. Judge the waist by the physical check.

### `fit_friction.py`
Coulomb per joint, from the same ± breakaway pairs.

Viscous friction is **not** obtainable here and the script does not pretend
otherwise: the sweep is quasi-static, measured at first motion where q̇≈0, so a
`b·q̇` term is invisible. Viscous comes from `fit_inertia.py` instead.

Prefer the **median**; with this few pairs one bad pair moves the mean a long
way. Negative Coulomb estimates are non-physical and bound nothing.

### `fit_inertia.py`
Regresses, per joint, over every sample:

```
τ_j − g_j(q)  =  M_j·q̈_j  +  b_j·q̇_j  +  c_j·sgn(q̇_j)
```

- **q̇, q̈ come from a non-causal Savitzky–Golay filter** — a local polynomial
  fit differentiated analytically, using samples from both sides of each point.
  Zero phase lag, strong noise rejection. The real-time DOB cannot do this: at
  time *t* it does not have *t+1*, so it is stuck with a causal filter trading
  lag against noise — the same lag the earlier audit flagged as destabilising in
  the reflection damping term. Offline identification has no such constraint and
  should not inherit that compromise; using a worse derivative here purely to
  "match" the observer would inject avoidable error into the constants the
  observer depends on.
- **`M_n` is nominal, not rigid-body.** A single constant per joint, ignoring
  M(q)'s configuration dependence and all Coriolis coupling. That is deliberate
  and matches `touch_dob.hpp` — a constant nominal inertia is what keeps the
  Q-filter LTI and is the true Ohnishi form. It absorbs mean coupling over the
  poses you sampled, which is why sampling the poses the observer will actually
  run at matters.
- **Tip-inertia cross-check** against the Omni Device Guide, Appendix F
  (~45 g apparent mass at tip), via `Λ(q) = (J M⁻¹ Jᵀ)⁻¹`. A sanity check only:
  it cannot detect an error smaller than ~3×, and the spec is one tip figure
  while `M_n` is per joint.

---

## Validation status of the suite itself

The fitting code was checked against synthetic data with known ground truth
before any hardware run:

| Quantity | Result |
|---|---|
| `M_n` recovery | within **2.1 %** on all three joints |
| Coulomb `c` recovery | within **3 %** |
| Viscous `b` recovery | within **11 %** |
| Python vs C++ `J(q)`, `τ=JᵀF`, FK | **bitwise identical** |

That validates the regression, the SG derivatives, the `Jᵀ F` torque path and
the CSV round-trip. It does **not** validate any physical assumption about the
real device — notably that breakaway cleanly separates gravity from friction.

### A limitation of the ± decomposition worth knowing

For one axis pair, `τ₊ = Jᵀu·F₊` and `τ₋ = −Jᵀu·F₋`, so both the recovered
`g = −(τ₊+τ₋)/2` and `c = (τ₊−τ₋)/2` are **parallel to `Jᵀu`**. A single axis
pair therefore constrains gravity only along its own direction — the three pairs
do not give three independent estimates of the same free 3-vector, they give
three different 1-D projections. The "spread across axis pairs" that both the
C++ tool and `fit_gravity.py` print is a useful smell test, but it is comparing
projections onto different directions, not repeated measurements of one thing.
Treat a small spread as necessary, not sufficient.

---

## What this suite deliberately does not do

It writes **nothing** to `touch_dob.hpp`. `JointDobParams.m_n_kg_m2` stays at its
unset sentinel, `dob_bank_` stays inactive, and both commissioning gates stay
closed. Every script says so on exit. Moving a fitted number into the DOB is a
separate, explicit decision for the operator.
