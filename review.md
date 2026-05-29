# Bug Review — `cp-integration` branch

Branch reviewed: `cp-integration`
Date: 2026-05-29
Imported: `cp_controller.py` from `integration-PID`; `--no-mpc` flag added to `simulation.py`.
Papers referenced:
- **[Morisawa 2012]** "Balance Control based on Capture Point Error Compensation for Biped Walking on Uneven Terrain", Morisawa et al., IEEE-RAS Humanoids 2012. (`reference/Balance_control.pdf`)
- **[Scianca 2020]** "MPC for Humanoid Gait Generation: Stability and Feasibility", Scianca et al., IEEE T-RO 2020. (`reference/ismpc.pdf`)

Legend: ✅ correct / fixed · ❌ false positive.

**Update:** all medium-to-critical bugs (1–6) have now been fixed in this branch. Each section
below is marked ✅ fixed with the change applied.

> This branch's IS-MPC path is the **integration** variant: it carries an internal CP-error
> integrator (`xi_error_int`, gain `k_i`), and propagates an internal LIP state `x_mpc` rather
> than feeding raw sensor state back as "desired". The imported `CPController` is the
> standalone integration-PID balance controller (Morisawa CPI, eq. 21–22). Both `solve()`
> return a 4-tuple `(lip_state, contact, p_cmd, xi_error_int)`, so `customPreStep` drives either
> with no unpacking change.

---

## Why `--no-mpc` runs faster

Same cause as on `cp-feedback`: purely computational. The IS-MPC solves a CasADi/OSQP QP
(N=100) every 10 ms step; at the 10× real-time target each step has ≈1 ms budget, so the solver
overruns and the sim drops below target speed. The `CPController` is sub-millisecond numpy and
runs at full 10×. Not a gait-speed difference (both use `ss=30, ds=10`).

---

## Architectural finding (makes Bugs 2 & 3 critical)

`inverse_dynamics.get_joint_torques` ([inverse_dynamics.py:62-84](inverse_dynamics.py#L62-L84))
reads **only** `desired['com']` (pos/vel/acc), feet, torso, base. It **never reads
`desired['zmp']`**. So the commanded ZMP `p_cmd` reaches the robot through exactly one channel:
the COM acceleration feedforward `desired['com']['acc'] = η²(x_c − p_cmd)`.
- Wrong gains (Bug 2) → mis-scaled correction.
- Acceleration built from `p_ref` instead of `p_cmd` (Bug 3) → correction discarded entirely;
  the controller runs open-loop.

---

## What is correct ✅

- **IS-MPC feedback gains.** [simulation.py](simulation.py) sets `k_1 = α/η − 1, k_2 = 0,
  k_i = 0`. On the lagless velocity-input plant ([Scianca 2020] eq. 4) this places the CP-error
  pole exactly at α. With α = −3: `λ = η(1 + k_1) = −3`. Verified. This is the correct
  lagless tuning (matches the fix applied on `cp-feedback`).
- **IS-MPC internal state propagation.** `solve` integrates `x_mpc` forward with the LIP
  dynamics and uses it for `lip_state`, instead of returning raw sensor state as "desired".
  This is cleaner than the `cp-feedback` MPC and keeps the QP initial condition consistent.
- **Stability constraint.** Periodic-tail CP periodicity `ẋ_c + η·x_c` equal at horizon start
  and end ([ismpc.py](ismpc.py)) — matches [Scianca 2020] eq. (23), capturability/periodic tail.

---

## Bug 1 — ZMP velocity always zero in CPController ✅ fixed

**File:** [`cp_controller.py`](cp_controller.py) · **Severity:** High

```python
self.lip_state['zmp']['pos'] = p_cmd
self.lip_state['zmp']['vel'] = (p_cmd - self.lip_state['zmp']['pos']) / self.delta   # = 0
```

Position was overwritten with `p_cmd` before the velocity diff, so velocity was always zero. The
Kalman filter `predict` then integrated with `u = 0` every step under `--no-mpc`, degrading the
state estimate the controller feeds back on.

**Fix applied:** cache the old position first —
`prev_zmp = zmp.copy(); zmp = p_cmd; vel = (p_cmd − prev_zmp)/δt`.

---

## Bug 2 — Lag-model gains applied to a lagless plant ✅ fixed

**File:** [`cp_controller.py`](cp_controller.py) · **Severity:** Critical
**Reference:** [Morisawa 2012] eq. (2)–(3), (20)–(22); [Scianca 2020] eq. (4)

The CPController computes its gains with Morisawa's **first-order-lag** model (eq. 22, denominator
`g_p`), but the plant has **no lag**: the commanded ZMP is realized within one control step
([simulation.py](simulation.py): `desired['zmp']['vel'] = (p_cmd − zmp)/δt`), i.e. `g_p → ∞`.

```python
self.k_1 = -((alpha - eta)*(beta - eta))/(eta*g_p)   # γ=0 lag form
self.k_2 = -(alpha + beta - eta + g_p)/g_p            # γ=0 lag form
self.k_I = (alpha*beta*gamma)/(eta*g_p)               # γ≠0 form  ← inconsistent with k_1,k_2
```

Two problems: (a) lag gains on a lagless plant, and (b) `k_1, k_2` use the γ=0 form while `k_I`
uses γ. With α=−3, β=−8, γ=−1, g_p=20, η=3.6912 the realized CP-error pole on the lagless plant is

```
λ_eff = η(1 + k_1/(1+k_2)) = −1.63    (designed α = −3)
```

≈46 % weaker than intended (numerically verified). Combined with Bug 3 the correction is in fact
zero; even once Bug 3 is fixed, this under-placed pole leaves the robot sluggish against
accumulated error.

**Correct gains for the lagless plant.** The CP-error + integral dynamics is 2nd order; place
the two poles at {α, γ}:

```
k_1 = (α + γ)/η − 1        k_2 = 0        k_I = −αγ/η
```

With α=−3, γ=−1: `k_1=−2.0837, k_2=0, k_I=−0.8127` → poles exactly at {−3, −1} (verified). This
is the same correction applied to `cp-feedback`; `beta` and `g_p` then drop out.

**Fix applied:** `k_1 = (α+γ)/η − 1, k_2 = 0, k_I = −αγ/η` in
[cp_controller.py](cp_controller.py) (`beta`/`g_p` no longer used).

---

## Bug 3 — COM-acceleration feedforward uses p_ref instead of p_cmd ✅ fixed

**File:** [`cp_controller.py`](cp_controller.py) · **Severity:** Critical
**Reference:** [Morisawa 2012] Fig. 6, eq. (1)

```python
com_acc_ref[0:2] = (self.eta**2) * (com_pos_ref_new[0:2] - p_ref[0:2])   # p_ref, not p_cmd
```

Because inverse dynamics ignores `desired['zmp']` (see architectural finding), this acceleration
is the only path from controller to robot. Using `p_ref` discarded the feedback correction
`p_cmd − p_ref`: the robot tracked the open-loop reference and never reacted to disturbances. Also
inconsistent with `desired['zmp']['pos'] = p_cmd`.

**Fix applied:** `com_acc_ref[0:2] = η² · (com_pos_ref_new[0:2] − p_cmd[0:2])`.

---

## Bug 4 — ZMP command clipped around p_ref, not the support polygon ✅ fixed

**File:** [`cp_controller.py`](cp_controller.py) · **Severity:** Medium
**Reference:** [Morisawa 2012] Sec. III-B

```python
limit = 0.06
p_cmd[0] = np.clip(p_cmd[0], p_ref[0]-limit, p_ref[0]+limit)
p_cmd[1] = np.clip(p_cmd[1], p_ref[1]-limit, p_ref[1]+limit)
```

The ±6 cm box is centered on `p_ref`. In double support `p_ref` is the smoothstep interpolation
between the feet, so the box sits between the feet instead of spanning the true (wider)
double-support polygon, throttling the correction when it is most needed.

**Fix applied:** clip to the actual support region — single support `p_current ± foot_size/2`;
double support the bounding box of both feet plus a `foot_size/2` margin.

---

## Bug 5 — Integrator not reset on contact loss under `--no-mpc` ✅ fixed (cp-integration-specific)

**Files:** [`simulation.py`](simulation.py) `retrieve_state`, [`cp_controller.py`](cp_controller.py)
**Severity:** Medium

On contact loss `retrieve_state` runs:

```python
if hasattr(self, 'mpc'):
    self.mpc.xi_error_int = np.zeros(3)
```

This resets the **IS-MPC** integrator attribute `xi_error_int`. But under `--no-mpc` the
controller is the `CPController`, whose integrator is `cp_error_integral` (a 2-vector). The line
just creates a stray unused `xi_error_int` attribute and **does not reset** the real integral, so
CP-error integral windup persists across a contact loss → larger transient ZMP command on
re-contact ([Morisawa 2012] Sec. III-A warns explicitly about integral wind-up).

**Fix applied:** on contact loss, reset whichever integrator attribute the active controller
exposes — `xi_error_int` (MPC) and/or `cp_error_integral` (CPController).

---

## Bug 6 — ZMP set to origin (not previous value) on contact loss ✅ fixed (both modes)

**File:** [`simulation.py`](simulation.py) `retrieve_state` · **Severity:** Low–Medium

```python
if force[2] <= 0.1:
    zmp = np.array([0., 0., 0.]) # FIXME: this should return previous measurement
```

Already flagged in-code. On a (brief) loss of contact the measured ZMP jumped to the world
origin instead of holding the last value, injecting a large spurious ZMP error into both the KF
and the CP feedback. Affected MPC and CPController alike.

**Fix applied:** track `self.prev_zmp` each step and reuse it on contact loss
(`zmp = getattr(self, 'prev_zmp', np.zeros(3)).copy()`).

---

## False positive ❌

- **IS-MPC moving-constraint start at the foot midpoint** ([ismpc.py](ismpc.py)
  `generate_moving_constraint`, `fs_current_pos = ... if j > 0 else [mc_x[0], mc_y[0]]`). Using
  the midpoint for the j=0→1 transition is **correct**: at t=0 the ZMP genuinely sits between
  the feet, not at the laterally-offset virtual footstep `plan[0]['pos']`.

---

## Summary

| # | File | Severity | Status | Description |
|---|------|----------|--------|-------------|
| — | `simulation.py` (MPC) | — | ✅ | Lagless IS-MPC gains place CP pole exactly at α=−3 |
| 1 | `cp_controller.py` | High | ✅ fixed | ZMP velocity always zero (position overwritten before diff) |
| 2 | `cp_controller.py` | Critical | ✅ fixed | Lag gains on lagless plant → CP pole −1.63 vs α=−3. New: `k_1=(α+γ)/η−1, k_2=0, k_I=−αγ/η` → poles {−3,−1} |
| 3 | `cp_controller.py` | Critical | ✅ fixed | COM-acc feedforward now uses p_cmd (only feedback path to robot) |
| 4 | `cp_controller.py` | Medium | ✅ fixed | ZMP now clipped to the support polygon |
| 5 | `simulation.py` / `cp_controller.py` | Medium | ✅ fixed | Contact-loss reset now zeros both `xi_error_int` and `cp_error_integral` |
| 6 | `simulation.py` | Low–Med | ✅ fixed | ZMP holds `prev_zmp` on contact loss instead of origin |
| — | `ismpc.py` moving constraint | — | ❌ | Midpoint start is correct, not a bug |

Bugs 1–3 mirrored the `cp-feedback` findings and together caused the `--no-mpc` fall: the
feedback was discarded (Bug 3), the gains were under-placed (Bug 2, −1.63 vs −3), and the KF saw
zero ZMP velocity (Bug 1). Bug 5 was specific to this branch's shared-integrator design. **All
six are now fixed**: the CPController closes the loop on the lagless plant with poles placed at
{α, γ} = {−3, −1} (verified). Test run recommended: `python simulation.py --no-mpc`.
