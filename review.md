# Bug Review — `cp-feedback` branch

Branch reviewed: `cp-feedback`
Date: 2026-05-29
Papers referenced:
- **[Morisawa 2012]** "Balance Control based on Capture Point Error Compensation for Biped Walking on Uneven Terrain", Morisawa et al., IEEE-RAS Humanoids 2012. (`reference/Balance_control.pdf`)
- **[Scianca 2020]** "MPC for Humanoid Gait Generation: Stability and Feasibility", Scianca et al., IEEE T-RO 2020. (`reference/ismpc.pdf`)

Legend: ✅ fixed in this branch · ⚠️ left in place on purpose (see note) · ❌ false positive.

---

## Why `--no-mpc` runs faster

This is **purely a computational-cost difference, not a gait-speed difference**. Both
modes use identical step timing (`ss_duration=30`, `ds_duration=10` → 0.4 s/cycle).

- **MPC**: solves a CasADi/OSQP convex QP over a horizon of N=100 every simulation step.
  Each sim step has only 1 ms of wall-clock budget at the 10× real-time target; the QP
  solve overruns it, so the simulation falls below target speed.
- **CPController**: a handful of numpy operations (sub-millisecond), easily within budget,
  so it runs at the intended 10×.

The mid-walk **fall** with `--no-mpc` is unrelated to speed — it is caused by Bugs 1–3 below.

---

## Key architectural finding (why Bugs 2 and 3 are critical)

`inverse_dynamics.get_joint_torques` ([inverse_dynamics.py:62-84](inverse_dynamics.py#L62-L84))
consumes **only** `desired['com']` (pos/vel/acc), the feet, torso and base. **It never reads
`desired['zmp']`.** Therefore the commanded ZMP `p_cmd` reaches the robot through exactly one
channel: the COM acceleration feedforward `desired['com']['acc'] = η²(x_c − p_cmd)`.

Consequences:
- If the feedback gains are wrong (Bug 2), the correction is mis-scaled.
- If that acceleration is computed from `p_ref` instead of `p_cmd` (Bug 3), **the feedback
  correction never reaches the plant at all** — the controller runs fully open-loop.

These two together fully explain the fall; the third contributor is the always-zero ZMP
velocity fed to the Kalman filter (Bug 1).

---

## Bug 1 — ZMP velocity always zero in CPController ✅

**File:** [`cp_controller.py`](cp_controller.py) (was lines 104-105) · **Severity:** High

```python
self.lip_state['zmp']['pos'] = p_cmd                                                 # overwrite
self.lip_state['zmp']['vel'] = (p_cmd - self.lip_state['zmp']['pos']) / self.delta    # = 0 !
```

Position is overwritten with `p_cmd` *before* the velocity diff, so the velocity is always the
zero vector. The Kalman filter `predict` step then integrates with `u = [0,0,0]` every step,
degrading the state estimate the controller feeds back on.

**Fix applied:** save the previous position before overwriting:

```python
prev_zmp = self.lip_state['zmp']['pos'].copy()
self.lip_state['zmp']['pos'] = p_cmd
self.lip_state['zmp']['vel'] = (p_cmd - prev_zmp) / self.delta
```

---

## Bug 2 — Plant-model mismatch: Morisawa lag gains applied to a lagless ZMP plant ✅

**File:** [`cp_controller.py`](cp_controller.py) (was lines 15-17) · **Severity:** Critical
**Reference:** [Morisawa 2012] eq. (2)–(3), (20)–(22); [Scianca 2020] eq. (4)

> This is the same root cause as the "plant-model mismatch" bug previously noted for the
> IS-MPC. That one was already fixed for the MPC path (`k_1 = α/η − 1, k_2 = 0` in
> [simulation.py:47](simulation.py#L47), which places the CP pole exactly at α on the lagless
> plant). The **same bug was still live in the CPController**, which was using the lag-model
> gains.

**What Morisawa assumes.** The balance controller is derived for a plant in which the ZMP has a
**first-order lag** driven by a desired-ZMP command (eq. 2): `ṗ = −g_p·p + g_p·p^d`. The three
CPI gains are obtained by placing the poles {α, β, γ} of that 3rd-order error system (eq. 20),
giving eq. (22):

```
k_1^cpi = −[αβ + βγ + γα − ω(α+β+γ−ω)] / (ω·g_p)
k_2^cpi = −(α + β + γ + g_p − ω) / g_p
k_I^cpi =  αβγ / (ω·g_p)
```

**What the plant actually is.** The ZMP command is realized within one control step —
[simulation.py:193](simulation.py#L193) sets `desired['zmp']['vel'] = (p_cmd − zmp)/dt`, i.e.
`p_{k+1} ≈ p_cmd`. This is the lagless case `g_p → ∞`, identical to the IS-MPC velocity-input
model ([Scianca 2020] eq. 4). The lag state, and hence the pole β, no longer exist.

**The old code mixed both worlds** (and even used the γ=0 form for k_1, k_2 while using γ=−1
for k_I):

```python
self.k_1 = -((alpha - eta)*(beta - eta))/(eta*g_p)   # γ=0 lag form
self.k_2 = -(alpha + beta - eta + g_p)/g_p            # γ=0 lag form
self.k_I = (alpha*beta*gamma)/(eta*g_p)               # γ≠0 form
```

With α=−1, β=−8, g_p=20, γ=−1, η=3.6912 these give k_1=−0.743, k_2=−0.365, k_I=−0.108. On the
**lagless** plant the realized CP-error pole is

```
λ_eff = η·(1 + k_1/(1+k_2)) = 3.6912·(1 − 0.743/0.635) = −0.63    (designed: −1)
```

so the correction is ≈37 % weaker than intended (numerically verified). Combined with Bug 3 the
correction was actually zero; once Bug 3 is fixed, this weak/mis-placed pole still leaves the
robot under-damped against accumulated error → fall.

**Correct gains for the lagless plant.** The CP-error + integral dynamics is 2nd order:

```
[ė_ξ]   [η(1+k_1)   η·k_I] [e_ξ]                char(λ) = λ² − η(1+k_1)λ − η·k_I
[σ̇  ] = [   1         0  ] [ σ ] ,   σ=∫e_ξ
```

Placing the two poles at {α, γ} (matching λ² − (α+γ)λ + αγ):

```
k_1 = (α + γ)/η − 1        k_2 = 0   (no ZMP-lag state to feed back)        k_I = −αγ/η
```

**Fix applied** (with α=−1, γ=−1 → k_1=−1.5418, k_2=0, k_I=−0.2709, giving a double pole at −1,
numerically verified). `beta` and `g_p` are no longer used and were removed from
[simulation.py](simulation.py). Dropping the integrator (γ=0) recovers the IS-MPC form
`k_1 = α/η − 1, k_2 = 0`.

---

## Bug 3 — COM-acceleration feedforward used p_ref instead of p_cmd ✅

**File:** [`cp_controller.py`](cp_controller.py) (was line 101) · **Severity:** Critical
**Reference:** [Morisawa 2012] Fig. 6 (balance output → force/torque → COM); eq. (1)

```python
com_acc_ref[0:2] = (self.eta**2) * (com_pos_ref_new[0:2] - p_ref[0:2])   # p_ref, not p_cmd!
```

Because inverse dynamics ignores `desired['zmp']` (see *Key architectural finding* above), this
acceleration is the **only** path from the controller to the robot. Computing it from `p_ref`
(the open-loop nominal ZMP) discards the entire feedback correction `p_cmd − p_ref`: the robot
tracks the open-loop reference and never reacts to disturbances. It also makes the feedforward
inconsistent with `desired['zmp']['pos'] = p_cmd`.

**Fix applied:**

```python
com_acc_ref[0:2] = (self.eta**2) * (com_pos_ref_new[0:2] - p_cmd[0:2])
```

(The COM position/velocity references stay open-loop, as in Morisawa: the balance controller
only modifies the ZMP, and the corrected ZMP enters the dynamics through the acceleration term.)

---

## Bug 4 — ZMP command clipped around p_ref instead of the support polygon ✅

**File:** [`cp_controller.py`](cp_controller.py) (was lines 83-86) · **Severity:** Medium
**Reference:** [Morisawa 2012] Sec. III-B — "the ZMP should be limited within the support polygon"

```python
limit = 0.06
p_cmd[0] = np.clip(p_cmd[0], p_ref[0] - limit, p_ref[0] + limit)
p_cmd[1] = np.clip(p_cmd[1], p_ref[1] - limit, p_ref[1] + limit)
```

The ±6 cm box was centered on `p_ref`. During double support `p_ref` is the smoothstep
interpolation between the two feet, so the box sits between the feet rather than spanning the
true (much wider) double-support polygon, artificially throttling the correction exactly when
it is needed most. During single support it is a ±6 cm box, slightly looser than the foot.

**Fix applied:** clip against the actual support region — single support: `p_current ±
foot_size/2`; double support: bounding box of the two feet plus a `foot_size/2` margin.

---

## Bug 8 — Kalman filter covariance update not in Joseph form ✅

**File:** [`filter.py`](filter.py) (was line 30) · **Severity:** Low (affects both modes)

```python
self.P = (I - K @ self.H) @ self.P
```

The simplified `(I−KH)P` form loses symmetry/positive-definiteness under finite precision and
model mismatch, and can let `P` drift to negative variances over a long run.

**Fix applied (Joseph form):**

```python
IKH = I - K @ self.H
self.P = IKH @ self.P @ IKH.T + K @ self.R @ K.T
```

---

## Bug 9 — IS-MPC: `self.eta` assigned twice ✅

**File:** [`ismpc.py`](ismpc.py) (was lines 11 & 19) · **Severity:** Low (cosmetic)

The redundant second `self.eta = params['eta']` was removed.

---

## Bug 6 — Footstep planner freezes the first two steps ⚠️ (left as-is)

**File:** [`footstep_planner.py:29`](footstep_planner.py#L29) · **Severity:** Medium / by design?

```python
if j > 1:
    unicycle_theta += vref[j][2] * params['world_time_step']
    ...
    unicycle_pos += R @ vref[j][:2] * params['world_time_step']
```

The unicycle only advances for `j ≥ 2`: step 0 is the dummy (intentionally frozen) and step 1
(the first real step) is also frozen at the initial pose. This appears **intentional** — it lets
the robot settle before commanded motion begins — and the IS-MPC walks with it. Changing it to
`j > 0` would alter the gait for *both* controllers and risk breaking the working MPC, so it was
**left unchanged**. Flagged for the author to confirm the freeze is deliberate.

---

## Bug 7 — Swing trajectory starts from planned (not measured) foot pose ⚠️ (left as-is)

**File:** [`foot_trajectory_generator.py:88`](foot_trajectory_generator.py#L88) · **Severity:** Medium

```python
start_pos = self.plan[step_index - 1]['pos']   # planned landing pose, not measured
```

The cubic swing starts from the *planned* previous landing pose. Small landing errors create a
position/velocity discontinuity at the start of each swing, exciting joint oscillation. Fixing
this properly requires feeding the measured foot pose into the generator — an architectural
change affecting both controllers and the working MPC — so it was **left as a recommendation**
rather than implemented here.

---

## Bug 5 — IS-MPC moving-constraint start position ❌ (false positive)

**File:** [`ismpc.py:150-160`](ismpc.py#L150-L160)

Earlier flagged as using the foot midpoint `mc_x[0]` for the j=0→j=1 transition instead of
`plan[0]['pos']`. On reflection this is **correct**: at t=0 the robot is in double support with
the ZMP genuinely at the midpoint of the feet, so the moving constraint should start there, not
at the laterally-offset virtual footstep `plan[0]['pos']`. No change made.

---

## Summary

| # | File | Severity | Status | Description |
|---|------|----------|--------|-------------|
| 1 | `cp_controller.py` | High | ✅ fixed | ZMP velocity always zero (position overwritten before diff) |
| 2 | `cp_controller.py` | **Critical** | ✅ fixed | Lag-model gains on a lagless plant → CP pole at −0.63 not −1; new gains `k_1=(α+γ)/η−1, k_2=0, k_I=−αγ/η` |
| 3 | `cp_controller.py` | **Critical** | ✅ fixed | COM-acc feedforward used p_ref; the only feedback path to the robot, so correction was discarded |
| 4 | `cp_controller.py` | Medium | ✅ fixed | ZMP clipped around p_ref, not the support polygon |
| 8 | `filter.py` | Low | ✅ fixed | KF covariance update now Joseph form |
| 9 | `ismpc.py` | Low | ✅ fixed | Duplicate `self.eta` removed |
| 6 | `footstep_planner.py` | Medium | ⚠️ left | First two steps frozen — likely by design; would affect MPC |
| 7 | `foot_trajectory_generator.py` | Medium | ⚠️ left | Swing from planned not measured pose — architectural change |
| 5 | `ismpc.py` | — | ❌ n/a | Moving-constraint start at midpoint is actually correct |

Bugs 1–3 together explain the mid-walk fall under `--no-mpc`: the feedback correction was
discarded entirely (Bug 3), the gains were mis-placed (Bug 2, −0.63 vs −1), and the Kalman
filter saw zero ZMP velocity (Bug 1). With all three fixed the CPController closes the loop on
the lagless plant with a double CP pole at the designed value α = −1.
