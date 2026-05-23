# Repo Review: IS-MPC + CP-ZMP Balance

## Scope and references

This repository implements IS-MPC for humanoid gait generation (Scianca et al., 2020)
and extends it with CP-ZMP balance control from Morisawa et al. (2012).
The review focuses on the CP-ZMP feedback controller, the Kalman filter dependency,
and correctness of the IS-MPC stability constraint.

References:
- Scianca et al., "MPC for Humanoid Gait Generation: Stability and Feasibility", *T-RO*, 2020.
- Morisawa et al., "Balance Control based on Capture Point Error Compensation for Biped
  Walking on Uneven Terrain", *Humanoids*, 2012.

> **Note on branch state**: Merging code from `cp-integration` reintroduced several
> bugs that were previously marked as fixed. Findings #2 (left foot clipping),
> #3 (global `world` reference), and #6 (missing node attribute assignments) were
> re-introduced and have been fixed in this branch. Finding #4 (ZMP reset to zero on
> low force) remains — see the FIXME comment in `simulation.py:248`.

---

## Findings (ordered by severity)

---

### 1 — Plant model mismatch: Morisawa gains applied to a lagless ZMP system `[Critical]`

**What the paper assumes.**  
Morisawa's balance controller was designed for a system where the ZMP has a first-order
lag driven by a desired-ZMP command `p_x^d`:

```
ṗ_x = −g_p · p_x + g_p · p_x^d       (paper eq. 2)
```

The feedback gains k_1, k_2 are derived by pole-assignment on this 2nd-order error system
`[ė_ξ; ė_p]` whose A-matrix explicitly depends on g_p.

**What the IS-MPC model uses.**  
The IS-MPC (Scianca) uses ZMP *velocity* as the control input:

```python
# ismpc.py:26
self.A_lip = np.array([[0, 1, 0], [self.eta**2, 0, -self.eta**2], [0, 0, 0]])
self.B_lip = np.array([[0], [0], [1]])
```

The third state row `[0, 0, 0]` means `ṗ_z = 0 + u`, i.e., the ZMP integrates directly
from the velocity command — no lag.

**How the code bridges the gap.**  
After computing `p_cmd` the code converts it to a velocity:

```python
# simulation.py:182
self.desired['zmp']['vel'] = (p_cmd - self.current['zmp']['pos']) / self.params['world_time_step']
```

This effectively sets `p_{k+1} = p_cmd` in one timestep — equivalent to g_p → ∞ in
Morisawa's lag model.

**Consequence: wrong effective poles.**  
With instantaneous ZMP response the closed-loop CP error pole is:

```
λ_eff = η · (1 + k_1 / (1 + k_2))
```

With the code's values (η=3.691, k_1=−1.486, k_2=+0.269):
`λ_eff = −0.63`, not the designed α=−1.

Both the code *and* the paper's parameters give `λ_eff ≈ −0.63` in the lagless plant.
This means the CP correction is ~37 % weaker than intended.

**Correct gains for the lagless IS-MPC plant.**  
Setting k_2 = 0 (no ZMP feedback needed when ZMP is instantly reachable) and solving
for pole at α:

```
k_1 = α/η − 1 = −1/3.691 − 1 ≈ −1.271,   k_2 = 0
```

This places the exact CP pole at α=−1 without the lag.
If ZMP feedback is still desired, re-derive k_1, k_2 by treating the one-step ZMP
update as a fast lag with `g_p = 1/δ = 100 s⁻¹` and re-running the pole-assignment.

**Location:** [simulation.py:46–51](simulation.py#L46-L51), [ismpc.py:26](ismpc.py#L26)

---

### 2 — `xi_ref` is one step *ahead* of `xi_meas`: CP error is a velocity, not a position `[Critical]`

**What the paper requires.**  
Morisawa eq. 14: `Δp = −k_1·(ξ − ξ_ref) − k_2·(p − p_ref)` where `ξ_ref` is the
*desired* CP at the **current** time, computed from the reference trajectory.

**What the code does.**

```python
# ismpc.py:104–118
xi_ref = self.compute_cp(
    self.x_pred[[0, 3, 6]],   # MPC prediction at t+δ  (one step AHEAD)
    self.x_pred[[1, 4, 7]]
)
xi_meas = self.compute_cp(
    current['com']['pos'],     # measured state at t    (NOW)
    current['com']['vel']
)
p_cmd = p_ref - self.k_1 * (xi_meas - xi_ref) - self.k_2 * (p_meas - p_ref)
```

`xi_ref` is the CP the MPC *predicts* the robot will be at in the **next** step, while
`xi_meas` is the actual CP **now**. For a well-tracked robot:

```
xi_meas(t) ≈ xi_ref(t−1)   →   xi_meas(t) − xi_ref(t) ≈ −δ · dξ/dt
```

The "CP error" is therefore proportional to **minus the CP velocity**, not to the
position tracking error. The controller accidentally implements derivative (D) feedback
instead of proportional (P) feedback.

**Fix.**  
Store `x_pred` at the end of each `solve()` call and use it as `xi_ref` the *next*
time `solve()` is called:

```python
# first call: self._xi_ref_prev = None
# at end of solve():
self._xi_ref_prev = self.x_pred.copy()

# at start of solve(), before computing p_cmd:
if self._xi_ref_prev is not None:
    xi_ref = self.compute_cp(
        self._xi_ref_prev[[0, 3, 6]],
        self._xi_ref_prev[[1, 4, 7]]
    )
else:
    xi_ref = xi_meas   # no correction on first step
```

**Location:** [ismpc.py:107–124](ismpc.py#L107-L124)

---

### 3 — IS-MPC stability constraint is CP-minus-ZMP periodic, not pure CP periodic `[Major]`

**IS-MPC paper (eq. 23, periodic tail terminal constraint):**

```
x_u(t_k + T_c) = x_u(t_k)     where x_u = x_c + ẋ_c / η
```

i.e., the CP (divergent component) is periodic over the horizon.

**Code (ismpc.py:72–77):**

```python
self.opt.subject_to(
    self.X[1, 0]     + self.eta * (self.X[0, 0]     - self.X[2, 0])     ==
    self.X[1, self.N] + self.eta * (self.X[0, self.N] - self.X[2, self.N])
)
```

Algebraically this enforces `η·(x_u(0) − p_z(0)) = η·(x_u(N) − p_z(N))`, i.e.,
**(CP − ZMP) is periodic**, not CP alone.

During walking the ZMP reference shifts forward between feet, so `p_z(0) ≠ p_z(N)`.
This allows the CP to drift by the same amount as the ZMP, weakening the stability
guarantee proven in the IS-MPC paper.

**Fix.**  Remove the `−X[2]` terms:

```python
self.opt.subject_to(
    self.X[1, 0]      + self.eta * self.X[0, 0]      ==
    self.X[1, self.N] + self.eta * self.X[0, self.N]
)
# same change for Y and Z axes
```

**Location:** [ismpc.py:72–77](ismpc.py#L72-L77)

---

### 4 — ZMP normalization bug: per-contact threshold not applied to the denominator `[Major]`

**The code (simulation.py:241–251):**

```python
# total force: ALL contacts, no threshold
force = np.zeros(3)
for contact in self.world.getLastCollisionResult().getContacts():
    force += contact.force                              # ← includes contacts with Fz ≤ 0.1

# ZMP numerator: contacts with Fz > 0.1 only
for contact in self.world.getLastCollisionResult().getContacts():
    if contact.force[2] <= 0.1: continue               # ← skips small contacts
    zmp[0] += contact.point[0] * contact.force[2] / force[2] + ...
```

The denominator `force[2]` is the sum of *all* vertical forces, while the numerator
sums only contacts above the threshold. If a contact with `Fz = 0.08 N` is active it
contributes to `force[2]` but not to the weighted ZMP position, biasing the result.
During stance transitions many such micro-contacts exist.

**Fix.** Use a single consistent force accumulation:

```python
valid_contacts = [c for c in self.world.getLastCollisionResult().getContacts()
                  if c.force[2] > 0.1]
force = sum((c.force for c in valid_contacts), np.zeros(3))
# then iterate valid_contacts for the ZMP sum
```

**Location:** [simulation.py:241–251](simulation.py#L241-L251)

---

### 5 — ZMP height derived from LIP equilibrium assumption introduces force-measurement noise `[Moderate]`

```python
# simulation.py:247
zmp[2] = com_position[2] - force[2] / (self.hrp4.getMass() * self.params['g'] / self.params['h'])
```

This uses the LIP balance identity `F_z ≈ m·g` to estimate floor height. Any deviation
from LIP equilibrium (swing phase, impacts, arm motion) makes `force[2] ≠ m·g` and
introduces noise into `zmp[2]`, which then contaminates the ZMP horizontal shear
correction `(zmp[2] − contact.point[2]) · contact.force[0]` in lines 250–251.

For a flat floor, the ground is at `z = 0` and the shear correction is negligible.
**Fix:** set `zmp[2] = 0.0` directly (or the known floor height) instead of deriving
it from the force measurement.

**Location:** [simulation.py:247](simulation.py#L247)

---

### 6 — Parameter choice places k_2 on the wrong side of zero `[Moderate]`

With the paper's parameters (α=−1, β=−8, ω=3.50, g_p=20):
`k_2 = −0.375` (negative).

With the code's parameters (α=−1, β=−8, η=3.69, g_p=10):
`k_2 = +0.269` (positive).

The sign flip occurs because `α + β + g_p − η` changes sign between the two parameter
sets. Both signs produce a stabilizing correction in the formula
`−k_2·(p_meas − p_ref)`, so this is not a correctness bug by itself. However, it
indicates that the code is operating in a qualitatively different gain regime from the
paper example, making direct comparison or tuning guidance from the paper unreliable.

**Suggested fix:** Either use the paper's g_p=20 (with h adjusted to match the actual
robot height h=0.72, giving ω=3.69), or re-derive the gains analytically for the
lagless IS-MPC plant as described in Finding #1.

**Location:** [simulation.py:39–51](simulation.py#L39-L51)

---

### 7 — `k_1` gain is off by a factor of `g_p²/η` due to operator-precedence bug `[Critical] [Fixed]`

**The formula (simulation.py:43):**

```python
self.params['k_1'] = -(numerator) / self.params['eta'] * self.params['g_p']
```

Python evaluates left-to-right: `a / b * c = (a / b) * c`, so this computes
`−numerator × g_p / η` instead of `−numerator / (η × g_p)`. With `g_p = 20`,
the gain is inflated by a factor of `g_p² = 400`:

```
k_1_wrong   ≈ −487    (with α=−4, β=−8, η=3.691, g_p=20)
k_1_correct ≈  −1.22  (same parameters)
```

The correct formula uses both η and g_p in the denominator, consistent with the
original derivation (verified: old params α=−1, g_p=10 → k_1=−1.486 only with
the division form).

**Fix** (already applied): wrap both denominators together:

```python
/ (self.params['eta'] * self.params['g_p'])
```

**Location:** [simulation.py:43](simulation.py#L43)

---

### 8 — `xi_ref ≡ xi_meas`: CP correction is permanently zero `[Critical] [Fixed]`

**The code after the merge (ismpc.py:110–111):**

```python
xi_meas = self.compute_cp(current['com']['pos'], current['com']['vel'])
xi_ref  = self.compute_cp(self.x[[0, 3, 6]], self.x[[1, 4, 7]])
```

`self.x` is assembled from `current` on the very first line of `solve()`, so
`self.x[[0,3,6]] = current['com']['pos']` and `self.x[[1,4,7]] = current['com']['vel']`
exactly. Therefore `xi_ref ≡ xi_meas` at every step, making
`(xi_meas − xi_ref) = 0` identically. Both the proportional term `−k_1·(...)` and
the integral `xi_error_int` are always zero; only the ZMP error term `−k_2·(...)` is
active.

This is a stronger form of the pre-existing Finding #2 (where xi_ref was one step
*ahead* instead of one step *behind*). The fix from Finding #2 is now implemented:
`xi_ref` is taken from `X[:,1]` of the **previous** MPC solve, stored in
`self._xi_ref_prev`. This gives the desired CP predicted at the current time t from
the MPC that ran at t−1.

**Fix** (already applied): [ismpc.py](ismpc.py#L37), [ismpc.py:107–119](ismpc.py#L107-L119)

---

### 9 — `xi_error_int` reset on contact loss targets wrong object `[Minor] [Fixed]`

When `force[2] ≤ 0.1`, `retrieve_state()` previously reset `self.xi_error_int` on
the controller (a stale copy logged after each `mpc.solve()`), not the live
integrator `self.mpc.xi_error_int` inside `Ismpc`. After fixing Finding #8, the
integral accumulates real non-zero values, so this mismatch causes the integrator to
run unchecked through contact-loss events.

**Fix** (already applied): `retrieve_state()` now resets `self.mpc.xi_error_int`
(guarded with `hasattr` for the initial call before `self.mpc` is created).

**Location:** [simulation.py:247–250](simulation.py#L247-L250)

---

### 10 — `use_kf`, `log_path`, `autosave_every` never set on `node` `[Bug] [Fixed]`

`Hrp4Controller.__init__` does not set these attributes, but `customPreStep()`
reads `self.use_kf` on every step and `autosave_every` at the autosave check, while
the `finally` block reads `node.use_kf`. Any run without explicit assignment crashes
immediately with `AttributeError`.

**Fix** (already applied): three attribute assignments added right after
`node = Hrp4Controller(world, hrp4)`:

```python
node.use_kf = not args.no_kf
node.log_path = args.log_path
node.autosave_every = args.autosave_every
```

**Location:** [simulation.py:308–310](simulation.py#L308-L310)

---

## Root cause of KF dependency

The robot falls without the Kalman filter for three compounding reasons:

**A. Noisy CoM velocity amplified by |k_1| > 1.**  
DART computes `getCOMLinearVelocity()` by analytical differentiation of joint angles
and velocities. During foot contacts, impulsive forces cause velocity spikes at the
rate of the simulation timestep (10 ms). With `|k_1| = 1.486`, these spikes are
amplified 1.49× in `p_cmd`, causing the ZMP command to chatter at high frequency.
The KF low-pass filters the CoM velocity before it reaches the controller.

**B. Discontinuous ZMP measurement at contact transitions.**  
DART's contact set changes discretely at each timestep as foot polygons engage/disengage.
The ZMP jumps by several centimetres at each transition. Without the KF, these jumps
directly enter both `p_meas` (ZMP error term) and `xi_meas` (CP error term). The KF
R-matrix `diag(1e1, 1e2, 1e4)` (position:vel:ZMP) trusts ZMP measurements 100× less
than position, effectively smoothing them.

**C. Noisy MPC initial condition breaks recursive feasibility.**  
The IS-MPC QP receives the raw (noisy) state as `x0_param`. Feasibility of the QP at
step k depends on the previous step's solution being a valid warm start. Noisy initial
conditions push the QP away from the previous solution, leading to erratic ZMP
references even without any feedback correction.

The stability constraint fix (Finding #3) only helps if the MPC receives smooth state
estimates. Without KF *and* without a lightweight filter, Findings A–C together cause
divergence within a few steps.

---

## Recommendations for robustness without KF

### R1 — Fix xi_ref (Finding #2): highest priority

This is the simplest code change and makes the CP feedback semantically correct.
It does not require a filter and reduces the effective noise in the CP error term
because the reference is now a stored value (smooth) rather than a live MPC output.

### R2 — EMA pre-filter as lightweight KF substitute

A single-pole low-pass filter on ZMP and CoM velocity costs four multiplications per
axis per step and requires no covariance bookkeeping:

```python
tau = 0.05  # 50 ms time constant
alpha_f = dt / (tau + dt)  # ≈ 0.167 at dt=10ms

zmp_filtered   = (1 - alpha_f) * zmp_prev   + alpha_f * zmp_raw
comvel_filtered = (1 - alpha_f) * comvel_prev + alpha_f * comvel_raw
```

This handles root causes A and B. The remaining step (C) is addressed by R3.

### R3 — Use the MPC internal state as the feedback state

Instead of passing raw sensor measurements to `mpc.solve()`, pass the MPC's own
propagated state `x_mpc` (Euler-integrated from the previous step's optimal control):

```python
# inside Ismpc, maintain self.x_mpc and integrate it each step:
self.x_mpc = self.x_mpc + self.delta * self.f(self.x_mpc, self.u).full().flatten()
```

The MPC QP always sees a kinematically consistent initial condition (no jumps),
removing root cause C. The CP-ZMP feedback term still uses sensor measurements to
correct for model/world mismatch.

### R4 — Fix the stability constraint (Finding #3)

Removing `−X[2]` from both sides of the terminal constraint restores the IS-MPC
stability guarantee for walking. This allows the MPC to maintain a tighter grip on
the CP trajectory even with noisier state inputs.

### R5 — Gate ZMP feedback during foot transitions

The ZMP measurement is least reliable during the first ~30 ms after a foot touchdown
(contact normal forces are still settling). Temporarily zeroing `k_2` during this
window (tracked by the step phase timer) prevents force-sensor noise from injecting
large ZMP correction impulses into `p_cmd`.

### R6 — Redesign gains for the actual (lagless) plant (Finding #1)

For the IS-MPC plant with effectively instantaneous ZMP tracking:

```python
# Place CP pole at alpha = -1, no ZMP error feedback needed:
params['k_1'] = alpha / eta - 1.0   # ≈ -1.271
params['k_2'] = 0.0
```

This gives exactly the designed CP pole without relying on the absent ZMP lag.
If ZMP error feedback is still desired, re-derive using g_p = 1/delta = 100.

---

## Change summary

This review does not change code. It supersedes the previous REVIEW.md entirely.
The previous findings #2–#4 were already fixed in the current codebase; only finding
#1 (CP feedback) was partially correct but incompletely described.
