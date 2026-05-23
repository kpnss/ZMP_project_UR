# CHANGES — Implemented findings from REVIEW.md

This document explains the theory behind each implemented change.
Findings are numbered as in `REVIEW.md`; only those actually applied to the codebase are listed.

---

## Finding #1 — Gains redesigned for the lagless IS-MPC plant

**Location:** `simulation.py:43–45`

### Theory

Morisawa (2012) derives the CP-ZMP balance gains for a plant in which the ZMP obeys
a first-order lag driven by a desired-ZMP command `p_d`:

```
ṗ = −g_p · p  +  g_p · p_d
```

The closed-loop CP error dynamics around this lag yields a 2nd-order system whose
characteristic polynomial depends on `g_p`. Setting two roots to the desired poles
`α` and `β` gives `k_1` and `k_2` explicitly in terms of `η`, `g_p`, `α`, `β`.

The IS-MPC (Scianca 2020) instead uses **ZMP velocity** as the control input — the
third row of `A_lip` is `[0,0,0]` and `B_lip = [0,0,1]ᵀ`, which means the ZMP
integrates the commanded velocity directly with no lag (g_p → ∞).

After the IS-MPC solve, the code converts the ZMP position command to a velocity:

```python
self.desired['zmp']['vel'] = (p_cmd - self.current['zmp']['pos']) / world_time_step
```

This sets `p_{k+1} = p_cmd` in a single timestep — identical to a lag with time
constant equal to one simulation step, or g_p ≈ 100 s⁻¹.

**Consequence of applying the Morisawa gains to this plant:**
The effective closed-loop CP pole is

```
λ_eff = η · (1 + k_1 / (1 + k_2))
```

With the original parameters this gives λ_eff ≈ −0.63 instead of the designed α = −1,
making the CP correction ~37 % weaker than intended.

**Fix — lagless gains:**

With instantaneous ZMP tracking there is no ZMP lag to exploit, so `k_2 = 0`.
The only free parameter is `k_1`, which is solved by requiring the CP pole to equal `α`:

```
ξ̇ = η·(ξ − p)   ⟹   ξ̇ + p_cmd = η·(ξ − p_ref) − k_1·(ξ − ξ_ref)
```

Setting the pole of the homogeneous part to `α` gives:

```
α = η + η·k_1   ⟹   k_1 = α/η − 1
```

Implemented as:

```python
self.params['k_1'] = self.params['alpha'] / self.params['eta'] - 1.0
self.params['k_2'] = 0.0
```

---

## Finding #2 / #8 — CP reference alignment (`xi_ref`)

**Location:** `ismpc.py:34, 123–127`

### Theory

Morisawa eq. 14 requires `ξ_ref(t)` to be the **desired CP at the current instant**,
so the correction term `−k_1·(ξ_meas − ξ_ref)` is a proportional position error.

The MPC solve at time `t` produces a predicted state sequence `X[:,0..N]`.
`X[:,0]` is the initial condition (time `t`); `X[:,1]` is the state predicted
for time `t + δ`.  
Therefore `X[:,1]` from the **previous** solve is the MPC's prediction of the state at
the **current** time `t` — exactly what `ξ_ref(t)` should be.

Two bugs were previously breaking this:
- **#8 (post-merge):** `xi_ref` was recomputed from the live sensor state at each step,
  making it identical to `xi_meas` and zeroing the entire proportional term.
- **#2 (pre-merge):** `xi_ref` was taken from `X[:,0]` of the current solve — one
  step *ahead* — so the "error" was actually approximating `−δ · dξ/dt`,
  i.e. derivative feedback instead of proportional feedback.

**Fix:**

Store `X[:,1]` at the end of every solve and use it as `xi_ref` on the next call:

```python
# initialised as None; no correction on the very first step
self._xi_ref_prev = None

# inside solve(), after computing x_next = sol.value(X[:,1]):
if self._xi_ref_prev is not None:
    xi_ref = self.compute_cp(self._xi_ref_prev[[0,3,6]], self._xi_ref_prev[[1,4,7]])
else:
    xi_ref = xi_meas          # first step: no correction
self._xi_ref_prev = x_next
```

This gives a one-step-delayed, kinematically smooth reference and ensures the
proportional term is a genuine CP position error.

---

## Finding #3 — IS-MPC stability constraint: pure CP periodicity

**Location:** `ismpc.py:73–79`

### Theory

The IS-MPC terminal constraint (Scianca 2020, eq. 23) imposes that the **divergent
component of motion** (capture point) is periodic over the horizon:

```
ξ(t_k + T_c) = ξ(t_k)     where  ξ = ẋ_c/η + x_c
```

Equivalently, since `ξ = com_vel/η + com_pos`:

```
com_vel(N)/η + com_pos(N)  =  com_vel(0)/η + com_pos(0)
```

In state-vector notation (x-axis block: indices 0=pos, 1=vel, 2=ZMP):

```
X[1,N] + η·X[0,N]  =  X[1,0] + η·X[0,0]
```

The buggy version subtracted `η·X[2,·]` from both sides:

```python
# WRONG
X[1,0] + η*(X[0,0] - X[2,0]) == X[1,N] + η*(X[0,N] - X[2,N])
```

This enforces `(ξ − p)(0) = (ξ − p)(N)`, i.e. **CP-minus-ZMP is periodic**, not the
CP itself. During walking the ZMP reference shifts forward between footsteps
(`p(0) ≠ p(N)`), so this constraint allows the CP to drift by the same amount as the
ZMP — precisely in the direction of instability. The IS-MPC convergence proof no
longer applies.

**Fix:**

```python
# CORRECT — pure CP periodicity
self.opt.subject_to(
    self.X[1, 0     ] + self.eta * self.X[0, 0     ] ==
    self.X[1, self.N] + self.eta * self.X[0, self.N]
)
# same for Y and Z axes
```

---

## Finding #4 — ZMP normalization: consistent contact set

**Location:** `simulation.py:232–234`

### Theory

The ZMP is defined as the point at which the net ground-reaction moment has no
horizontal component. For `n` contact points with forces `f_i` and positions `r_i`:

```
p_x = Σ (r_{i,x} · f_{i,z})  /  Σ f_{i,z}
```

Both numerator and denominator **must sum over exactly the same set of contacts**.
If the denominator includes a contact `j` with `f_{j,z} = 0.08 N` (below the 0.1 N
threshold) while the numerator skips it, the normalisation is wrong:

```
p_x ≈ Σ_{valid} r_{i,x}·f_{i,z}  /  (Σ_{valid} f_{i,z} + 0.08)
```

The small residual force in the denominator biases `p_x` towards the origin. During
stance transitions many such micro-contacts exist (DART's contact set changes
discretely), causing systematic bias in the ZMP estimate that corrupts both the CP
feedback term and the MPC initial condition.

**Fix:**

Build one list of *valid* contacts and use it everywhere:

```python
valid_contacts = [c for c in world.getLastCollisionResult().getContacts()
                  if c.force[2] > 0.1]
force = sum((c.force for c in valid_contacts), np.zeros(3))
# numerator loop also iterates valid_contacts
```

---

## Finding #6 — k₂ sign: resolved by the lagless redesign

**Location:** `simulation.py:44`

### Theory

With the Morisawa lag-plant parameters and `g_p = 20 s⁻¹`, the gain formula yields
`k_2 = −0.375` (negative ZMP error feedback). With the code's parameters
(`η = 3.69`, `g_p = 10`), the same formula gives `k_2 = +0.269` (positive), because
the sign of `α + β + g_p − η` flips between the two parameter sets.

Both signs are stabilising in isolation, but they correspond to qualitatively different
operating regimes, making the code's behaviour difficult to compare against the paper's
examples or to tune from the paper's intuitions.

This ambiguity is eliminated by the lagless redesign (Finding #1): setting `k_2 = 0`
removes ZMP error feedback entirely. The CP pole is placed exactly at `α` by `k_1`
alone, without any dependence on `g_p` or the sign of that expression.

---

## Additional: internal MPC state propagation (Recommendation R3)

**Location:** `ismpc.py:93–107`

### Theory

The IS-MPC feasibility proof assumes that the initial condition `x_0` fed to the QP
at each step is the *result of applying the previous optimal control* to the previous
initial condition (recursive feasibility). In a real closed-loop system the measured
state differs from the propagated one due to sensor noise, model mismatch, and
discretisation.

When the noise is large (contact impulses, discontinuous ZMP jumps — see REVIEW.md
section "Root cause of KF dependency"), the QP is initialised far from the warm-start
solution, forcing the solver to search a larger region and potentially producing
erratic ZMP references even before any feedback correction is applied.

**Fix:** maintain an internal `x_mpc` variable that is Euler-integrated using the
LIP dynamics and the previous optimal input `u`:

```python
dx = [A_lip @ x_mpc[0:3] + B*u[0],
      A_lip @ x_mpc[3:6] + B*u[1],
      A_lip @ x_mpc[6:9] + B*u[2] + g_vec]
x_mpc += delta * dx
```

The QP always receives a kinematically consistent initial condition. The CP-ZMP
feedback term still uses the raw (or KF-filtered) sensor measurements to correct for
accumulated model error, so the two signals serve complementary roles:
`x_mpc` stabilises the QP; `(xi_meas − xi_ref)` corrects for plant mismatch.
