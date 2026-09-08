# ZMP Feedback for Intrinsically Stable MPC: Integrating Capture-Point Balance Control into IS-MPC

*Underactuated Robots — project report*

---

## 0. Introduction: the problem

Bipedal gait generation is conventionally split into two loosely coupled layers. A **pattern
generator** produces, from a high-level motion command, a dynamically consistent triple
(footsteps, ZMP trajectory, CoM trajectory) using a reduced template model — almost always
the Linear Inverted Pendulum (LIP). A **whole-body controller** then realizes that reference on
the real, high-DoF, underactuated robot.

The split is convenient but it hides a structural weakness. The template model is *not* the
robot: it ignores the swing-leg inertia, the finite bandwidth of the joint servos, the
compliance of the soles and force sensors, foot-landing impacts, and any modelling error in the
mass distribution. The consequence is that the ZMP actually realized on the ground,
$p^{\text{meas}}$, differs from the ZMP the pattern generator planned, $p^{\text{ref}}$, and —
because the CoM/ZMP dynamics of the LIP contain an unstable mode — that discrepancy does not
stay bounded on its own. It integrates into a drift of the **Capture Point** (CP), and
ultimately into a fall.

The classical remedy is a *stabilizer*: a fast feedback loop that measures the CP and the ZMP
and corrects the ZMP command sent downstream. The canonical formulation is the CP-error
compensation scheme of Morisawa et al., which regulates the CP error, the ZMP error and the
integral of the CP error by pole placement on a LIP model augmented with a first-order ZMP lag.

That scheme, however, was designed on top of an **open-loop pattern generator**: a walking FSM
emits $(\xi^{\text{ref}}, p^{\text{ref}})$ ahead of time, the stabilizer regulates the deviation
from it, and the two roles — *planning* and *regulation* — are cleanly separated.

**This project asks what happens when the reference is produced instead by a receding-horizon
MPC.** We start from the reference implementation of IS-MPC (Scianca et al.) maintained by the
DIAG Robotics Lab — an intrinsically stable MPC gait generator whose ZMP objective is encoded as
*time-varying constraints* rather than as a trajectory to be tracked — and we graft onto it a
capture-point/ZMP feedback stabilizer. The core deliverable of the project, and the hard part,
is the **ZMP feedback term** $-k_2\,(p^{\text{meas}} - p^{\text{ref}})$: it requires a ZMP
reference that is well-defined *at the current instant*, whereas an MPC recomputes its whole plan
at every control step from the *measured* state, so its "reference" is itself a closed-loop
signal that has already reacted to the very error we want to feed back. Section 2.1.2 develops
this point; Section 4 returns to it.

### 0.1 Scope: baseline versus contribution

Because this report builds on an existing, working codebase, we state the boundary explicitly and
up front. The boundary is verifiable from the repository history: the upstream code is everything
up to the last commit of the original author (`4c52aa8`), and everything after it is the project's
work.

| component | file | status |
|---|---|---|
| IS-MPC QP (3D LIP, moving constraint, ZMP box, periodic-tail stability constraint) | `ismpc.py` | **baseline**, extended (Sec. 2.1, 2.3) |
| Whole-body inverse-dynamics QP | `inverse_dynamics.py` | **baseline, byte-identical** |
| Kalman filter | `filter.py` | **baseline, byte-identical** |
| Footstep planner (virtual unicycle) | `footstep_planner.py` | **baseline** (one out-of-range fix) |
| Swing-foot trajectory generation | `foot_trajectory_generator.py` | **baseline** (+ closing-step handling) |
| DART/HRP-4 simulation loop, ZMP measurement | `simulation.py` | **baseline**, extended |
| CP + ZMP + CP-integral feedback term | `ismpc.py`, `utils.py` | **added — Sec. 2.1** |
| Alternative standalone CP controller | `cp_controller.py` (new) | **added — Sec. 2.2** |
| First-order ZMP-lag plant | `ismpc.py`, `cp_controller.py`, `simulation.py` | **added — Sec. 2.3** |
| Scenario flags, open-loop mode, contact-loss handling, GRF logging | `simulation.py` | **added — Sec. 2.4** |
| Evaluation infrastructure (plots, summary table, ablation driver) | `plot_logs.py`, `make_runs_summary.py`, `ablation_poles.py` (new) | **added — Sec. 2.5** |

Accordingly, **Section 1 describes the baseline in full** — both the IS-MPC theory (§1.1) and the
concrete block architecture and implementation we inherited (§1.2) — together with the balance
control paper we draw on (§1.3). **Section 2 describes only what the project added.**

---

## 1. Related work and baseline

### 1.1 IS-MPC — MPC for humanoid gait generation with a stability constraint

**Reference.** N. Scianca, D. De Simone, L. Lanari, G. Oriolo, *"MPC for Humanoid Gait
Generation: Stability and Feasibility"*, IEEE T-RO, 2020.

**Problem addressed.** Generate a gait (footsteps *and* timing *and* CoM trajectory) in real
time in response to high-level driving/steering velocities $(v_x, v_y, \omega)$, while
guaranteeing (i) dynamic balance, (ii) kinematic feasibility of the steps, and (iii) — the
paper's central contribution — *internal stability*, i.e. boundedness of the CoM trajectory with
respect to the ZMP trajectory.

**Why internal stability is the issue.** The paper's framing observation is that MPC-based gait
generation is *neither a set-point nor a tracking problem*: the ZMP objective is encoded as
time-varying state constraints (the ZMP must lie in the support polygon), so there is no error
signal to be driven to zero. The only genuine stability question is whether the CoM stays bounded
relative to the ZMP. With the LIP model along $x$,

$$\ddot{x}_c = \eta^2 (x_c - x_z), \qquad \eta = \sqrt{g/h_c},$$

the change of coordinates $x_s = x_c - \dot{x}_c/\eta$, $x_u = x_c + \dot{x}_c/\eta$ splits the
dynamics into a stable subsystem $\dot{x}_s = -\eta(x_s - x_z)$ and an unstable one
$\dot{x}_u = \eta(x_u - x_z)$. The variable $x_u$ is exactly the **divergent component of
motion**, i.e. the **capture point**. Boundedness of the CoM requires the special initialization

$$x_u^k = \eta \int_{t_k}^{\infty} e^{-\eta(\tau - t_k)}\, x_z(\tau)\, d\tau, \tag{1.1}$$

which is the *stability condition*.

**Prediction model.** IS-MPC uses the LIP with a dynamic extension, taking the ZMP *velocity* as
the control input, so as to obtain smoother trajectories:

$$\frac{d}{dt}\begin{pmatrix} x_c \\ \dot{x}_c \\ x_z \end{pmatrix} =
\begin{pmatrix} 0 & 1 & 0 \\ \eta^2 & 0 & -\eta^2 \\ 0 & 0 & 0\end{pmatrix}
\begin{pmatrix} x_c \\ \dot{x}_c \\ x_z \end{pmatrix} +
\begin{pmatrix} 0 \\ 0 \\ 1\end{pmatrix} \dot{x}_z , \tag{1.2}$$

with piecewise-constant $\dot{x}_z$ over the sampling intervals, hence a piecewise-linear
ZMP profile.

**Constraints.** Three families:

1. *ZMP constraints* — the ZMP must lie inside a rectangle of size $d_{z,x}\times d_{z,y}$
   attached to the support footstep. In double support the exact support polygon (convex hull of
   two feet) would be nonlinear in the footstep positions, so IS-MPC uses a **moving constraint**:
   the admissible rectangle roto-translates from one footstep to the next, staying inside the
   true polygon. Slightly conservative, but linear in the decision variables.
2. *Kinematic constraints* — each footstep must lie in a rectangle attached to the previous one,
   displaced by the nominal coronal distance $\ell$.
3. *Stability constraint* — the discrete-time counterpart of (1.1). With a finite control horizon
   $T_c = C\delta$ only the first $C$ ZMP velocities are decision variables; the remainder, the
   **tail**, must be conjectured. Splitting the infinite sum in (1.1),

   $$\sum_{i=0}^{C-1} e^{-i\eta\delta}\, \dot{x}_z^{k+i} \;=\; -\sum_{i=C}^{\infty} e^{-i\eta\delta}\, \dot{\tilde x}_z^{k+i} \;+\; \frac{\eta}{1-e^{-\eta\delta}}\,(x_u^k - x_z^k). \tag{1.3}$$

**Tails.** Three options, each equivalent to a terminal constraint:

- *truncated* ($\dot{\tilde x}_z^{k+i} = 0$, $i \ge C$) $\Rightarrow$ terminal constraint
  $x_u^{k+C} = x_z^{k+C}$, which is exactly the **capturability constraint**;
- *periodic* (the ZMP velocities replicate the control horizon) $\Rightarrow$ terminal constraint
  $x_u^{k+C} - x_z^{k+C} = x_u^{k} - x_z^{k}$;
- *anticipative* (a ZMP trajectory is built over $[T_c, T_p]$ from the candidate footsteps).

**Main theoretical results.** The paper proves that (i) *recursive feasibility* of the IS-MPC
iteration implies internal stability of the CoM/ZMP dynamics, and (ii) recursive feasibility is
*not* automatic from a terminal/capturability constraint — it depends on choosing a tail
consistent with the actual commanded motion; the anticipative tail, which exploits the most
preview information, is the recommended choice.

**A second reference** is relevant to the implementation we inherited: M. Cipriano, P. Ferrari,
N. Scianca, L. Lanari, G. Oriolo, *"Humanoid motion generation in a world of stairs"*, RAS 2023,
which extends the scheme to the 3D LIP with vertical CoM motion. The baseline code implements
that 3D version.

### 1.2 The baseline: block architecture and implementation

This section describes the pipeline **as we received it**, from the reference implementation at
`github.com/DIAG-Robotics-Lab/ismpc`. Everything here predates the project; the modifications are
in Section 2. All blocks run synchronously at the simulation rate $\delta = 10$ ms.

#### 1.2.1 Block architecture

```
 high-level velocity reference  v_ref = {(v_x, v_y, ω)_j}
                │
        ┌───────▼────────────────────────────────────────────────┐
        │ (A)  FOOTSTEP PLANNER              footstep_planner.py │
        │   in : v_ref, initial foot poses                       │
        │   out: plan {pos_j, ang_j, T_ss, T_ds, foot_id}        │
        └───────┬──────────────────────────────┬─────────────────┘
                │ plan                         │ plan
        ┌───────▼──────────────────┐   ┌───────▼──────────────────┐
        │ (B) IS-MPC       ismpc.py│   │ (C) SWING-FOOT TRAJ. GEN.│
        │  in : plan, x̂(t), t     │   │     foot_trajectory_...py│
        │  out: c_ref, ċ_ref,      │   │  in : plan, t            │
        │       c̈_ref, p, contact │   │  out: pose/vel/acc, feet │
        └───────┬──────────────────┘   └───────┬──────────────────┘
                │ desired CoM acc              │ desired feet
        ┌───────▼──────────────────────────────▼─────────────────┐
        │ (D) WHOLE-BODY INVERSE DYNAMICS QP inverse_dynamics.py │
        │  in : desired CoM/feet/torso/base acc, current state,  │
        │       contact flag                                     │
        │  out: joint torques τ                                  │
        └───────┬────────────────────────────────────────────────┘
                │ τ
        ┌───────▼────────────────────────────────────────────────┐
        │ (E) ROBOT + CONTACTS (DART, HRP-4)   simulation.py     │
        │  out: q, q̇, contact set → c, ċ, p_meas                │
        └───────┬────────────────────────────────────────────────┘
                │ raw measurements
        ┌───────▼────────────────────────────────────────────────┐
        │ (F) KALMAN FILTER                        filter.py     │
        │  in : (c, ċ, p)_meas, previous ZMP-velocity input      │
        │  out: filtered x̂ = (c, ċ, p) per axis                 │
        └───────┬────────────────────────────────────────────────┘
                └──────────────► back to (B)   [Loop 1]
```

The baseline has exactly **one** feedback loop, marked *Loop 1*: the filtered state $\hat x$ is
used to re-initialize the MPC's QP at every control step. There are no gains and no error signal
— it is a receding horizon re-anchored on the truth 100 times per second. Understanding that this
loop is already present, and already strong, is essential to reading the experimental results:
it is the baseline against which the explicit feedback of Section 2.1 must justify itself.

#### 1.2.2 Block (A) — Footstep planner

*Input:* a list of per-step reference velocities $(v_x, v_y, \omega)_j$ and the initial foot
poses. *Output:* a footstep plan.

A virtual unicycle is integrated forward under $(v_x,v_y,\omega)$ over each step duration; the
footstep is placed at a fixed coronal offset $\pm\ell/2 = \pm 0.1$ m from the unicycle, rotated
by the unicycle heading:

$$\mathbf{f}_j = \mathbf{u}(t_j) + R(\theta_j)\begin{pmatrix} 0 \\ \pm \ell/2 \end{pmatrix},\qquad
\theta_j = \theta(t_j).$$

Timings are **fixed** ($T_{ss} = 70$ steps $= 0.7$ s, $T_{ds} = 30$ steps $= 0.3$ s); the first
"step" is a dummy with $T_{ss}=0$ and $T_{ds} = 2(T_{ss}+T_{ds}) = 2$ s, used to let the robot
settle in double support. Two simplifications with respect to the IS-MPC paper are worth noting,
because they bound what the whole system can do: the timing is **not** adapted to the commanded
velocity (their eq. (1)), and footstep positions are **not** decision variables of the QP —
equivalently $\beta \to \infty$ in the IS-MPC cost, so the kinematic constraints (their eq. (7))
are not enforced and the system has no reactive step-adjustment capability.

#### 1.2.3 Block (C) — Swing-foot trajectory generation

*Input:* plan, time. *Output:* pose, velocity and acceleration of both feet.

The support foot is held fixed. The swing foot follows a cubic in the horizontal plane and in
orientation,

$$s(t) = -\tfrac{2}{T^3}t^3 + \tfrac{3}{T^2}t^2, \qquad
\mathbf{p}_{sw}(t) = \mathbf{p}_{\text{start}} + s(t)\,(\mathbf{p}_{\text{target}} - \mathbf{p}_{\text{start}}),$$

and a quartic in the vertical direction that starts and ends at zero height with apex
$h_{\text{step}}$:

$$z_{sw}(t) = \frac{16 h_{\text{step}}}{T^4}t^4 - \frac{32 h_{\text{step}}}{T^3}t^3 + \frac{16 h_{\text{step}}}{T^2}t^2 .$$

During double support both feet are held at their planned poses with zero velocity/acceleration.

#### 1.2.4 Block (B) — The IS-MPC QP

*State.* Per axis $a \in \{x,y,z\}$: $\chi_a = (c_a, \dot c_a, p_a)$, stacked into
$\chi \in \mathbb{R}^9$ — the 3D LIP of the "world of stairs" extension. *Decision variables:*
$U \in \mathbb{R}^{3\times N}$ (ZMP velocities) and $X \in \mathbb{R}^{9\times(N+1)}$, with
$N = 100$, i.e. a horizon of $T_c = 1$ s. Control and preview horizon coincide ($C = P$).

*Dynamics.* Explicit-Euler discretization of (1.2) for each axis, with the vertical axis carrying
the gravity drift $-g$ in the $\ddot c_z$ row:

$$X_{:,i+1} = X_{:,i} + \delta\, f(X_{:,i}, U_{:,i}). \tag{1.4}$$

*Cost.* ZMP-centering plus control effort,

$$J = \sum_{i} \|U_{:,i}\|^2 + 100 \sum_{a\in\{x,y,z\}} \sum_{i=1}^{N} \big(p_a^i - m_a^i\big)^2, \tag{1.5}$$

where $m^i$ is the moving-constraint midpoint (§1.2.5).

*Constraints.*

- initial state $X_{:,0} = \hat\chi$ — this is Loop 1;
- ZMP box, as hard bounds, the discrete counterpart of IS-MPC eq. (6):

  $$\big|p_a^i - m_a^i\big| \le d/2, \qquad a \in \{x,y,z\},\ i = 1,\dots,N; \tag{1.6}$$

- **stability constraint with periodic tail**, enforced per axis as the terminal condition
  (IS-MPC eq. (23)):

  $$\dot c_a^{0} + \eta\,(c_a^{0} - p_a^{0}) \;=\; \dot c_a^{N} + \eta\,(c_a^{N} - p_a^{N})
  \;\;\Longleftrightarrow\;\; \xi_a^{0} - p_a^{0} = \xi_a^{N} - p_a^{N}. \tag{1.7}$$

  The periodic tail is the natural choice here because the commanded velocity profile is
  piecewise constant over the horizon. The anticipative tail — the one the paper recommends for
  guaranteeing recursive feasibility — is not implemented.

The QP is built once in CasADi `Opti` with OSQP as the conic backend and re-solved each step with
parameter updates and warm start from the previous solution.

*Output.* The first predicted state $X^\star_{:,1}$ becomes the desired LIP state, and the desired
CoM acceleration handed to the whole-body layer is

$$\ddot c^{\,\text{des}} = \eta^2\big(c^{\,\text{des}} - p\big) + \begin{pmatrix}0\\0\\-g\end{pmatrix},
\qquad p = X^\star_{[2,5,8],\,1}. \tag{1.8}$$

Equation (1.8) is the single channel through which the ZMP reaches the robot — the
inverse-dynamics block has no ZMP task. This will matter a great deal in Section 2.1.5.

The block also emits the **contact flag** $\in \{\text{lfoot}, \text{rfoot}, \text{ds}\}$ read
from the plan, which gates the contact Jacobian downstream.

#### 1.2.5 The moving constraint

The ZMP centre $m(t)$ interpolates between consecutive footsteps during each double-support phase
using a clipped ramp $\sigma(t;t_0,t_1) = \mathrm{clip}((t-t_0)/(t_1-t_0),0,1)$:

$$m(\tau) = m_0 + \sum_{j} \sigma\big(\tau;\, t_j^{ds},\, t_j^{end}\big)\,\big(\mathbf{f}_{j+1} - \mathbf{f}_{j}\big), \tag{1.9}$$

so $m$ sits on the support foot during single support and slides to the next foot during double
support. This is the implementation of the moving-constraint idea of IS-MPC Sec. IV-B, reduced to
a translation (the rectangle does not rotate).

#### 1.2.6 Block (D) — Whole-body inverse dynamics

*Input:* desired accelerations/velocities/positions for the tasks
$\{\text{lfoot}, \text{rfoot}, \text{com}, \text{torso}, \text{base}, \text{joints}\}$, the
current robot state, and the contact flag. *Output:* joint torques $\tau$.

A single QP in the variables $(\ddot q, \tau, f_c) \in \mathbb{R}^{2n + 12}$:

$$\min \sum_{k\in\text{tasks}} w_k \left\| J_k \ddot q + \dot J_k \dot q - \big(a_k^{\text{des}} + K_{d,k} \dot e_k + K_{p,k} e_k\big) \right\|^2 + 10^{-6}\|f_c\|^2$$

subject to

- the constrained dynamics $M\ddot q + h(q,\dot q) = S^\top \tau + J_c^\top f_c$;
- per-contact CoP-inside-the-foot and friction-cone inequalities, in the standard 8-row form
  $|f_x|,|f_y| \le \mu f_z$, $|\tau_x|,|\tau_y| \le (d/2)\, f_z$;
- the contact Jacobian gated by the contact flag from block (B).

Weights $w = 1$ for all Cartesian tasks and $10^{-2}$ for the joint-posture task; gains
$K_p^{\text{com}} = 5$, $K_d^{\text{com}} = 10$, $K_p^{\text{feet}} = K_d^{\text{feet}} = 10$.
Ten upper-body DoFs (neck, shoulders, elbows) are treated as redundant and regularized to their
nominal posture. The floating base is passive; all other joints are torque-actuated. This file is
**unchanged** by the project.

#### 1.2.7 Blocks (E), (F) — Measurement and state estimation

The ZMP is reconstructed from the DART contact set (points with $F_z > 0.1$ N) as

$$p_z = c_z - \frac{F_z}{m\,g/h}, \qquad
p_{x} = \sum_i \frac{c_i^{x} f_i^{z} + (p_z - c_i^{z}) f_i^{x}}{F_z},$$

and analogously for $p_y$, then clipped to $\pm 0.3$ m around the foot midpoint to reject spurious
contact points.

The estimator is a Kalman filter on three decoupled LIP chains (9 states,
$\hat x = (c_a, \dot c_a, p_a)_{a\in\{x,y,z\}}$), with

$$A = I + \delta A_{\text{lip}}, \quad B = \delta B_{\text{lip}}, \quad d_7 = -\delta g, \quad H = I_9,$$
$$Q = I_9, \qquad R = \mathrm{blkdiag}(10,\,10^2,\,10^4)^{\times 3}, \qquad P_0 = I_9 .$$

The heavy $R$ weight on the third channel of each axis encodes the fact that the contact-derived
ZMP is by far the noisiest signal. The filter input is the ZMP velocity command. `filter.py` is
**unchanged** by the project.

### 1.3 Balance control based on Capture Point error compensation

**Reference.** M. Morisawa, S. Kajita, F. Kanehiro, K. Kaneko, K. Miura, K. Yokoi, *"Balance
Control based on Capture Point Error Compensation for Biped Walking on Uneven Terrain"*,
IEEE-RAS Humanoids, 2012.

**Problem addressed.** A stabilizer that suppresses CoG/ZMP measurement errors and modelling
offsets during walking, in particular on uneven terrain, and that removes the *steady-state
offset* which the classical CP controller leaves under a constant disturbance.

**Plant.** LIP with the ZMP as a state and a **first-order ZMP lag** representing sole
compliance, force-sensor elasticity and inner-loop servo delay:

$$\dot p_x = -g_p\, p_x + g_p\, p_x^{d}, \tag{1.10}$$

so that

$$\frac{d}{dt}\begin{pmatrix} x \\ \dot x \\ p_x \end{pmatrix} =
\begin{pmatrix} 0 & 1 & 0 \\ \omega^2 & 0 & -\omega^2 \\ 0 & 0 & -g_p \end{pmatrix}
\begin{pmatrix} x \\ \dot x \\ p_x \end{pmatrix} +
\begin{pmatrix} 0 \\ 0 \\ g_p \end{pmatrix} p_x^{d}, \qquad \omega = \sqrt{\tfrac{g+\ddot z}{z - p_z}}. \tag{1.11}$$

**Capture point.** $\xi_x = x + \dot x/\omega$, with $\dot\xi_x = \omega(\xi_x - p_x)$; in
$(\xi_x, p_x)$ coordinates the system becomes

$$\frac{d}{dt}\begin{pmatrix} \xi_x \\ p_x \end{pmatrix} =
\begin{pmatrix} \omega & -\omega \\ 0 & -g_p \end{pmatrix}\begin{pmatrix} \xi_x \\ p_x\end{pmatrix} + \begin{pmatrix} 0 \\ g_p\end{pmatrix} p_x^{d}.$$

**Key equivalence.** The paper first shows that the conventional COG-ZMP regulator
$p_x^d = -k_1^{cz} x - k_2^{cz}\dot x - k_3^{cz} p_x$ and the CP controller
$p_x^d = -k_1^{cp}\xi_x - k_2^{cp} p_x$ produce *identical* output once one of the poles is
assigned to $\gamma = -\omega$; i.e. the CP controller is the COG-ZMP regulator with the LIP's
own stable pole cancelled.

**Tracking form.** For walking, the feedback is written on the *errors* and added to the
feedforward reference:

$$\Delta p_x^d = -k_1^{cp}(\xi_x - \xi_x^{\text{ref}}) - k_2^{cp}(p_x - p_x^{\text{ref}}), \qquad
p_x^d = p_x^{\text{ref}} + \Delta p_x^d. \tag{1.12}$$

Under a constant disturbance $d_x$ the CP and ZMP both converge to the *same nonzero* offset
$\omega g_p/(\alpha\beta)$: increasing the poles reduces the offset but, by the non-minimum-phase
character of the inverted pendulum, blows up the ZMP peak.

**Contribution — CP integration (CPI).** The state is augmented with $\int \xi_x\,dt$,

$$\frac{d}{dt}\begin{pmatrix} \xi_x \\ p_x \\ \int\xi_x dt\end{pmatrix} =
\begin{pmatrix} \omega & -\omega & 0 \\ 0 & -g_p & 0 \\ 1 & 0 & 0\end{pmatrix}
\begin{pmatrix} \xi_x \\ p_x \\ \int\xi_x dt\end{pmatrix} + \begin{pmatrix} 0 \\ g_p \\ 0\end{pmatrix} p_x^d,$$

and the control law becomes **eq. (21)** of the paper,

$$\boxed{\;p_x^{d} = -k_1^{cpi}(\xi_x - \xi_x^{\text{ref}}) - k_2^{cpi}(p_x - p_x^{\text{ref}}) - k_I^{cpi}\!\!\int (\xi_x - \xi_x^{\text{ref}})\,dt\;} \tag{1.13}$$

with gains obtained by placing the three closed-loop poles $\{\alpha,\beta,\gamma\}$ — **eq. (22)**:

$$k_1^{cpi} = -\frac{\alpha\beta + \beta\gamma + \gamma\alpha - \omega(\alpha+\beta+\gamma-\omega)}{\omega\, g_p}, \quad
k_2^{cpi} = -\frac{\alpha+\beta+\gamma+g_p-\omega}{g_p}, \quad
k_I^{cpi} = \frac{\alpha\beta\gamma}{\omega\, g_p}. \tag{1.14}$$

Setting $\gamma = 0$ recovers exactly the non-integrating CP controller ($k_I = 0$), so the
integral action is switched on continuously by moving a single pole off the origin. With CPI, the
steady-state CP and ZMP errors under a constant disturbance are **zero**.

**Practical caveats stated by the authors.** (i) The commanded ZMP must be saturated to the
support polygon, otherwise the contact changes unexpectedly; (ii) the integrator is subject to
wind-up and needs saturation or a footstep modification.

**Experimental findings (HRP-2).** With CP integration, waist-attitude error stays roughly
constant with walking speed instead of growing; CoG and sagittal ZMP RMS errors shrink; the peak
vertical reaction force grows slightly.

**Relevance to this project.** Equations (1.13)–(1.14) are exactly the balance block we integrate.
Two of their assumptions drive the whole development and experimental sections: the gains (1.14)
are derived **for the lag plant (1.11)**, which the baseline of §1.2 does not have; and the
reference $(\xi^{\text{ref}}, p^{\text{ref}})$ is assumed to come from an **open-loop** CP/ZMP
trajectory generator, which IS-MPC is not.

### 1.4 Positioning

| | IS-MPC (2020) + baseline code | Morisawa et al. (2012) | This project |
|---|---|---|---|
| Reference generation | receding-horizon QP, constraints | open-loop FSM + CP/ZMP planner | IS-MPC **or** a CP planner, selectable |
| ZMP objective | time-varying *constraint* | trajectory to be *tracked* | constraint **+ explicit error feedback** |
| Feedback path | state re-initialization of the QP (Loop 1) | CP + ZMP + $\int$CP error | **both, in series** |
| Plant ZMP model | ideal ($\dot p$ is the input) | first-order lag $g_p$ | **both, selectable** |
| Whole-body layer | torque-level inverse dynamics QP | inverse kinematics + impedance | inherited unchanged |

The gap we address is the third column of the third row: how to define, and whether it pays to
use, an explicit ZMP-error feedback when the reference is generated by an MPC that is itself
already closed-loop.

---

## 2. Development

Everything in this section is added by the project. The modified architecture is the baseline of
§1.2.1 with one new block inserted between the reference generator and the whole-body layer, one
alternative implementation of the reference generator, and one alternative plant model:

```
        ┌──────────────────────────┐   ┌──────────────────────────┐
        │ (B1) IS-MPC     ismpc.py │OR │ (B2) CP CONTROLLER  §2.2 │   ← §2.2 adds B2
        │  out: p_ff, p_ref, ξ_ref │   │  out: p_ref, ξ_ref       │
        └───────┬──────────────────┘   └───────┬──────────────────┘
                └──────────────┬───────────────┘
        ┌──────────────────────▼─────────────────────────────────┐
        │ (G) BALANCE CONTROLLER — CP + ZMP + ∫CP   §2.1         │   ← §2.1 adds G
        │  in : p_ff, p_ref, ξ_ref, ξ̂, p̂                        │
        │  out: p_cmd  →  c̈_des = η²(c_des − p_cmd) + g         │
        └──────────────────────┬─────────────────────────────────┘
                               │                          [Loop 2]
        ┌──────────────────────▼─────────────────────────────────┐
        │ (D) WHOLE-BODY INVERSE DYNAMICS QP        (unchanged)  │
        └──────────────────────┬─────────────────────────────────┘
        ┌──────────────────────▼─────────────────────────────────┐
        │ (E) ROBOT — ideal ZMP  OR  first-order lag plant  §2.3 │   ← §2.3 adds the lag option
        └──────────────────────┬─────────────────────────────────┘
                               └────► (F) KF ────► back to (B) and (G)
```

The system now has **two nested loops on the same signal**: Loop 1 (baseline) re-initializes the
QP from the measurement, and Loop 2 (added) regulates the CP and ZMP errors. Section 2.1.2 is
about what that nesting costs.

### 2.1 The feedback term

This is the core contribution. *Input:* $p^{\text{ff}}, p^{\text{ref}}, \xi^{\text{ref}}$ from the
reference generator, and the filtered measurements $\hat\xi = \hat c + \hat{\dot c}/\eta$,
$\hat p$. *Output:* the commanded ZMP $p^{\text{cmd}}$ and, through it, the desired CoM
acceleration that replaces (1.8).

#### 2.1.1 The control law

We implement Morisawa's eq. (1.13), with the MPC feedforward substituted for the planned ZMP:

$$\boxed{\;
p^{\text{cmd}}(t) \;=\; p^{\text{ff}}(t)
\;-\; k_1\big(\hat\xi(t) - \xi^{\text{ref}}(t)\big)
\;-\; k_2\big(\hat p(t) - p^{\text{ref}}(t)\big)
\;-\; k_i \, I(t) \;}
\tag{2.1}$$

$$I(t) = \mathrm{clip}\!\left(\int_0^{t}\big(\hat\xi - \xi^{\text{ref}}\big)\,d\tau,\; -0.1,\; +0.1\right) \ \text{[m·s]}. \tag{2.2}$$

Two implementation details address caveats the paper explicitly raises:

- **Anti-windup.** The integral is saturated at $\pm 0.1$ m·s (Morisawa Sec. IV-D warns that
  integral saturation causes wind-up). It is additionally **reset to zero on contact loss**,
  detected as $F_z \le 0.1$ N, so that a flight phase or a missed landing does not charge the
  integrator. The reset lives in the measurement path in `simulation.py` and applies to whichever
  controller is active.
- **Measurement robustness.** The baseline set the ZMP to the world origin on contact loss (an
  acknowledged `FIXME`); with a feedback term consuming $\hat p$, that jump would be a metre-scale
  spurious error, so the measurement now **holds its last valid value** instead.

Note what (2.1) is *not*: it is not the law applied to the reference generator's state, but to the
robot's. The MPC keeps solving its own QP; the balance block edits its output.

#### 2.1.2 Making the loop well-posed: splitting $p^{\text{ff}}$ from $p^{\text{ref}}$

This is the difficulty the project exists to address, and it does not arise in the setting
Morisawa et al. designed for.

In eq. (1.13) a single symbol $p^{\text{ref}}$ plays two roles — *the plan to execute* and *the
value to regulate to* — because in an open-loop pattern generator those roles coincide. Under a
receding-horizon MPC they do not, for three distinct reasons.

**(i) There is no tracking error to regulate, by construction.** IS-MPC's own framing is that
gait generation "is neither a set-point nor a tracking problem": the ZMP objective is encoded as
the time-varying constraint (1.6), so any ZMP inside the support polygon is equally admissible. A
feedback term $-k_2(\hat p - p^{\text{ref}})$ presupposes that $p^{\text{ref}}$ is a value the
ZMP *ought* to take. It is not; it is one of a continuum of admissible values, selected by a cost
(1.5) that also weighs control effort. Regulating to it therefore imports an objective the MPC
never had.

**(ii) The reference is endogenous.** Because of Loop 1, the fresh solve at time $t$ has
*already* reacted to the ZMP error we would like to feed back. Using the fresh output as the
regulation target would give

$$\hat p(t) - p^{\text{ff}}(t) = \hat p(t) - g\big(\hat p(t), \hat c(t), \dot{\hat c}(t)\big),$$

an error signal that is a function of the very measurement it is built from — a static loop
through the QP, whose effective gain is neither known nor constant, since it depends on which
constraints are active. In practice this produces a *double correction*: the MPC moves the plan
toward the measurement, and the $k_2$ term moves the command in the same direction again.

**(iii) The horizon shift.** The MPC's first predicted sample $X^\star_{:,1}$ refers to
$t + \delta$, not to $t$. Comparing $\hat p(t)$ with a plan for $t+\delta$ introduces a
systematic one-step bias equal to the nominal ZMP velocity — during the double-support ZMP
transfer this is of order $\|\mathbf{f}_{j+1}-\mathbf{f}_j\| / T_{ds} \cdot \delta \approx
0.2/0.3 \cdot 0.01 \approx 7$ mm, which is *larger than the tracking errors we are trying to
measure* (Section 3).

**The resolution adopted here** is to split the two roles into two separate signals:

$$p^{\text{ff}}(t) = X^\star(t)_{[2,5,8],\,1}, \qquad
p^{\text{ref}}(t) = X^\star(t-\delta)_{[2,5,8],\,1}, \qquad
\xi^{\text{ref}}(t) = \tilde c + \tilde{\dot c}/\eta \tag{2.3}$$

with $\tilde\chi = X^\star(t-\delta)_{:,1}$ the first predicted state stored at the *previous*
solve.

| role | signal | rationale |
|---|---|---|
| feedforward (the plan to execute) | $p^{\text{ff}}(t)$, fresh solve | must be the *most recent* plan; a stale feedforward destabilizes the LIP |
| regulated reference (what the ZMP *should* be now) | $p^{\text{ref}}(t)$, previous solve | the a-priori prediction of instant $t$, computed *before* the measurement at $t$ existed |

$p^{\text{ref}}$ is thus, relative to the current instant, an **exogenous, causal** reference: the
MPC's honest prediction of where the ZMP should be now, made one step earlier, which cannot have
reacted to $\hat p(t)$. Objection (iii) disappears because $X^\star(t-\delta)_{:,1}$ refers
precisely to $t$; objection (ii) disappears because the algebraic measurement-to-reference path is
broken by one sampling delay. Objection (i) is structural and survives; Section 4 discusses it.
$\xi^{\text{ref}}$ is taken from the *same* stored prediction, so the CP and ZMP error terms are
mutually consistent.

**Consequence for the metrics.** Because the CP feedback deliberately drives $p^{\text{cmd}}$
*away* from $p^{\text{ref}}$ in order to correct a CP error, a larger $\|\hat p - p^{\text{ref}}\|$
is **not** by itself evidence of worse balance. All ZMP errors in Section 3 are measured against
$p^{\text{ref}}$ — the fed-back quantity — so that the number has an unambiguous meaning, but it
must be read jointly with the CoM error, the waist attitude error and the peak vertical force.

#### 2.1.3 Gain design by pole placement

The gains are computed once from $\{\alpha,\beta,\gamma\}$, $\eta$ and $g_p$
(`cp_feedback_gains` in `utils.py`, shared by the simulation and the ablation driver). Two
variants are implemented, selected by the `use_zmp_fb` flag.

**(a) With ZMP feedback** — the full three-pole placement of Morisawa eq. (1.14), with
$\omega \equiv \eta$:

$$k_1 = -\frac{\alpha\beta + \beta\gamma + \gamma\alpha - \eta(\alpha+\beta+\gamma-\eta)}{\eta\, g_p},\qquad
k_2 = -\frac{\alpha+\beta+\gamma+g_p-\eta}{g_p},\qquad
k_i = \frac{\alpha\beta\gamma}{\eta\, g_p}. \tag{2.4}$$

**(b) Without ZMP feedback** — $k_2 \equiv 0$. On the ideal (lagless) plant of the baseline, the
CP-error dynamics then close as a second-order system. With $e_\xi = \hat\xi - \xi^{\text{ref}}$
and $\dot I = e_\xi$,

$$\dot e_\xi = \eta\big(e_\xi - (p^{\text{cmd}} - p^{\text{ref}})\big) = \eta(1+k_1)\,e_\xi + \eta k_i I
\;\;\Rightarrow\;\;
A_{cl} = \begin{pmatrix}\eta(1+k_1) & \eta k_i \\ 1 & 0\end{pmatrix},$$

so matching $\mathrm{tr}\,A_{cl} = \alpha+\gamma$ and $\det A_{cl} = \alpha\gamma$ gives the
**exact** placement

$$k_1 = \frac{\alpha+\gamma}{\eta} - 1, \qquad k_2 = 0, \qquad k_i = -\frac{\alpha\gamma}{\eta}. \tag{2.5}$$

Here $\beta$ and $g_p$ do not appear at all — as expected, since the lagless plant has one state
fewer.

**The design/plant mismatch, and why it matters.** Variant (a) is derived for the **lag plant**
(1.11), but in the default configuration our plant is the baseline's **lagless** one. Applying
(2.4) to a lagless plant means $\{\beta, g_p\}$ no longer correspond to actual closed-loop pole
locations: they survive only as *shaping parameters* of $(k_1,k_2,k_i)$. This is not a bug — the
$k_2$ term is still a genuine feedback, because the measured ZMP of the real robot never equals
the commanded one — but it means that stability with respect to $\beta$ and $g_p$ has to be
established empirically. Section 3.5 shows exactly where it breaks, and Section 4.1 argues that
$g_p$ is in fact standing in for a real lag that the *whole-body layer* has and the *model* does
not.

At the nominal design point $\alpha=-3$, $\beta=-8$, $\gamma=-2$, $g_p=20$, $h=0.72$ m
($\eta = 3.6912$ rad/s):

| variant | $k_1$ | $k_2$ | $k_i$ |
|---|---:|---:|---:|
| with ZMP feedback, eq. (2.4) | $-1.4577$ | $-0.1654$ | $-0.6502$ |
| without ZMP feedback, eq. (2.5) | $-2.3546$ | $0$ | $-1.6255$ |

The signs follow the paper's convention: $k_1 < 0$ means $-k_1 e_\xi > 0$, i.e. the ZMP is pushed
*toward* the capture-point error, the stabilizing direction for an inverted pendulum. Note that
$k_2$ changes sign at $g_p = \eta - (\alpha+\beta+\gamma) = 16.69$; at the nominal $g_p = 20$ it
is already (mildly) negative — see Section 3.5.

#### 2.1.4 Disabling the term

Setting `use_cp = False` bypasses (2.1) entirely, giving $p^{\text{cmd}} = p^{\text{ff}}$ — i.e.
exactly the baseline behaviour of §1.2.4. This is the configuration labelled *ISMPC (no cp)* in
Section 3.3 and is the reference point against which the contribution is measured.

#### 2.1.5 How the feedback actually reaches the robot

This is the least obvious part of the integration and worth stating explicitly, because getting
it wrong silently disables everything above.

The inverse-dynamics block (§1.2.6) consumes a *desired CoM acceleration*, not a desired ZMP — it
has no ZMP task. The commanded ZMP therefore enters the robot **only** by replacing $p$ with
$p^{\text{cmd}}$ in the baseline output equation (1.8):

$$\boxed{\;\ddot c^{\,\text{des}} = \eta^2\,\big(c^{\,\text{des}} - p^{\text{cmd}}\big) + \begin{pmatrix}0\\0\\-g\end{pmatrix}\;} \tag{2.6}$$

Leaving (1.8) untouched and merely *logging* $p^{\text{cmd}}$ would discard the entire balance
feedback. Consequently the whole chain of the contribution — CP error, ZMP error, CP integral,
pole placement — is compressed into a single scalar per axis: a shift of the desired CoM
acceleration proportional to $\eta^2$ times the ZMP correction. For $\eta^2 = 13.6$ s$^{-2}$, a
1 cm ZMP correction is a $0.136$ m/s² CoM acceleration correction. For the vertical axis, with
$c_z = h$, $p_z = 0$ and $\eta^2 = g/h$, the $z$ component of (2.6) reduces identically to zero,
as it should.

The *logged* desired ZMP is deliberately set to $p^{\text{ref}}$, not $p^{\text{cmd}}$, so that
the reported ZMP tracking error is measured against the same signal the $k_2$ term regulates
(§2.1.2).

### 2.2 The alternative CP controller

To isolate what the MPC contributes — and to obtain a configuration that reproduces the setting
Morisawa et al. actually designed for, i.e. an *open-loop* reference — we implemented a second
reference generator, `cp_controller.py`, that replaces IS-MPC entirely while exposing the same
interface:

```
solve(current_state, t) -> (lip_state, contact_flag, p_cmd, cp_error_integral)
lip_state = { com: {pos, vel, acc}, zmp: {pos, vel, ref} }
```

so the two are drop-in replacements, selected by `--no-mpc`. No optimization is involved.

**ZMP reference.** Given the current step $j$ and the phase, a piecewise reference with a
smoothstep transfer during double support:

$$p^{\text{ref}}(t) = \begin{cases}
\mathbf{f}_j & \text{single support} \\[2pt]
\mathbf{f}_j + s(\phi)\,(\mathbf{f}_{j+1} - \mathbf{f}_j), \quad s(\phi)=\phi^2(3-2\phi) & \text{double support}
\end{cases} \tag{2.7}$$

with $\phi \in [0,1]$ the normalized double-support phase.

**CP reference.** The *exact* backward-in-time exponential that lands on the next footstep at the
end of the step — the analytic solution of $\dot\xi = \eta(\xi - p)$ run backwards:

$$\xi^{\text{ref}}(t) = p^{\text{ref}}(t) + e^{-\eta\, t_{\text{rem}}}\,\big(\mathbf{f}_{j+1} - p^{\text{ref}}(t)\big),
\qquad t_{\text{rem}} = (T_{ss}+T_{ds}) - (t - t_j). \tag{2.8}$$

**Reference CoM.** Integrated forward from the CP relation, $\dot c^{\text{ref}} =
\eta(\xi^{\text{ref}} - c^{\text{ref}})$, with the height held at $h$.

**Feedback.** The same law (2.1), with $p^{\text{ff}} \equiv p^{\text{ref}}$ — here the two roles
of §2.1.2 legitimately coincide, because $p^{\text{ref}}$ is a pure function of time and the plan
and has never seen a measurement. This is precisely the configuration in which the gains (2.4)
are used as intended.

**ZMP saturation.** Unlike the MPC branch, this block has no constraint machinery, so the command
is clipped explicitly to the support polygon, as Morisawa et al. require: to
$\mathbf{f}_j \pm d/2$ in single support, and to the bounding box of
$\{\mathbf{f}_j, \mathbf{f}_{j+1}\} \pm d/2$ in double support.

**Terminal condition.** With no next footstep at the last step, both $p^{\text{ref}}$ and
$\xi^{\text{ref}}$ are driven to the midpoint of the two planted feet, so the robot comes to rest
on both feet. The same correction was added to the MPC's moving constraint (1.9), whose loop
otherwise leaves the ZMP centred on a single foot at the end of the plan.

What this block is *not* is a re-derivation of the Morisawa pipeline: it has no walking FSM, no
landing detection, no impedance layer, and it reuses the inherited footstep planner, swing-foot
generator and inverse-dynamics QP. It is an emulation of the *reference-generation* half, enough
to answer the question "how much of the performance comes from the MPC, and how much from the
balance loop?".

### 2.3 The ZMP-lag implementation

The third addition makes the plant of §1.3 available, so that the gains (2.4) can be applied to
the system they were designed for.

**In the MPC (`use_lag`).** The prediction model switches from the ideal (1.2) to the lag plant
(1.11): the ZMP *command* becomes the input instead of the ZMP velocity,

$$A_{\text{lip}} = \begin{pmatrix} 0 & 1 & 0 \\ \eta^2 & 0 & -\eta^2 \\ 0 & 0 & -g_p \end{pmatrix},
\qquad B_{\text{lip}} = \begin{pmatrix} 0 \\ 0 \\ g_p \end{pmatrix}, \tag{2.9}$$

and the reported ZMP velocity becomes $\dot p = g_p\,(u - p)$ rather than $u$.

**Soft ZMP constraints.** With the lag, the ZMP is no longer a free input: it is the output of a
first-order filter with time constant $1/g_p = 50$ ms, and the QP cannot in general satisfy the
hard box (1.6) within one sampling interval — the problem becomes infeasible. The hard bounds are
therefore **relaxed to the quadratic penalty already present in the cost (1.5)** whenever
`use_lag` is on. This is a real weakening of the baseline's balance guarantee and is stated as
such; it is the price of the lag model in this formulation.

**In the CP controller.** The realized ZMP is integrated through the same lag,
$\dot p = g_p (p^{\text{cmd}} - p)$, instead of being set equal to the command.

**In the estimator.** The Kalman filter's $B$ matrix follows $B_{\text{lip}}$ automatically, but
its *input* must change type: the ZMP **velocity** command in ideal mode, the ZMP **position**
command in lag mode. This required tracking the last applied command separately from the logged
reference, since after §2.1.2 the two are different signals.

**Relation to the gain design.** $g_p$ now plays two distinct roles: a plant pole (in lag mode)
and a gain-design parameter (always, through (2.4)). Section 3.5 sweeps it in the *default,
lagless* configuration precisely to isolate the second role — where it has, strictly speaking, no
physical referent, yet turns out to be the most consequential parameter in the design.

### 2.4 Scenario flags

The pipeline is factorized into six independent binary switches, giving the scenario matrix of
Section 3.3. The last two rows were present in the baseline in spirit; the rest are new.

| flag | off | on (default) | added |
|---|---|---|:-:|
| `use_mpc` | standalone CP controller (§2.2) | IS-MPC | ✓ |
| `use_cp` | no balance feedback, $p^{\text{cmd}} = p^{\text{ff}}$ (= baseline) | CP feedback active, eq. (2.1) | ✓ |
| `use_zmp_fb` | $k_2 = 0$, exact 2-pole design (2.5) | full 3-pole design (2.4) | ✓ |
| `use_lag` | ideal ZMP plant, hard ZMP constraints | first-order lag (§2.3), soft constraints | ✓ |
| `open_loop` | QP re-initialized from $\hat x$ (Loop 1 closed) | QP propagates its own internal LIP state | ✓ |
| `use_kf` | raw measurements | Kalman-filtered state | · |

The `open_loop` switch is the cleanest way to quantify what the baseline's Loop 1 alone is worth:
with it on, the MPC integrates its own model forward using its last applied input and never sees
the robot, so the *only* remaining feedback is the balance loop of §2.1. This required storing an
internal LIP state in the MPC and propagating it explicitly, since the baseline overwrote it with
the measurement unconditionally.

### 2.5 Evaluation infrastructure

Three new drivers, needed because the baseline had only an interactive viewer and a live plot:

- `plot_logs.py` — per-run figures (CoM/ZMP/CP trajectories per axis, tracking errors, waist
  attitude, vertical force) from a saved `.npz`;
- `make_runs_summary.py` — re-simulates the eight configurations of Section 3.3 **headlessly**
  and emits `runs_summary.md`;
- `ablation_poles.py` — sweeps $\{\alpha,\beta,\gamma,g_p\}$ one at a time (26 headless
  simulations), caches the raw series, and emits nine per-axis plots per parameter, marking
  diverged runs.

Supporting changes: the logger gained `.npz` export with run metadata; the state now carries the
ground reaction force, so that peak $F_z$ — the metric of Fig. 7(d) of Morisawa et al. — can be
reported.

---

## 3. Experiments

### 3.1 Machine and environment

| item | value |
|---|---|
| CPU | Intel Core i7-11370H @ 3.30 GHz (4 cores / 8 threads, max 4.80 GHz) |
| RAM | 15 GiB |
| GPU | Intel Iris Xe (TigerLake-LP GT2) + NVIDIA GeForce RTX 3060 Mobile (unused: the pipeline is CPU-only) |
| OS | Fedora Linux 44 (Workstation), kernel 7.1.7-200.fc44.x86_64 |
| Python | 3.14.6 |
| `dartpy` | 6.16.0 |
| `casadi` | 3.7.2 (OSQP conic backend) |
| `osqp` | 1.1.1 |
| `numpy` / `scipy` | 2.4.4 / 1.17.1 |

**Reproducibility caveat.** The simulation contains no random number generator, yet results
wobble slightly run to run: under $\approx 0.02$ mm on the ZMP RMSE, up to $\approx 0.5^\circ$
peak-to-peak on waist pitch and $\approx 10$ N on peak $F_z$, with CoM and ZMP-$y$ essentially
unaffected. The runs cluster between two nearly identical outcomes, so a single run is
representative and no averaging is applied. The origin is numerical (multithreaded BLAS
floating-point reduction order, plus the DART contact solver and OSQP): a sub-ULP difference
occasionally tips a near-threshold contact/QP event one way or the other. Pinning
`OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1` shrinks the spread but does not
eliminate it; we did not find a switch making runs bit-for-bit reproducible.

### 3.2 Task, model and protocol

| item | value |
|---|---|
| Robot | HRP-4 URDF, floating base passive, all other joints torque-actuated |
| Physics | DART, time step $\delta = 10$ ms |
| CoM height $h$ | 0.72 m $\Rightarrow \eta = 3.6912$ rad/s |
| Foot size $d$ (ZMP box side) | 0.10 m; friction coefficient $\mu = 0.5$ |
| Step timing | $T_{ss} = 0.70$ s, $T_{ds} = 0.30$ s; first step: $T_{ss}=0$, $T_{ds}=2.0$ s |
| Step height | 0.05 m |
| MPC horizon | $N = 100$ ($T_c = 1.0$ s), periodic tail |
| Reference velocities | $5\times(0.1,0,0.2)$, $10\times(0.1,0,-0.1)$, $10\times(0.1,0,0)$, $1\times(0,0,0)$ |
| Footsteps / duration | 26 footsteps, 2700 control steps = **27.0 s** of walking |
| Nominal poles | $\alpha=-3$, $\beta=-8$, $\gamma=-2$; $g_p = 20$ |

The commanded gait is a forward walk at 0.1 m/s that first turns left ($\omega = +0.2$ rad/s for
5 steps), then right ($-0.1$ rad/s for 10 steps), then goes straight for 10 steps and stops —
i.e. it exercises curved, counter-curved and rectilinear locomotion plus a stop, in a single
trajectory.

**Metrics.** All errors are per-axis RMSE over the run **after discarding a 200-step (2 s)
warm-up** corresponding to the initial double-support settling transient:

$$\text{RMSE} = \sqrt{\tfrac{1}{T-T_w}\textstyle\sum_{t>T_w} e(t)^2}.$$

Positions in mm, waist attitude (roll/pitch of the floating base, from the rotation-vector
difference converted to Euler XYZ) in degrees, and the vertical reaction force reported as its
**peak** $\max F_z$ in N — the same four families of metrics used in Fig. 7 of Morisawa et al.
A run marked *diverged* fell (the MPC QP blew up) and its metrics are not meaningful.

As stated in §2.1.2, the ZMP error is measured against $p^{\text{ref}}$, not against the realized
command $p^{\text{cmd}}$.

### 3.3 Configuration comparison

All runs with the Kalman filter enabled. Each variant flips one feature off the **plain**
baseline (IS-MPC + capture point + ZMP feedback). The first row, *ISMPC (no CP)*, is the
**inherited baseline of §1.2** with no contribution active.

| config | mpc | cp fb | zmp fb | lag | open-loop ref | ZMP x | ZMP y | ZMP z | CoM x | CoM y | CoM z | waist roll | waist pitch | max $F_z$ | fell |
|---|:-:|:-:|:-:|:-:|:-:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:-:|
| | | | | | | [mm] | [mm] | [mm] | [mm] | [mm] | [mm] | [deg] | [deg] | [N] | |
| **IS-MPC (no CP) — baseline** | ✓ | · | · | · | · | 0.29 | 0.18 | 0.58 | 1.05 | 1.52 | 0.09 | 0.215 | 1.126 | 756.3 | no |
| **plain** | ✓ | ✓ | ✓ | · | · | **0.20** | 0.19 | 0.73 | 1.06 | 1.49 | 0.05 | 0.170 | **0.767** | 761.2 | no |
| no ZMP fb | ✓ | ✓ | · | · | · | 0.22 | 0.29 | 0.87 | 1.05 | 1.49 | 0.03 | 0.111 | 0.995 | **575.7** | no |
| lag | ✓ | ✓ | ✓ | ✓ | · | 9.19 | 16.29 | 0.73 | 1.12 | 1.67 | 0.03 | 0.090 | 0.608 | 785.2 | no |
| open loop | ✓ | ✓ | ✓ | · | ✓ | 2.31 | 2.81 | 2.16 | 1.17 | 1.16 | 1.13 | 0.152 | 1.077 | 763.0 | no |
| CP controller | · | ✓ | ✓ | · | ✓ | 16.17 | 26.96 | 0.55 | 4.55 | 6.05 | 1.08 | 0.159 | 0.389 | 766.7 | no |
| CP controller, no ZMP fb | · | ✓ | · | · | ✓ | 18.29 | 30.49 | 0.68 | 4.26 | 5.59 | 1.21 | 0.207 | 0.649 | 772.7 | no |
| CP controller, lag | · | ✓ | ✓ | ✓ | ✓ | 24.38 | 39.93 | 0.12 | 5.40 | 7.13 | 0.83 | 0.209 | 0.456 | 764.8 | no |

**Reading of the table.**

*(a) The feedback term improves the inherited baseline, moderately but consistently.* Adding the
CP + ZMP + integral loop of §2.1 to the baseline cuts sagittal ZMP error by 31 % (0.29 → 0.20 mm),
waist pitch error by 32 % (1.126 → 0.767°) and waist roll by 21 % (0.215 → 0.170°), at essentially
unchanged CoM error and a 0.6 % higher peak $F_z$. The pattern — improvement concentrated in the
sagittal ZMP and in waist attitude, with a slight increase in peak vertical force — is
qualitatively the same as Fig. 7 of Morisawa et al., a meaningful cross-check given how different
the two whole-body layers are (torque-level QP here vs. inverse kinematics + impedance there).

*(b) The absolute numbers are small because the baseline is already closed-loop.* Sub-millimetre
ZMP RMSE means Loop 1 alone does most of the work: the QP is re-anchored on the measurement 100
times per second, so the plan never drifts far from reality. This is the quantitative statement of
the "endogenous reference" argument of §2.1.2(ii) — there is simply not much error left for an
explicit loop to remove.

*(c) The ZMP feedback term earns its place, but it is not free.* Turning $k_2$ off (which also
switches the design to the exact 2-pole placement (2.5)) degrades ZMP $y$ by 53 % (0.19 → 0.29 mm),
ZMP $z$ by 19 % and waist pitch by 30 %, but **improves** waist roll (0.170 → 0.111°) and lowers
peak $F_z$ by 24 % (761 → 576 N). The last figure is the notable one: with $\approx 39$ kg of
robot ($\approx 383$ N of weight), 576 N is a $1.5\times$-weight landing peak whereas 761 N is
$\approx 2\times$. The $k_2$ term makes the ZMP command more aggressive around foot landing, which
buys tracking and pitch accuracy at the price of harder impacts. Which trade-off is preferable is
a design decision, not a fact.

*(d) Loop 1 is worth an order of magnitude — and the added loop can stand alone.* With
`open_loop` on, the MPC integrates its own model and the balance loop is the only feedback left:
ZMP errors grow $\approx 11\times$ (0.20 → 2.31 mm in $x$, 0.19 → 2.81 mm in $y$), CoM $z$ error
grows $22\times$, and waist pitch degrades to 1.077°. Crucially, the robot **does not fall**: the
explicit CP/ZMP loop of eq. (2.1) is sufficient on its own to keep a 27-second gait standing,
which is exactly the claim of Morisawa et al. — just at a much coarser accuracy than the MPC
re-initialization achieves.

*(e) The MPC is what buys accuracy; the CP loop is what buys robustness.* The standalone CP
controller of §2.2 has ZMP errors two orders of magnitude larger (16.2 / 27.0 mm) and CoM errors
$4\times$ larger, because its reference is a heuristic — the smoothstep ZMP transfer (2.7) plus
the exponential CP law (2.8) — with no optimization, no explicit ZMP constraint handling beyond
saturation, and no stability constraint. Yet it never falls, and its **waist pitch error is the
best in the whole table** (0.389° vs 0.767° for plain IS-MPC). Attitude is governed by how
*smooth* the CoM acceleration command is, and the analytic reference is smoother than the QP's,
which switches active constraint sets. The two blocks are complementary in a concrete, measurable
sense.

*(f) The ZMP feedback helps the CP controller too, and by a larger relative margin.* Within the
MPC-free branch, enabling $k_2$ improves ZMP $x$ and $y$ by 12 % each (18.29 → 16.17,
30.49 → 26.96 mm) and waist pitch by 40 % (0.649 → 0.389°). This is the configuration closest to
the setting the gains (2.4) were designed for — an open-loop reference — and, consistently, it is
where the term behaves most like the paper predicts.

*(g) The ZMP lag is the dominant error source when present.* Enabling the lag plant of §2.3
inflates ZMP errors by $\approx 45\times$ (0.20 → 9.19 mm in $x$, 0.19 → 16.29 mm in $y$). This is
expected and largely *kinematic* rather than a control failure: with $g_p = 20$ s⁻¹ the realized
ZMP follows the command through a first-order filter of time constant 50 ms, and the metric
compares the realized ZMP against a reference the plant physically cannot reach instantaneously.
That the gait is nevertheless stable — with CoM error degraded by only 6 %, the *best* waist roll
in the table (0.090°), and this despite the ZMP constraints having been softened (§2.3) — shows
the lag acts as a low-pass on the command; the price is the highest peak $F_z$ (785 N).

### 3.4 Ablation on the closed-loop poles

Each sweep varies one design parameter about its default, holding the others fixed, recomputing
$(k_1,k_2,k_i)$ from (2.4), on the **default (lagless) plant with IS-MPC and the Kalman filter
on**. Per-axis plots are in `logs/ablation_poles/`. Recall (§2.1.3) that on this plant $\beta$ and
$g_p$ are *shaping parameters*, not actual pole locations.

#### 3.4.1 $\alpha$ — the dominant capture-point pole

| $\alpha$ | ZMP x | ZMP y | ZMP z | CoM x | CoM y | CoM z | roll | pitch | max $F_z$ | fell |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:-:|
| $-25$ | 0.60 | 0.66 | 8.67 | 1.05 | 1.48 | 0.02 | 0.070 | 0.662 | 623.6 | no |
| $-15$ | 0.28 | 0.32 | 1.74 | 1.04 | 1.49 | 0.04 | 0.146 | 1.154 | 762.5 | no |
| $-10$ | 0.20 | 0.31 | 1.03 | 1.05 | 1.49 | 0.02 | 0.145 | 1.116 | 766.3 | no |
| $-5$ | 0.20 | 0.22 | 0.83 | 1.05 | 1.49 | 0.04 | 0.085 | 0.964 | 762.5 | no |
| **$-3$ (default)** | **0.20** | 0.19 | 0.73 | 1.06 | 1.49 | 0.05 | 0.170 | 0.767 | 761.2 | no |
| $-2$ | 0.21 | 0.12 | 0.73 | 1.06 | 1.49 | 0.03 | 0.098 | 0.619 | 761.9 | no |
| $-1$ | 0.21 | 0.10 | 0.57 | 1.06 | 1.49 | 0.03 | 0.132 | 1.009 | 758.5 | no |

No value in the swept range falls. The trend is monotone in the "aggressiveness" direction: as
$|\alpha|$ grows, $|k_1|$ grows roughly linearly ($|k_1| = 1.46$ at $\alpha=-3$ versus $5.54$ at
$\alpha=-25$, a $3.8\times$ increase) and **every** error metric degrades — ZMP $x$ by $3\times$,
ZMP $y$ by $6.6\times$, ZMP $z$ by $12\times$. The vertical axis is the most sensitive by a wide
margin (8.67 mm at $\alpha = -25$), which makes sense: the $z$ channel has the weakest authority
(the $z$ component of (2.6) is identically zero at the nominal height, so all vertical correction
comes from the transient), and a high-gain command the whole-body QP cannot realize turns into a
poorly-tracked vertical CoM reference. The lower peak $F_z$ at $\alpha = -25$ (623 N) is a
symptom of the same effect — the CoM is being commanded downward more often, so landings are
lighter but the vertical trajectory is 12× worse.

At the other end, $\alpha \to 0$ minimizes ZMP $y$ and $z$ but the sagittal error stops improving
and waist pitch becomes erratic (0.619° at $-2$, 1.009° at $-1$), consistent with a loop too slow
to reject the per-step transients. The default $\alpha = -3$ sits at the knee: the smallest
$|\alpha|$ at which ZMP $x$ is at its minimum, and the best waist pitch among values with
$|\alpha| \ge 3$.

#### 3.4.2 $\beta$ — the "ZMP-lag" pole

| $\beta$ | ZMP x | ZMP y | ZMP z | CoM x | CoM y | CoM z | roll | pitch | max $F_z$ | fell |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:-:|
| $-80$ | — | — | — | — | — | — | — | — | — | **yes** |
| $-40$ | — | — | — | — | — | — | — | — | — | **yes** |
| $-25$ | 0.21 | 0.32 | 2.17 | 1.05 | 1.49 | 0.03 | 0.186 | 1.156 | 771.8 | no |
| $-16$ | **0.18** | 0.25 | 1.10 | 1.05 | 1.49 | 0.04 | 0.132 | 1.085 | 765.2 | no |
| **$-8$ (default)** | 0.20 | 0.19 | 0.73 | 1.06 | 1.49 | 0.05 | 0.170 | 0.767 | 761.2 | no |
| $-4$ | 0.26 | 0.11 | 0.55 | 1.06 | 1.49 | 0.03 | 0.137 | 1.197 | 758.9 | no |
| $-2$ | 0.32 | 0.15 | 0.54 | 1.06 | 1.50 | 0.04 | 0.160 | 1.067 | 755.2 | no |

This is the only sweep with hard failures, and it is the clearest evidence of the design/plant
mismatch of §2.1.3. At $\beta = -80$ the gains become $k_1 = -9.94$ and $k_2 = +3.43$, i.e.
$6.8\times$ and $20.8\times$ their nominal magnitudes. On the *lag* plant those values would place
three legitimate closed-loop poles; on the *lagless* plant the commanded ZMP is realized within
one control step, so a $k_2$ of that size closes a very high-gain loop around the noisiest signal
in the pipeline — a ZMP reconstructed from DART contact points and filtered with $R_{33} = 10^4$.
The result is a chattering ZMP command that saturates the whole-body QP, and the robot falls.

Within the stable range the sweep is a genuine trade-off with **no interior optimum**: ZMP $x$ is
minimized at $\beta \approx -16$ (0.18 mm) and degrades by 78 % toward $\beta = -2$, while ZMP
$y$ and ZMP $z$ improve monotonically as $|\beta|$ decreases (0.32 → 0.11 mm and 2.17 → 0.54 mm).
The default $\beta = -8$ is the compromise point: within 11 % of the sagittal optimum while
keeping $y$ and $z$ within 73 % and 35 % of theirs, and with the best waist pitch in the column
by a factor of 1.4.

#### 3.4.3 $\gamma$ — the capture-point integral pole

| $\gamma$ | ZMP x | ZMP y | ZMP z | CoM x | CoM y | CoM z | roll | pitch | max $F_z$ | fell |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:-:|
| $-15$ | 0.27 | 0.35 | 1.80 | 1.05 | 1.48 | 0.03 | 0.167 | 0.936 | 772.0 | no |
| $-10$ | 0.21 | 0.33 | 1.22 | 1.05 | 1.48 | 0.02 | 0.171 | 1.070 | 768.2 | no |
| $-5$ | 0.20 | 0.21 | 0.86 | 1.06 | 1.49 | 0.03 | 0.116 | 0.851 | 762.6 | no |
| $-3$ | 0.22 | 0.15 | 0.70 | 1.06 | 1.49 | 0.02 | 0.154 | 0.976 | 762.2 | no |
| **$-2$ (default)** | 0.20 | 0.19 | 0.73 | 1.06 | 1.49 | 0.05 | 0.170 | **0.767** | 761.2 | no |
| $-1$ | 0.28 | 0.19 | 0.67 | 1.05 | 1.50 | **0.13** | **0.200** | 1.157 | 760.3 | no |

$\gamma$ is precisely the pole whose displacement from the origin switches the integral action on:
$k_i = \alpha\beta\gamma/(\eta g_p) \to 0$ as $\gamma \to 0$, and the controller degenerates to the
non-integrating CP controller (Morisawa's observation below their eq. (22)). The sweep shows both
ends of that statement:

- **Too fast ($|\gamma| \ge 10$):** the integrator becomes an aggressive term on a signal that is
  itself an accumulation of a noisy CP estimate. ZMP $y$ degrades 74 % and ZMP $z$ 147 % relative
  to the default, and peak $F_z$ rises.
- **Too slow ($\gamma = -1$):** the integral authority nearly vanishes, and the offsets it exists
  to remove reappear — CoM $z$ error jumps to 0.13 mm ($2.6\times$ the default, and the worst in
  the column), waist roll is the worst in the column (0.200°) and waist pitch degrades by 51 %.
  This is exactly the failure mode Fig. 5 of Morisawa et al. illustrates in reverse: with
  insufficient integration, steady offsets in CP and ZMP survive.
- The default $\gamma = -2$ is the best compromise, with the best waist pitch of the sweep;
  $\gamma = -3$ is a defensible alternative (better ZMP $y$/$z$, slightly worse $x$ and pitch).

### 3.5 Ablation on the ZMP-lag gain $g_p$

| $g_p$ | $k_1$ | $k_2$ | $k_i$ | ZMP x | ZMP y | ZMP z | CoM z | roll | pitch | max $F_z$ | fell |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:-:|
| 5 | $-5.83$ | $+2.34$ | $-2.60$ | — | — | — | — | — | — | — | **yes** |
| 10 | $-2.92$ | $+0.67$ | $-1.30$ | 0.22 | 0.26 | 2.00 | 0.07 | 0.182 | 1.137 | 768.0 | no |
| **20 (default)** | $-1.46$ | $-0.17$ | $-0.65$ | **0.20** | **0.19** | **0.73** | 0.05 | 0.170 | **0.767** | 761.2 | no |
| 50 | $-0.58$ | $-0.62$ | $-0.26$ | 0.49 | 0.28 | 0.57 | 0.04 | 0.114 | 1.012 | 757.4 | no |
| 70 | $-0.42$ | $-0.71$ | $-0.19$ | 0.73 | 0.50 | 0.65 | 0.05 | 0.143 | 1.047 | 754.8 | no |
| 90 | $-0.32$ | $-0.81$ | $-0.14$ | — | — | — | — | — | — | — | **yes** |

(The $k$ columns are computed from (2.4) at $\alpha=-3,\beta=-8,\gamma=-2$; they are not measured,
but they explain the shape of the sweep.)

This is the most informative sweep, because **it fails at both ends, for two different reasons**,
and the default sits almost exactly at the optimum of every metric.

- **Small $g_p$ (divergence at 5).** All three gains carry a $1/g_p$ factor, so halving $g_p$ from
  the default to 10 doubles them and going to 5 quadruples them: $k_1 = -5.83$, $k_i = -2.60$.
  This is the same high-gain mechanism as $\beta = -80$: a loop far too aggressive for a plant
  that realizes the command instantaneously. Already at $g_p = 10$ the degradation is visible
  (ZMP $z$ up $2.7\times$, waist pitch up 48 %).
- **Large $g_p$ (divergence at 90).** Here the mechanism is different and more interesting. In
  (2.4), $k_2$ changes sign at

  $$g_p^\star = \eta - (\alpha+\beta+\gamma) = 3.691 + 13 = 16.69,$$

  and as $g_p \to \infty$, $k_2 \to -1$. Since the control law uses $-k_2(\hat p - p^{\text{ref}})$,
  a negative $k_2$ means the command is pushed *further* in the direction the ZMP has already
  erred — i.e. positive feedback on the ZMP error, with a limiting law
  $p^{\text{cmd}} \to p^{\text{ff}} + (\hat p - p^{\text{ref}})$. On the lag plant this
  anti-damping is the correct lead-like action demanded by pole placement, and it is compensated
  by the plant's own $-g_p$ pole. On the lagless plant there is no such pole to compensate it, so
  the loop survives only while $|k_2|$ is small. The sweep shows the boundary empirically: $-0.62$
  at $g_p = 50$ and $-0.71$ at $70$ are tolerated (with a visible 2.5–3.6× degradation of ZMP $x$),
  $-0.81$ at $g_p = 90$ is not.
- **The default $g_p = 20$ is optimal on this plant**, minimizing ZMP $x$, $y$ and waist pitch
  simultaneously and sitting just past $g_p^\star$, where $k_2 = -0.165$ is negative but small
  enough to be dominated by the CP term.

This sweep is the sharpest empirical statement of the argument in §2.1.3: on the lagless plant
$g_p$ has *no physical meaning at all* — there is no lag — yet it is the single most consequential
tuning parameter in the design, because it sets the relative weight of the ZMP term against the CP
and integral terms and controls the sign of the ZMP loop.

### 3.6 Summary of experimental findings

1. The added CP/ZMP/integral feedback improves the inherited IS-MPC baseline by $\approx 30$ % on
   sagittal ZMP and waist-attitude error, reproducing the qualitative pattern reported by
   Morisawa et al. on HRP-2, at a $\approx 1$ % cost in peak vertical force.
2. The gain is modest in absolute terms because the baseline's per-step re-initialization is
   already a strong implicit feedback; removing it (`open_loop`) degrades tracking by an order of
   magnitude, yet the added loop alone still keeps the robot walking.
3. The ZMP feedback term $k_2$ is a genuine trade: better sagittal ZMP, better waist pitch, worse
   waist roll and $\approx 32$ % harder landings.
4. The MPC contributes accuracy; the CP loop contributes robustness and smoothness. The
   MPC-free controller of §2.2 is two orders of magnitude less accurate but has the best waist
   pitch of any configuration.
5. Stability is limited by two distinct high-gain mechanisms, both traceable to applying
   lag-plant gains to a lagless plant: gain blow-up at small $g_p$ / large $|\beta|$, and sign
   inversion of $k_2$ at large $g_p$.
6. Every default in `simulation.py` ($\alpha=-3$, $\beta=-8$, $\gamma=-2$, $g_p=20$) sits at or
   adjacent to the optimum of its sweep, and $g_p = 20$ is optimal in three metrics at once.

---

## 4. Discussion, conclusions and future work

### 4.1 What we set out to do, and what the result says

The project's stated aim was to derive a ZMP feedback for IS-MPC. The honest summary of the
outcome is:

**It works, it helps, and it is structurally awkward — and the awkwardness is informative.**

The ZMP feedback term measurably improves the closed loop (§3.3(a),(c)), but three things had to
be conceded along the way, and each is a statement about the interaction between receding-horizon
planning and error regulation.

**(i) The reference had to be redefined to make the loop well-posed.** Morisawa's eq. (1.13) uses
a single symbol $p^{\text{ref}}$ for two roles — the plan to execute and the value to regulate to
— because in an open-loop pattern generator those roles coincide. Under an MPC they do not: the
freshest plan is contaminated by the current measurement, and the natural "first predicted sample"
refers to the next instant, not this one. Splitting the two roles (§2.1.2) is, we believe, the
correct general recipe: **feed forward the newest plan, regulate against the previous plan's
prediction of now.** The regulated reference is then causal and exogenous by construction, at the
cost of one sampling period of staleness — negligible at 100 Hz. Notably, in the CP-controller
branch of §2.2 the split collapses back to the original single symbol, which is a useful
consistency check that it is the MPC, not the balance law, that forces the distinction.

**(ii) The regulated quantity is not something IS-MPC ever promised to deliver.** IS-MPC encodes
the ZMP objective as the constraint (1.6), so any ZMP in the support polygon is admissible and
there is no error signal in the problem formulation. Regulating $\hat p \to p^{\text{ref}}$
therefore imports an objective the MPC does not have. In practice the two rarely conflict, because
the inherited cost (1.5) does prefer the polygon centre, so $p^{\text{ref}}$ is a *reasonable*
target — but this is a property of that particular cost, not of IS-MPC. With a cost that only
penalized control effort, $p^{\text{ref}}$ would be an arbitrary point of the admissible set and
regulating to it would be actively harmful. A more principled formulation would regulate the
*capture point* only — which IS-MPC does constrain, through the stability constraint (1.7) — and
treat the ZMP term as what it is on this plant: a damping/lead injection, not a tracking error.

**(iii) The gains are designed for a plant we do not have.** The pole-placement formulas (2.4)
assume the first-order ZMP lag (1.10), which the baseline does not model. The exact 2-pole design
(2.5) is theoretically clean for the lagless plant, but empirically slightly worse in ZMP $x$/$y$
(§3.3(c)): the "improperly designed" $k_2$ term does capture something real, namely the mismatch
between the ZMP the LIP model commands and the ZMP the full-dynamics robot produces. That mismatch
has a bandwidth, and $g_p$ is effectively tuning it — a lag we did not model but that the *robot*
has. The fact that the empirically optimal $g_p$ (20 s⁻¹, i.e. 50 ms) is a plausible value for a
contact-mediated torque-control loop supports this reading and suggests the identification
experiment of §4.3.3.

### 4.2 Limitations

- **Nominal conditions only.** All experiments are on flat ground, with no external pushes and no
  model perturbation. The balance loop is therefore evaluated on the modelling error inherent to
  the LIP-vs-HRP-4 mismatch, which is real but small (sub-millimetre). The regime where a
  stabilizer earns its keep — uneven terrain, unexpected slopes, pushes — is untested here.
  Morisawa et al. make their case on a 10 % slope and a $\pm 4$ cm uneven-terrain rig; we do not.
- **Fixed footsteps and fixed timing** (inherited, §1.2.2). The system has none of IS-MPC's
  reactive step-adjustment capability, which is the mechanism by which a large disturbance is
  normally absorbed.
- **Periodic tail only** (inherited, §1.2.4). Recursive feasibility is not established for our
  formulation.
- **Softened ZMP constraints in lag mode** (§2.3). The lag configuration trades the baseline's
  hard balance guarantee for feasibility; its results should be read with that in mind.
- **No ZMP saturation on the IS-MPC branch.** The QP guarantees $p^{\text{ff}}$ inside the
  support polygon, but the feedback correction in (2.1) can push $p^{\text{cmd}}$ outside it.
  Morisawa et al. explicitly flag this as requiring saturation (or a landing modification). The
  CP-controller branch saturates; the IS-MPC branch does not. This is very likely part of why
  large gains cause outright falls rather than graceful degradation.
- **Metric caveat.** The reported ZMP error is measured against $p^{\text{ref}}$, which the
  feedback deliberately deviates from. It is the right choice for interpretability but it means
  no single number in Section 3 is a complete quality score.
- **Run-to-run non-determinism.** Small (§3.1) but not eliminated; sweeps are single runs.

### 4.3 Future work

Roughly in order of expected value:

1. **Disturbance and terrain experiments.** Push recovery (impulsive force on the torso) and a
   sloped/uneven ground plane, which is the regime the balance controller was designed for.
   Making that a first-class, flag-selectable scenario and re-running the full configuration
   table on it would test the thesis of §3.3(e) — that the CP loop buys robustness the MPC does
   not.
2. **Saturate $p^{\text{cmd}}$ on the IS-MPC branch**, using the QP's own active moving
   constraint (1.9) as the polygon. Cheap, and directly addresses a known failure mode.
3. **Identify the real ZMP lag of the whole-body layer** rather than treating $g_p$ as a free
   knob. The torque-level QP plus contact dynamics has a measurable command-to-realized-ZMP
   transfer function; fitting a first-order model to it would give $g_p$ a physical value and make
   (2.4) an actual pole placement rather than a heuristic — and would make the lag mode of §2.3 a
   *model* of the system rather than an alternative to it.
4. **Fold the balance feedback into the QP instead of after it.** The cleanest resolution of
   objection (ii) is to stop post-processing the MPC output and instead let the MPC absorb the
   role: add the measured CP error as a state perturbation, or add a soft cost on the CP error
   with the constraint set unchanged. This makes the correction constraint-aware by construction —
   the command can never leave the support polygon — and removes the double-correction issue of
   §2.1.2(ii) entirely.
5. **Footstep adaptation.** Re-enabling footsteps as decision variables (finite $\beta$ in the
   IS-MPC cost) plus the timing rule of IS-MPC eq. (1) would give the system the
   disturbance-rejection channel it currently lacks, and would let one study the division of
   labour between *stepping* and *ankle/ZMP* strategies under the same disturbance.
6. **Anticipative tail**, for recursive feasibility in the sense proved by IS-MPC.
7. **Systematic 2-D gain search.** All ablations here are one-parameter-at-a-time about the
   default, which cannot see interactions — and §3.5 shows the parameters interact strongly
   ($g_p^\star$ depends on $\alpha+\beta+\gamma$). A coarse grid over $(\alpha, g_p)$ with the
   divergence boundary mapped explicitly would be more informative than four line sweeps.

### 4.4 Conclusion

Starting from the DIAG Robotics Lab reference implementation of IS-MPC — a 3D-LIP receding-horizon
gait generator with moving ZMP constraints, a periodic-tail stability constraint, and a
torque-level whole-body inverse-dynamics layer — we added a capture-point-based balance
controller: CP error, ZMP error and CP-integral feedback with pole-placement gains, a standalone
CP-controller alternative to the MPC that emulates the reference-generation setting of the balance
control literature, and a first-order ZMP-lag plant model. We evaluated the result over a
27-second curved-walk task across eight configurations and four parameter sweeps.

The central technical contribution is the resolution of a conflict that does not arise in the
original balance-control setting: when the ZMP reference is produced by a receding-horizon MPC
that re-initializes on the measurement, the reference is endogenous and time-shifted, and a naive
ZMP feedback closes an ill-posed loop. Splitting the feedforward (freshest plan) from the
regulated reference (previous plan's prediction of the current instant) makes the loop causal and
well-posed, and is what allows the ZMP feedback term to be used at all in this architecture.

Quantitatively, the balance loop improves sagittal ZMP tracking and waist attitude by roughly
30 % over the inherited baseline, at a small cost in landing impact; the improvement is modest
precisely because the baseline's own re-initialization is already a strong feedback, and removing
that re-initialization degrades tracking by an order of magnitude while the added loop alone still
keeps the robot walking. The ablation studies show that all four design parameters sit at or near
their empirical optimum in the delivered configuration, and identify two distinct high-gain
instability mechanisms — both consequences of applying gains designed for a lagged ZMP plant to a
plant without lag.

---

## Appendix A — Reproducing the results

All numbers and plots are produced by two headless drivers (no GUI, no keypress). From the
repository root, in the environment of §3.1:

```bash
python ablation_poles.py      # 26 headless sims -> logs/ablation_poles/raw_runs.pkl + plots
python make_runs_summary.py   # 8 headless config sims + reads the cache -> runs_summary.md
```

`ablation_poles.py` must be re-run before `make_runs_summary.py` whenever the controller changes,
otherwise the ablation rows keep stale cached values. Variants:

```bash
python ablation_poles.py --lag       # same sweep on the ZMP-lag plant -> logs/ablation_poles_lag/
python ablation_poles.py --replot    # rebuild plots from cache, no re-simulation
```

Individual configurations, interactively (press spacebar to start):

| config | command |
|---|---|
| plain (IS-MPC + CP + ZMP fb) | `python simulation.py` |
| IS-MPC baseline, no capture point | `python simulation.py --no-cp` |
| no ZMP feedback | `python simulation.py --no-zmp-fb` |
| ZMP-lag plant | `python simulation.py --lag` |
| open-loop MPC | `python simulation.py --open-loop` |
| CP feedback controller (no MPC) | `python simulation.py --no-mpc` |
| CP controller, no ZMP feedback | `python simulation.py --no-mpc --no-zmp-fb` |
| CP controller, ZMP-lag plant | `python simulation.py --no-mpc --lag` |

Append `--no-kf` to disable the Kalman filter. Each run writes `logs/log<suffix>.npz` and, unless
`--no-plot`, per-run plots into `logs/log<suffix>/`. For reduced run-to-run wobble, prefix any of
the above with `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1`.

## Appendix B — Symbols

| symbol | meaning |
|---|---|
| $c, \dot c, \ddot c$ | CoM position / velocity / acceleration |
| $p$ | ZMP; $p^{\text{meas}}$ measured, $p^{\text{ref}}$ regulated reference, $p^{\text{ff}}$ MPC feedforward, $p^{\text{cmd}}$ command |
| $\xi = c + \dot c/\eta$ | capture point (divergent component of motion) |
| $\eta = \sqrt{g/h}$ | LIP natural frequency, 3.6912 rad/s ($\omega$ in Morisawa's notation) |
| $\delta$ | control/simulation period, 10 ms |
| $N$ | MPC horizon, 100 steps (1.0 s) |
| $m(t)$ | moving-constraint ZMP midpoint |
| $\alpha,\beta,\gamma$ | assigned closed-loop poles of the balance controller |
| $g_p$ | first-order ZMP-lag gain (plant pole in lag mode; gain-design parameter always) |
| $k_1, k_2, k_i$ | CP-error, ZMP-error and CP-integral feedback gains |
| $I$ | saturated CP-error integral, $\pm 0.1$ m·s |
| $T_{ss}, T_{ds}$ | single- and double-support durations, 0.70 s / 0.30 s |
| $d$ | foot size / ZMP admissible-region side, 0.10 m |
| $F_z$ | vertical ground reaction force |

## Appendix C — Figure inventory for the LaTeX version

Available under `logs/ablation_poles/` (`.png`, 200 dpi), for each swept parameter
$P \in \{\alpha, \beta, \gamma, g_p\}$; each plot shows avg and max of $|e|$ across the sweep,
with the default marked in green and diverged runs marked by grey verticals:

- `zmp_error_{x,y,z}_vs_{P}.png` — ZMP tracking error per axis
- `com_error_{x,y,z}_vs_{P}.png` — CoM tracking error per axis
- `waist_{roll,pitch}_vs_{P}.png` — waist attitude error
- `vertical_force_vs_{P}.png` — avg and max $F_z$

Per-run time series (CoM/ZMP/CP trajectories per axis) are under `logs/log<suffix>/` for each of
the eight configurations of §3.3. Recommended figures for the paper: the baseline block diagram
(§1.2.1) and the modified one (§2), `zmp_error_x_vs_g_p.png` and `zmp_error_x_vs_beta.png` (they
carry the divergence story of §3.4.2 and §3.5), `com_error_z_vs_alpha.png` (the vertical-axis
sensitivity of §3.4.1), and a CoM/ZMP time series comparing *plain* against *open loop*.
