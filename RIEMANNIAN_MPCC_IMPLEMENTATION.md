# Riemannian MPCC Implementation

This document describes the current Riemannian MPCC implementation in this
repository and the parts that should be ported to a MuJoCo + ROS2 simulator.

Main MATLAB file:

```text
+rmpc/+controllers/solveRiemannianMPCCTrackingQP.m
```

## 1. State, Input, and Reference

The controller state is a pose on SE(3):

```math
T_k =
\begin{bmatrix}
R_k & p_k \\
0 & 1
\end{bmatrix}
\in SE(3).
```

The control input is a body-frame twist:

```math
u_k =
\begin{bmatrix}
v_k \\
\omega_k
\end{bmatrix}
\in \mathbb{R}^6.
```

The plant integration used by the Lie-state simulation is right-multiplicative:

```math
T_{k+1} = T_k \exp(\Delta t\,u_k).
```

In MuJoCo/ROS2 this means the controller output should be interpreted as a
body-frame spatial velocity command unless an adapter explicitly converts it to
world-frame velocity or joint commands.

The reference path is parameterized by normalized progress:

```math
s \in [s_{\min}, s_{\max}] = [0, 1].
```

At any progress value, the reference callback must provide:

```math
T_{\mathrm{ref}}(s), \quad p_{\mathrm{ref}}(s), \quad R_{\mathrm{ref}}(s).
```

The code also uses the SE(3) path tangent:

```math
\tau(s)
=
\frac{\partial}{\partial s}
\log\left(T_{\mathrm{ref}}(s)^{-1}T_{\mathrm{ref}}(s+\delta s)\right)
\in \mathbb{R}^6.
```

In the MATLAB code this is computed numerically by
`rmpc.reference.referenceTangentTwist`.

## 2. Controller Decision Variables

For horizon length `N`, the QP decision vector is:

```math
z =
\begin{bmatrix}
u_1 \\
\vdots \\
u_N \\
\dot{s}_1 \\
\vdots \\
\dot{s}_N
\end{bmatrix}
\in \mathbb{R}^{7N}.
```

Each stage optimizes:

- a 6D body twist `u_j`;
- a scalar progress rate `sdot_j`.

The first optimized progress rate is executed:

```math
s_{k+1} = \min(s_k + \Delta t\,\dot{s}_1,\ s_{\max}).
```

## 3. Initial SE(3) Error

At the current progress `s_k`, the Riemannian tracking error is:

```math
e_0 =
\log\left(T_{\mathrm{ref}}(s_k)^{-1}T_k\right)
\in \mathbb{R}^6.
```

This is implemented as:

```matlab
e0 = logSE3(invSE3(referenceAtProgress(progress0).T) * state.T);
```

The error is left-reference / body-local: it measures the current pose relative
to the reference pose at the same progress.

## 4. Linear Prediction Model

The implementation uses a first-order LTV prediction of the SE(3) error,
linearized about the *tracking nominal* `(e = 0, u = tau * sdot)`. The per-step
recursion is:

```math
e_j
=
\mathrm{Ad}_{\exp(-r_j)}\,e_{j-1}
+
\Delta t\,u_j
-
\Delta t\,\tau_j\,\dot{s}_j,
\qquad
r_j = \Delta t\,\dot{s}_j\,\tau_j.
```

### Derivation

The body/right error is `E = T_ref(s)^{-1} T`, with `e = log(E)`. Differentiating:

```math
[E^{-1}\dot E]^\vee
=
u - \mathrm{Ad}_{E^{-1}}(\tau\dot{s}).
```

Linearizing about `e = 0`, `u = tau * sdot` (so the body twist nominally moves
with the reference) and using `Ad_{exp(-e)} ≈ I - ad_e`:

```math
\dot{\delta e}
=
\delta u - \mathrm{ad}_{\tau\dot{s}}\,\delta e,
\qquad
\delta u = u - \tau\dot{s}.
```

The homogeneous part `d(δe)/dt = -ad_{τ sdot} δe` has the step transition
matrix `exp(-ad_r) = Ad_{exp(-r)}`, which is exactly the factor applied in the
code. The input term is approximated to first order as `dt * δu = dt*(u - tau*sdot)`.

A useful invariant: when `e0 = 0` and every `u_i = tau_i * sdot_i`, the model
predicts `e_j = 0` for all `j` (perfect tracking is a fixed point).

### Stacked form

```math
E = A z + b,
\qquad
E = [e_1; \dots; e_N].
```

Building the recursion column by column (see `errorPredictionModel`):

- the carried-over error block is left-multiplied by `F_j = Ad_{exp(-r_j)}`
  each step, so `b = F_N ... F_1 e_0`;
- each control `u_i` enters its own stage with `dt * I_6`, then is transported
  by the subsequent `F_{i+1} ... F_j`;
- each progress rate `sdot_i` contributes `-dt * tau(s_i)`, transported the same way.

### Linearization-point notes (important for porting)

- This is not a full nonlinear shooting MPC. It is a single QP per tick,
  linearized about the warm-start trajectory.
- The transition factors `F_j` are frozen at the **guessed** progress rate
  (`dsGuess = dt * vprogressGuess(j)`), because putting the decision variable
  `sdot_j` inside the matrix exponential would make the model nonlinear and break
  the QP structure. The affine term `-dt * tau_j * sdot_j` keeps `sdot_j` as the
  true decision variable. The model is therefore accurate when the solved `sdot`
  stays close to the warm start; the next tick re-linearizes around the updated
  warm start (real-time-iteration / single-SQP behavior).
- The input term uses `dt * I_6` rather than the exact zero-order-hold integral
  `∫_0^{dt} Ad_{exp(-σ τ sdot)} dσ`. The difference is `O(dt^2)` and is ignored.
- The earlier "no-adjoint" model `e_j ≈ e_0 + dt * Σ (u_i - tau_i sdot_i)` is the
  cruder linearization (about `u = 0`) that drops the `-ad_{τ sdot}` coupling. The
  Euclidean MPCC variant still uses that simpler form; the Riemannian variant
  here keeps the adjoint and is more accurate.

## 5. Riemannian Contour and Lag Cost

The stage cost decomposes the 6D SE(3) error into lag and contour components
using the full SE(3) path tangent.

Let:

```math
M = \texttt{mpcc.G}.
```

The current default is:

```matlab
mpcc.G = diag([450 450 250 10 10 10]);
```

For each predicted progress `s_j`:

```math
\tau_j = \tau(s_j).
```

The lag projection is:

```math
P_{\ell}
=
\frac{\tau_j(\tau_j^\top M)}
       {\tau_j^\top M \tau_j}.
```

The contour projection is:

```math
P_c = I_6 - P_{\ell}.
```

The normalized lag row is:

```math
\ell^\top =
\frac{\tau_j^\top M}
     {\sqrt{\tau_j^\top M \tau_j}}.
```

The Riemannian stage weight is:

```math
Q_e
=
P_c^\top Q_c P_c
+
Q_l\,\ell\ell^\top.
```

Current defaults:

```matlab
mpcc.Qc = diag([600 600 350 600 600 350]);
mpcc.Ql = 80;
```

The last prediction stage receives a terminal multiplier:

```math
Q_{e,N} = 2.2\,Q_e.
```

## 6. QP Objective

The QP has the standard MATLAB `quadprog` form:

```math
\min_z \frac{1}{2} z^\top H z + f^\top z.
```

The main tracking contribution is:

```math
\sum_{j=1}^{N} e_j^\top Q_{e,j} e_j.
```

Using `E = A z + b`, this becomes:

```math
H_{\mathrm{track}} = 2 A^\top Q A,
\quad
f_{\mathrm{track}} = 2 A^\top Q b.
```

Additional regularization terms:

### Control magnitude

```math
\sum_{j=1}^{N} u_j^\top R_u u_j.
```

Default:

```matlab
mpcc.Ru = 1e-5 * eye(6);
```

### Control rate

```math
\sum_{j=1}^{N} (u_j-u_{j-1})^\top R_{\Delta u}(u_j-u_{j-1}).
```

The first difference uses the previously executed control `uPrev`.

Default:

```matlab
mpcc.Rdu = 1e-5 * eye(6);
```

### Full reference body-twist tracking

The controller also penalizes deviation from the full discrete reference twist:

```math
\sum_{j=1}^{N}
(u_j - u_{\mathrm{ref}}(s_j))^\top
R_{\mathrm{pathVelocity}}
(u_j - u_{\mathrm{ref}}(s_j)).
```

Here `u_ref(s)` comes from `referenceBodyTwists`, not from
`tau(s) * vprogressRef`.

Default:

```matlab
mpcc.RpathVelocity = 1e-5 * eye(6);
```

### Progress-rate tracking

```math
\sum_{j=1}^{N}
R_{\dot{s}}
(\dot{s}_j - \dot{s}_{\mathrm{ref}})^2.
```

Default:

```matlab
mpcc.Rvprogress = 80;
```

`vprogressRef` is filled in `finalizeConfig`:

```matlab
cfg.mpcc.vprogressRef = cfg.referenceProgressRate;
```

### Progress-rate smoothness

```math
\sum_{j=1}^{N}
R_{\Delta \dot{s}}
(\dot{s}_j-\dot{s}_{j-1})^2.
```

Default:

```matlab
mpcc.Rdvprogress = 0.5;
```

### Forward progress reward

The implementation adds a linear reward:

```math
- r_s \sum_{j=1}^{N} \dot{s}_j.
```

Default:

```matlab
mpcc.progressReward = 0.01;
```

Optional progress tracking and terminal progress costs exist in the solver, but
their current default weights are zero:

```matlab
mpcc.progressTrackingWeight = 0;
mpcc.progressTerminalWeight = 0;
```

## 7. Constraints

### Twist limits

Each predicted twist is box-constrained:

```math
u_{\min} \le u_j \le u_{\max}.
```

Current defaults:

```matlab
limits.vMax = [1.45; 0.34; 0.28];
limits.wMax = [0.90; 0.90; 1.10];
```

The QP bounds repeat these limits across the horizon.

### Progress-rate limits

```math
\dot{s}_{\min} \le \dot{s}_j \le \dot{s}_{\max}.
```

`finalizeConfig` computes:

```matlab
mpcc.progressRateMin = 0.25 / abs(pathParamEnd - pathParamStart);
mpcc.progressRateMax = 1.15 / abs(pathParamEnd - pathParamStart);
```

Near the terminal progress, the lower bound is relaxed to zero:

```matlab
if remaining <= completionTol
    lb = zeros(N, 1);
elseif remaining < N * dt * progressRateMin
    lb = zeros(N, 1);
end
```

### First-step terminal progress constraint

Only the first optimized progress step is constrained against overrun:

```math
s_k + \Delta t\,\dot{s}_1
\le
s_{\max} + \epsilon_s.
```

Default:

```matlab
mpcc.progressUpperSlack = 1e-4;
```

The executed progress is clamped to `progressEnd`.

## 8. Warm Start

The warm start stores:

```matlab
mpcc.zGuess = [uGuess; vprogressGuess];
```

Initial values:

- `uGuess`: first `N` full reference body twists;
- `vprogressGuess`: `vprogressRef` repeated `N` times.

After every solve:

```matlab
U = [U(:, 2:end), U(:, end)];
V = [V(2:end), V(end)];
```

The shifted vector becomes the next QP warm start.

If `quadprog` fails, the implementation clips the warm start to the current
bounds and reuses it.

## 9. Reference-Speed Normalization

The current config includes a task-level speed normalization:

```matlab
cfg.referenceMaxControlUtilization = 0.45;
```

This estimates:

```math
\max_s
\left|
u_{\mathrm{ref}}(s)
\right|
/
\left[
v_{\max};
\omega_{\max}
\right].
```

If this utilization exceeds the configured cap, `referenceOmega` and
`referenceProgressRate` are scaled down. This is not an RMPCC gain. It makes
different trajectory geometries have comparable actuator headroom before
disturbances.

For MuJoCo/ROS2:

- keep this if you want comparable experiments across trajectory types;
- disable it with `0` or `Inf` if the simulator should run the raw requested
  trajectory speed.

## 10. Runtime Loop for ROS2/MuJoCo

At each control tick:

1. Read the current end-effector pose `T_k` from MuJoCo or TF.
2. Keep the current MPCC progress `s_k` as controller state.
3. Build or query the reference functions:
   - `T_ref(s)`;
   - `tau(s)`;
   - `u_ref(s)` lookup/interpolation.
4. Solve the QP for `z`.
5. Execute `u_1 = z(1:6)`.
6. Update:

   ```math
   s_{k+1} = \min(s_k + \Delta t\,\dot{s}_1,\ 1).
   ```

7. Shift the warm start.

For a velocity-controlled MuJoCo body, apply:

```math
T_{k+1} = T_k\exp(\Delta t\,u_1)
```

or convert the body twist to the simulator's actuator command. For a joint-space
robot, use a resolved-rate controller:

```math
\dot{q} = J(q)^\#\,V_{\mathrm{desired}},
```

with the correct body/world frame conversion before applying the Jacobian.

## 11. Porting Checklist

- Implement `expSE3`, `logSE3`, and `invSE3` with the same twist ordering:
  `[vx vy vz wx wy wz]`.
- Keep the body-frame convention consistent between RMPCC output and MuJoCo
  actuation.
- Use a QP solver that supports:
  - dense positive semidefinite Hessian;
  - box bounds;
  - one linear inequality for first-step progress.
- Preserve the controller-owned progress state `s`; do not infer it only from
  wall-clock time.
- Store and shift the warm start each control tick.
- If comparing trajectories, normalize reference speed or report control-limit
  utilization.
- For debugging, log:
  - current progress;
  - `||log(T_ref(s)^-1 T)||`;
  - path contour error;
  - rotation error;
  - QP exit status;
  - first control and first progress rate.

## 12. Important Current Limitations

- The prediction model is first-order in SE(3), not full nonlinear shooting.
- `u_ref(s)` is currently selected by nearest lower table index, not continuous
  interpolation.
- Only the first progress step is explicitly constrained against terminal
  overrun.
- The QP assumes a velocity-level interface. A torque-level MuJoCo controller
  needs an additional tracking layer.
