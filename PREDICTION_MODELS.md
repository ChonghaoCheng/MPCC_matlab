# Prediction Models

This document summarizes the prediction models currently implemented in the
trajectory tracking benchmark.

Files covered:

- `+rmpc/+simulation/stepQuatState.m`
- `+rmpc/+simulation/stepLieState.m`
- `+rmpc/+controllers/euclideanPidTrackingControl.m`
- `+rmpc/+controllers/pidTrackingControl.m`
- `+rmpc/+controllers/solveTrackingMPC.m`
- `+rmpc/+controllers/solveEuclideanMPCCTrackingQP.m`
- `+rmpc/+controllers/solveRiemannianMPCCTrackingQP.m`

## Common State Propagation

All controllers output a body twist

$$
u =
\begin{bmatrix}
v_b \\
\omega_b
\end{bmatrix}
\in \mathbb{R}^6 .
$$

The physical simulation step is

$$
T_{k+1} = T_k \exp_{\mathrm{SE}(3)}(\Delta t\,u_k).
$$

For quaternion-state controllers, the code converts the quaternion state to
an SE(3) matrix, applies the same update, and converts back:

$$
T_k =
\begin{bmatrix}
R(q_k) & p_k \\
0 & 1
\end{bmatrix}.
$$

The reference body twist used by PID and MPC is the exact discrete reference
increment used for model sanity checks and MPCC warm starts:

$$
u_{\mathrm{ref},k}
=
\frac{1}{\Delta t}
\log_{\mathrm{SE}(3)}
\left(
T_{\mathrm{ref},k}^{-1}T_{\mathrm{ref},k+1}
\right).
$$

This is implemented in `rmpc.reference.referenceBodyTwists`.

The controller reference trajectory also stores a world-frame translational
velocity for diagnostics and Euclidean-coordinate conversions:

$$
v_{\mathrm{ref},k}
=
\frac{p_{\mathrm{ref},k+1}-p_{\mathrm{ref},k}}{\Delta t}.
$$

For the last sample, the previous velocity is reused. This velocity is stored
as `ref.v` by `rmpc.reference.makeReference(progress, trajectory, dt)`.

The tracking PID and tracking MPC controllers use the full reference body
twist \(u_{\mathrm{ref},k}\), including angular velocity.

## PID Controllers

PID controllers do not build a horizon prediction model. They use the current
tracking error and the exact one-step reference feedforward.

### Euclidean PID

File: `+rmpc/+controllers/euclideanPidTrackingControl.m`

The Euclidean PID error is

$$
e_k =
\begin{bmatrix}
p_k - p_{\mathrm{ref},k} \\
\mathrm{wrap}(\mathrm{rpy}(R_k)-\mathrm{rpy}(R_{\mathrm{ref},k}))
\end{bmatrix}.
$$

The full reference body twist is converted to world/RPY coordinates:

$$
\nu_{\mathrm{ref},k}
=
\begin{bmatrix}
R_{\mathrm{ref},k}u^v_{\mathrm{ref},k} \\
E(\mathrm{rpy}_{\mathrm{ref},k})u^\omega_{\mathrm{ref},k}
\end{bmatrix}.
$$

The Euclidean PID command in world/RPY coordinates is

$$
\nu_k =
\nu_{\mathrm{ref},k}
-K_p e_k - K_i \sum e_k\Delta t - K_d\frac{e_k-e_{k-1}}{\Delta t}.
$$

The final control uses the full reference body twist. Its translational part is
converted to world velocity and its angular part is converted to RPY rate before
Euclidean feedback is applied:

$$
u_k =
\begin{bmatrix}
v^b_k \\
E(\mathrm{rpy}_k)^{-1}\nu^r_k
\end{bmatrix}.
$$

Here `E(rpy)` maps body angular velocity to RPY rate.

### Lie Group PID

File: `+rmpc/+controllers/pidTrackingControl.m`

The Lie group PID uses the left-invariant SE(3) error

$$
e_k =
\log_{\mathrm{SE}(3)}
\left(
T_{\mathrm{ref},k}^{-1}T_k
\right).
$$

The control is

$$
u_k =
u_{\mathrm{ref},k}
-K_p e_k
-K_i \sum e_k\Delta t
-K_d\frac{e_k-e_{k-1}}{\Delta t}.
$$

## Tracking MPC

File: `+rmpc/+controllers/solveTrackingMPC.m`

Tracking MPC solves a QP over the stacked body-twist sequence

$$
U =
\begin{bmatrix}
u_1^\top & u_2^\top & \cdots & u_N^\top
\end{bmatrix}^\top .
$$

The common lifted prediction form is

$$
E = S U + b,
$$

where

$$
E =
\begin{bmatrix}
e_1^\top & e_2^\top & \cdots & e_N^\top
\end{bmatrix}^\top .
$$

For stage `j`, the implemented first-order prediction is

$$
e_j =
e_0
+
\Delta t
\sum_{i=1}^{j}
B_i u_i
-
\Delta t
\sum_{i=1}^{j}
B_i u_{\mathrm{trans},i}.
$$

Equivalently,

$$
S_{ji} =
\begin{cases}
\Delta t\,B_i, & i \le j, \\
0, & i > j.
\end{cases}
$$

and

$$
b_j =
e_0
-
\Delta t
\sum_{i=1}^{j}
B_i u_{\mathrm{trans},i}.
$$

The QP objective is

$$
\min_U
E^\top \bar Q E
+
U^\top \bar R U
+
(DU+d_{\mathrm{prev}})^\top \bar R_d(DU+d_{\mathrm{prev}}),
$$

subject to box constraints

$$
u_{\min} \le u_i \le u_{\max}.
$$

### Euclidean MPC

Controller key: `euclidean_mpc`

The initial error is

$$
e_0 =
\begin{bmatrix}
p_0-p_{\mathrm{ref},0} \\
\mathrm{wrap}(\mathrm{rpy}(R_0)-\mathrm{rpy}(R_{\mathrm{ref},0}))
\end{bmatrix}.
$$

The input map is

$$
B_i =
\begin{bmatrix}
R_{\mathrm{ref},i} & 0 \\
0 & E(\mathrm{rpy}_{\mathrm{ref},i})
\end{bmatrix},
$$

where `E(rpy)` maps body angular velocity to RPY rate.

### Quaternion MPC

Controller key: `quaternion_mpc`

The initial error is

$$
e_0 =
\begin{bmatrix}
R_{\mathrm{ref},0}^\top(p_0-p_{\mathrm{ref},0}) \\
\log(q_{\mathrm{ref},0}^{-1}\otimes q_0)
\end{bmatrix}.
$$

The input map is currently approximated as identity:

$$
B_i = I_6.
$$

### SE(3) MPC

Controller key: `se3_mpc`

The initial error is

$$
e_0 =
\log_{\mathrm{SE}(3)}
\left(
T_{\mathrm{ref},0}^{-1}T_0
\right).
$$

The input map is also currently approximated as identity:

$$
B_i = I_6.
$$

This is a local left-invariant error prediction model. It is accurate near the
reference trajectory and becomes less exact for larger errors or longer
horizons because it ignores higher-order SE(3) error propagation terms.

## MPCC Progress Prediction

Files:

- `+rmpc/+controllers/solveEuclideanMPCCTrackingQP.m`
- `+rmpc/+controllers/solveRiemannianMPCCTrackingQP.m`

MPCC optimizes both body twists and progress rates:

$$
Z =
\begin{bmatrix}
u_1^\top & \cdots & u_N^\top &
v_{\theta,1} & \cdots & v_{\theta,N}
\end{bmatrix}^\top .
$$

The progress state is normalized:

$$
s \in [0,1].
$$

The current predicted progress model is

$$
s_j =
\min
\left(
s_0 + \Delta t\sum_{i=1}^{j} v_{\theta,i},
s_{\max}
\right).
$$

The implementation forms two progress grids:

$$
s_i^{\mathrm{start}}
=
s_0 + \Delta t\sum_{\ell=1}^{i-1} v_{\theta,\ell},
$$

and

$$
s_i^{\mathrm{end}}
=
s_0 + \Delta t\sum_{\ell=1}^{i} v_{\theta,\ell}.
$$

The error prediction currently uses `start` progress. The stage weights use
`end` progress.

The executed progress update is

$$
s_{k+1} =
\min(s_k + \Delta t\,v_{\theta,1}, s_{\max}).
$$

The first progress step is constrained by

$$
s_k + \Delta t\,v_{\theta,1}
\le
s_{\max} + \epsilon_s.
$$

The progress rate bounds are

$$
v_{\theta,\min} \le v_{\theta,i} \le v_{\theta,\max}.
$$

Near the end, the lower bound is relaxed to allow stopping at the terminal
progress.

## Euclidean MPCC Prediction

Controller key: `euclidean_mpcc`

The initial error is

$$
e_0 =
\begin{bmatrix}
R_{\mathrm{ref}}(s_0)^\top(p_0-p_{\mathrm{ref}}(s_0)) \\
\log_{\mathrm{SO}(3)}
\left(
R_{\mathrm{ref}}(s_0)^\top R_0
\right)
\end{bmatrix}.
$$

The current QP prediction model is

$$
e_j =
e_0
+
\Delta t
\sum_{i=1}^{j}
u_i
-
\Delta t
\sum_{i=1}^{j}
\tau(s_i^{\mathrm{start}})\,v_{\theta,i}.
$$

Here

$$
\tau(s) =
\frac{\partial}{\partial s}
\log_{\mathrm{SE}(3)}
\left(
T_{\mathrm{ref}}(s)^{-1}
T_{\mathrm{ref}}(s+\delta s)
\right)
$$

is approximated numerically by `referenceTangentTwist`.

In lifted form:

$$
E = A Z + b.
$$

For stage `j`, the implemented blocks are

$$
A_{j,u_i} =
\begin{cases}
\Delta t I_6, & i \le j, \\
0, & i > j,
\end{cases}
$$

and

$$
A_{j,v_i} =
\begin{cases}
-\Delta t\,\tau(s_i^{\mathrm{start}}), & i \le j, \\
0, & i > j.
\end{cases}
$$

with

$$
b_j = e_0.
$$

The Euclidean MPCC stage weight uses only the translational tangent for
contouring and lag:

$$
\hat t(s)
=
\frac{\tau_p(s)}{\|\tau_p(s)\|}.
$$

The contour projector is

$$
P_c = I_3 - \hat t \hat t^\top.
$$

The translational stage weight is

$$
Q_p =
Q_c P_c^\top P_c
+
Q_l \hat t\hat t^\top.
$$

The full stage weight is

$$
Q_e =
\begin{bmatrix}
Q_p & 0 \\
0 & Q_r
\end{bmatrix}.
$$

## Riemannian MPCC Prediction

Controller key: `riemannian_mpcc`

The initial error is the SE(3) left-invariant error:

$$
e_0 =
\log_{\mathrm{SE}(3)}
\left(
T_{\mathrm{ref}}(s_0)^{-1}T_0
\right).
$$

The current QP prediction model is the same first-order accumulated model:

$$
e_j =
e_0
+
\Delta t
\sum_{i=1}^{j}
u_i
-
\Delta t
\sum_{i=1}^{j}
\tau(s_i^{\mathrm{start}})\,v_{\theta,i}.
$$

The difference from Euclidean MPCC is the stage metric. Given

$$
M = G,
$$

and tangent

$$
\tau = \tau(s),
$$

the lag projection in the SE(3) metric is

$$
P_l =
\frac{\tau(\tau^\top M)}{\tau^\top M\tau}.
$$

The contour projector is

$$
P_c = I_6 - P_l.
$$

The normalized lag row is

$$
\ell^\top =
\frac{\tau^\top M}{\sqrt{\tau^\top M\tau}}.
$$

The Riemannian MPCC stage weight is

$$
Q_e =
P_c^\top Q_c P_c
+
Q_l \ell\ell^\top.
$$

## MPCC Regularization Terms

Both MPCC QPs add control, control-rate, progress-rate, and progress-rate-change
terms.

The body twist regularization is centered around the nominal path tangent
feedforward:

$$
u_{\mathrm{path},i}
=
\tau(s_i^{\mathrm{start}})v_{\theta,\mathrm{ref}}.
$$

The term is

$$
\sum_{i=1}^{N}
(u_i-u_{\mathrm{path},i})^\top R_u (u_i-u_{\mathrm{path},i}).
$$

The MPCC code has an optional explicit path-velocity consistency error:

$$
\sum_{i=1}^{N}
(u_i-\tau(s_i^{\mathrm{start}})v_{\theta,i})^\top
R_{\mathrm{path}}
(u_i-\tau(s_i^{\mathrm{start}})v_{\theta,i}).
$$

This term couples the optimized body twist to the optimized progress speed. It
uses the trajectory tangent as the reference velocity direction and makes
`vtheta` a direct velocity-tracking variable instead of only a progress state.
It is currently disabled by default while the MPCC velocity design is being
reconsidered:

$$
R_{\mathrm{path}} = 0.
$$

The input-rate penalty is

$$
\sum_{i=1}^{N}
(u_i-u_{i-1})^\top R_{\Delta u}(u_i-u_{i-1}).
$$

The progress speed penalty is

$$
\sum_{i=1}^{N}
R_v(v_{\theta,i}-v_{\theta,\mathrm{ref}})^2.
$$

The progress acceleration penalty is

$$
\sum_{i=1}^{N}
R_{\Delta v}(v_{\theta,i}-v_{\theta,i-1})^2.
$$

The forward progress reward is implemented with the MPCC minimization sign:

$$
-\sum_{i=1}^{N} r_v v_{\theta,i}.
$$

This negative sign encourages larger progress.

Optional soft progress tracking and terminal terms exist in the code, but are
currently disabled by default:

$$
w_s
\sum_{j=1}^{N}
\left(
s_j - s_{j,\mathrm{nominal}}
\right)^2,
$$

and

$$
w_{s,N}
\left(
s_N - s_{N,\mathrm{target}}
\right)^2.
$$

## Known Approximation in Current MPCC Prediction

The current MPCC prediction model is intentionally simple and QP-friendly, but
it is not an exact discrete SE(3) prediction model.

The real one-step SE(3) error propagation is

$$
E_{i+1}
=
T_{\mathrm{ref}}(s_{i+1})^{-1}
T_i
\exp_{\mathrm{SE}(3)}(\Delta t\,u_i),
$$

with

$$
T_i =
T_{\mathrm{ref}}(s_i)E_i.
$$

Equivalently,

$$
E_{i+1}
=
\Delta T_{\mathrm{ref},i}^{-1}
E_i
\exp_{\mathrm{SE}(3)}(\Delta t\,u_i),
$$

where

$$
\Delta T_{\mathrm{ref},i}
=
T_{\mathrm{ref}}(s_i)^{-1}T_{\mathrm{ref}}(s_{i+1}).
$$

A more accurate local model should linearize

$$
e_{i+1}
=
\log_{\mathrm{SE}(3)}
\left(
\Delta T_{\mathrm{ref},i}^{-1}
\exp_{\mathrm{SE}(3)}(e_i)
\exp_{\mathrm{SE}(3)}(\Delta t\,u_i)
\right),
$$

including the dependence of

$$
s_{i+1}
=
s_i+\Delta t\,v_{\theta,i}
$$

on all previous progress rates. The current model instead uses

$$
e_{i+1}
\approx
e_i
+
\Delta t\,u_i
-
\Delta t\,\tau(s_i)v_{\theta,i}.
$$

This explains why the current MPCC has a small but nonzero contour error in the
no-disturbance case at larger `dt`, even though the tracking MPC can nearly
cancel the discrete reference rollout.
