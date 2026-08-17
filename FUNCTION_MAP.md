# RMPC Function Map

This repository runs a pure SE(3) trajectory tracking benchmark on configurable
3D reference trajectories. The active entry point is `compare_se3_mpc.m`.

## Active Experiment Flow

`compare_se3_mpc.m`

- loads `cfg = rmpc.config.defaultConfig()`;
- builds the reference with `rmpc.reference.makeReference`;
- builds controller specs from `cfg.controllerKeys`;
- runs each selected controller at every time step;
- applies the same plant integration and disturbance model to each controller;
- evaluates all controllers with the same trajectory-tracking metrics.

Controller selection is configured in `+rmpc/+config/defaultConfig.m`:

```matlab
cfg.controllerKeys = {'euclidean_pid', 'liegroup_pid', 'euclidean_mpc', ...
    'quaternion_mpc', 'se3_mpc', 'euclidean_mpcc', 'riemannian_mpcc'};
```

To compare a subset, edit only this list, for example:

```matlab
cfg.controllerKeys = {'quaternion_mpc', 'se3_mpc', 'riemannian_mpcc'};
```

## Package Layout

### `+rmpc/+controllers`

Controller definitions and controller-specific optimization code.

| File | Role |
| --- | --- |
| `controllerSpecs.m` | Maps controller keys to display names, state type, controller kind, and solver file. |
| `euclideanPidTrackingControl.m` | Euclidean PID using world position error and RPY error. |
| `pidTrackingControl.m` | Lie Group PID on SE(3) log error with reference body-twist feedforward. |
| `initPidState.m` | PID integral/previous-error state. |
| `initMpccState.m` | MPCC progress and warm-start state. |
| `solveEuclideanMPCControl.m` | Euclidean MPC controller wrapper. |
| `solveQuaternionMPCControl.m` | Quaternion MPC controller wrapper. |
| `solveRiemannianMPCControl.m` | SE(3) MPC controller wrapper. The legacy key `riemannian_mpc` still maps here. |
| `solveTrackingMPC.m` | Shared `quadprog` QP builder for the three MPC controllers. |
| `solveEuclideanMPCCTrackingQP.m` | Euclidean MPCC QP solver. |
| `solveRiemannianMPCCTrackingQP.m` | Riemannian SE(3) MPCC QP solver. |

### `+rmpc/+evaluation`

Metrics, sanity checks, and plotting.

| File | Role |
| --- | --- |
| `mathSanityCheck.m` | Checks that reference body-twist rollout reconstructs the reference. |
| `printSanityReport.m` | Prints the sanity-check report. |
| `computeTrackingMetrics.m` | Computes contour, lag, rotation, and legacy pose metrics. |
| `mpccTrackingErrors.m` | Single-pose contour/lag/rotation error calculation. |
| `printTrackingMetrics.m` | Prints the active benchmark metric table. |
| `plotTrackingSummary.m` | Plots trajectory, contour error, lag error, and rotation error. |
| `initRealtimeViews.m` | Optional live visualization setup. |
| `updateRealtimeViews.m` | Optional live visualization update. |

### `+rmpc/+config`

Experiment-level configuration.

| File | Role |
| --- | --- |
| `defaultConfig.m` | Simulation time, trajectory parameters, controller list, weights, limits, solver options, and MPCC progress settings. |

### `+rmpc/+geometry`

Shared SO(3), SE(3), and quaternion math.

| Group | Files |
| --- | --- |
| SE(3) math | `expSE3.m`, `logSE3.m`, `invSE3.m`, `makeTransform.m` |
| SO(3) math | `expSO3.m`, `logSO3.m`, `projectSO3.m`, `rotmToQuat.m`, `quatToRotm.m`, `rotmToRpy.m` |
| Quaternion math | `quatMultiply.m`, `quatConj.m`, `quatExp.m`, `quatNormalize.m`, `quatLogError.m` |
| Lie algebra helpers | `skew.m`, `vee.m` |

### `+rmpc/+reference`

Reference trajectory generation and path-relative geometry.

| File | Role |
| --- | --- |
| `makeReference.m` | Samples the configured trajectory over the normalized progress grid. |
| `referenceAtProgress.m` | Evaluates one pose at normalized progress `s in [0, 1]`. |
| `referenceAtTheta.m` | Internal path-parameter pose evaluation for `spiral`, `line3d`, `sine3d`, or `flower3d`. |
| `referenceBodyTwists.m` | Converts sampled reference poses to body-twist feedforward controls. |
| `referenceTangentTwist.m` | Computes the body-twist derivative with respect to normalized path progress `s`. |
| `progressToPathParam.m` | Maps normalized progress to the path's internal parameter interval. |
| `sliceReference.m` | Extracts a finite horizon reference block. |
| `pathNormal.m` | Path-normal helper retained for spiral/cylinder-compatible metrics. |
| `pathDiagnosticDistance.m` | Path-distance diagnostic helper retained for sanity diagnostics. |
| `tangentProjector.m` | Tangent-plane projection helper. |
| `toolAxisAngle.m` | Tool-axis angle relative to the trajectory normal. |

### `+rmpc/+simulation`

Plant integration, disturbances, and history storage.

| File | Role |
| --- | --- |
| `stepQuatState.m` | Integrates a quaternion-state plant with body twist. |
| `stepLieState.m` | Integrates an SE(3)-state plant with body twist. |
| `makeDisturbance.m` | Builds the shared progress-triggered ARMA(2,1) disturbance sequence. |
| `applyQuatDisturbance.m` | Applies the disturbance to quaternion-state controllers. |
| `applyLieDisturbance.m` | Applies the disturbance to SE(3)-state controllers. |
| `initHistory.m` | Allocates per-controller rollout history. |
| `storeInitial.m` | Records the initial state. |
| `storeStep.m` | Records one simulation step. |

### `+rmpc/+utils`

Small shared numerical helpers.

| File | Role |
| --- | --- |
| `clamp.m` | Elementwise clamp. |
| `saturateControl.m` | Applies translational and angular velocity limits. |
| `shiftWarmStart.m` | Shifts an MPC/MPCC control warm start by one step. |
| `wrapToPiLocal.m` | Wraps angles to `[-pi, pi]`. |

### Removed Top-Level Utility Files

The old flat `+rmpc/*.m` utility layout has been removed. New code should call
the second-level packages directly, for example `rmpc.geometry.logSE3`,
`rmpc.reference.referenceAtTheta`, and `rmpc.simulation.stepLieState`.

### Removed Legacy Code

The current benchmark is pure trajectory tracking. Legacy trajectory/admittance
experiments, old metric/plot files, and old CasADi/Ipopt wrappers were removed
to keep the active code path clear. The active MPCC implementations are the
`quadprog` QP solvers in `+rmpc/+controllers`.

## State and Control Conventions

The control input is a body twist:

```math
u_k =
\begin{bmatrix}
v_k \\
\omega_k
\end{bmatrix}
\in \mathbb{R}^6 .
```

Quaternion-state controllers store:

```math
x = (p, q).
```

SE(3)-state controllers store:

```math
T =
\begin{bmatrix}
R & p \\
0 & 1
\end{bmatrix}.
```

The plant rollout uses right multiplication:

```math
T_{k+1} = T_k \exp(\Delta t\, u_k^\wedge).
```

The experiment-level reference trajectory is parameterized by normalized
progress:

```math
s(t) = \min(\dot{s}_{\mathrm{ref}}t, 1),
\qquad s \in [0,1].
```

Supported paths are configured in `+rmpc/+config/defaultConfig.m` with
`cfg.trajectory.pathType`. Each path also has an internal parameter
\(\theta\), mapped from progress by:

```math
\theta(s)=\theta_0+s(\theta_1-\theta_0).
```

The interval is configured by:

```matlab
cfg.trajectory.pathParamStart
cfg.trajectory.pathParamEnd
```

### `spiral`

This is the default cylinder helix:

```math
p_d(\theta) =
\begin{bmatrix}
r\cos\theta \\
r\sin\theta \\
z_0 + h\theta
\end{bmatrix}.
```

The tangent and normal are:

```math
t(\theta) =
\begin{bmatrix}
-r\sin\theta \\
r\cos\theta \\
h
\end{bmatrix},
\qquad
n(\theta) =
\begin{bmatrix}
\cos\theta \\
\sin\theta \\
0
\end{bmatrix}.
```

### `line3d`

The straight-line path is:

```math
p_d(\theta)
=
p_0
+
s\theta\,d,
\qquad
\|d\| = 1.
```

It is configured by:

```matlab
cfg.trajectory.pathType = 'line3d';
cfg.trajectory.lineP0
cfg.trajectory.lineDirection
cfg.trajectory.lineScale
cfg.trajectory.preferredNormal
```

### `sine3d`

The spatial sine-wave path is:

```math
p_d(\theta) =
\begin{bmatrix}
x_0 + s_x\theta \\
y_0 + A_y\sin(f_y\theta) \\
z_0 + A_z\sin(f_z\theta+\varphi_z)
\end{bmatrix}.
```

It is configured by:

```matlab
cfg.trajectory.pathType = 'sine3d';
cfg.trajectory.sineP0
cfg.trajectory.sineLengthScale
cfg.trajectory.sineAmpY
cfg.trajectory.sineAmpZ
cfg.trajectory.sineFreqY
cfg.trajectory.sineFreqZ
cfg.trajectory.sinePhaseZ
```

### `flower3d`

The rising flower path is a rose-like polar curve with increasing height:

```math
r(\theta)=r_0 + A\cos(n\theta),
```

```math
p_d(\theta) =
\begin{bmatrix}
r(\theta)\cos\theta \\
r(\theta)\sin\theta \\
z_0 + h\theta
\end{bmatrix}.
```

It is configured by:

```matlab
cfg.trajectory.pathType = 'flower3d';
cfg.trajectory.flowerBaseRadius
cfg.trajectory.flowerAmp
cfg.trajectory.flowerPetals
cfg.trajectory.flowerZ0
cfg.trajectory.flowerPitch
```

The default flower parameters are intentionally mild enough to respect the
current velocity and angular-rate limits. For a more visually pronounced
flower, reduce `cfg.referenceProgressRate` through `cfg.referenceOmega` or
relax the control limits.

### Orientation convention

For all supported paths, the reference frame is constructed as:

```math
x_d = \frac{t}{\|t\|},
```

where `x_d` is the path tangent direction. The tool axis is the body `z` axis,
and it is explicitly projected to be perpendicular to the tangent:

```math
z_d =
\frac{n - x_d(x_d^\top n)}
{\|n - x_d(x_d^\top n)\|}.
```

Then:

```math
y_d = z_d \times x_d,
\qquad
R_d = [x_d,\; y_d,\; z_d].
```

Therefore, for both `spiral` and `line3d`:

```math
x_d^\top z_d = 0.
```

## Controller Algorithms

Controller formulas and algorithm details are maintained separately in
`CONTROLLER_ALGORITHMS.md`.

## Active Evaluation Convention

PID and fixed-time MPC controllers are evaluated using the clock progress:

```math
s_{\mathrm{eval}}(t) = \min(\dot{s}_{\mathrm{ref}}t, 1).
```

MPCC controllers are evaluated using their optimized progress \(s\).

The reported metrics are:

- RMS contour error;
- RMS lag error;
- RMS rotation error;
- max contour error;
- max lag error;
- max rotation error;
- mean optimization cost.

This is not a free path-following evaluation. If a future experiment should
evaluate path following, the metrics should use each controller's optimized
progress or a closest-point projection instead.
