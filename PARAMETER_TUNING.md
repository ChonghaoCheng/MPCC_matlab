# Parameter Tuning Guide

This document lists the parameters that are currently useful to tune for the
trajectory-tracking and MPCC comparison experiments. Most experiment-level
parameters live in `+rmpc/+config/defaultConfig.m`.

## Experiment Timing and Progress

| Parameter | Meaning | Increase it when | Decrease it when |
| --- | --- | --- | --- |
| `cfg.dt` | Simulation/control sampling time. | You want faster runs and can tolerate coarser integration. | Tracking is unstable, QP linearization is too rough, or curves are sharp. |
| `cfg.N` | MPC/MPCC horizon length in discrete prediction steps. | Controllers need more lookahead, especially near curves or terminal progress. | QP solve time is too slow. |
| `cfg.referenceOmega` | Nominal speed in the path's internal parameter. It determines `cfg.referenceProgressRate`. | The reference should move faster. | Controls saturate or sanity check shows `max reference/control lim > 1`. |
| `cfg.referenceMaxControlUtilization` | Optional automatic cap on nominal reference speed as a fraction of control limits. Lower values leave more recovery authority after disturbances. | You want smaller contour/rotation error under disturbance, or trajectories with different geometry to have comparable actuator headroom. | You want to run every trajectory faster or at the raw `referenceOmega`; set it to `Inf` or `0` to disable. |
| `cfg.maxOverrunFactor` | Extra time allowed for free-progress MPCC to finish after nominal time. | MPCC needs more time to reach `s=1`. | You want stricter comparison by finish time. |
| `cfg.completionTol` | Completion tolerance for normalized progress. `s >= 1 - tol` counts as complete. | You want to avoid unnecessary final-step infeasibility. | You need stricter finish reporting. |
| `trajectory.pathParamStart`, `trajectory.pathParamEnd` | Internal path parameter interval mapped from normalized progress `s in [0,1]`. | You want a longer trajectory segment. | You want a shorter segment or easier benchmark. |

The experiment-level progress is normalized:

```matlab
s = 0      % trajectory start
s = 1      % trajectory end
```

For the default spiral, `trajectory.pathParamEnd = 2*pi`, but this is only the
spiral's internal parameter end, not a global rule for all paths.

## Trajectory Shape

Select the path with:

```matlab
trajectory.pathType = 'spiral';   % 'line3d', 'sine3d', or 'flower3d'
```

### Spiral

| Parameter | Meaning |
| --- | --- |
| `trajectory.radius` | Spiral radius. Larger values increase tangential speed for the same `referenceOmega`. |
| `trajectory.z0` | Initial height. |
| `trajectory.pitch` | Vertical rise per internal parameter unit. Larger pitch increases vertical velocity. |

### 3D Line

| Parameter | Meaning |
| --- | --- |
| `trajectory.lineP0` | Start point of the line. |
| `trajectory.lineDirection` | Line direction before normalization. |
| `trajectory.lineScale` | Distance traveled per internal parameter unit. |
| `trajectory.preferredNormal` | Preferred tool-normal direction projected perpendicular to the tangent. |

### 3D Sine

| Parameter | Meaning |
| --- | --- |
| `trajectory.sineP0` | Start offset for the sine curve. |
| `trajectory.sineLengthScale` | Forward speed scale along x. |
| `trajectory.sineAmpY`, `trajectory.sineAmpZ` | Lateral/vertical wave amplitudes. |
| `trajectory.sineFreqY`, `trajectory.sineFreqZ` | Wave frequencies. Higher values increase curvature and velocity demand. |
| `trajectory.sinePhaseZ` | Vertical sine phase. |

### Rising Flower

| Parameter | Meaning |
| --- | --- |
| `trajectory.flowerBaseRadius` | Base radial size. |
| `trajectory.flowerAmp` | Petal amplitude. Larger values make the flower stronger and harder. |
| `trajectory.flowerPetals` | Number of radial petals. Larger values increase curvature. |
| `trajectory.flowerZ0` | Initial height. |
| `trajectory.flowerPitch` | Vertical rise per internal parameter unit. |

If `mathSanityCheck` reports `max reference/control lim > 1`, reduce
`cfg.referenceOmega`, reduce curvature/amplitude, or increase the control
limits.

## Controller Selection

Choose which methods to compare with:

```matlab
cfg.controllerKeys = {'euclidean_pid', 'liegroup_pid', 'euclidean_mpc', ...
    'quaternion_mpc', 'se3_mpc', 'euclidean_mpcc', 'riemannian_mpcc'};
```

Available keys:

| Key | Controller |
| --- | --- |
| `euclidean_pid` | Euclidean PID |
| `liegroup_pid` | Lie Group PID |
| `euclidean_mpc` | Euclidean MPC |
| `quaternion_mpc` | Quaternion MPC |
| `se3_mpc` | SE(3) MPC |
| `euclidean_mpcc` | Euclidean MPCC |
| `riemannian_mpcc` | Riemannian MPCC |

## Control Limits

| Parameter | Meaning | Tuning direction |
| --- | --- | --- |
| `limits.vMax` | Body-frame linear velocity limits `[vx; vy; vz]`. | Increase if many methods saturate; decrease for a stricter actuator benchmark. |
| `limits.wMax` | Body-frame angular velocity limits `[wx; wy; wz]`. | Increase if orientation tracking or high-curvature paths saturate. |

These limits are converted to QP bounds:

```matlab
cfg.lb = repmat([-limits.vMax; -limits.wMax], cfg.N, 1);
cfg.ub = repmat([ limits.vMax;  limits.wMax], cfg.N, 1);
```

If you change `cfg.N` after these bounds are built, rebuild `cfg.lb` and
`cfg.ub`.

## MPC Tracking Weights

These weights affect Euclidean MPC, Quaternion MPC, and SE(3) MPC.

| Parameter | Meaning | Increase it when |
| --- | --- | --- |
| `weights.Qp` | Position tracking weight for Euclidean/Quaternion MPC. | Position error is too high. |
| `weights.Qr` | Rotation tracking weight for Euclidean/Quaternion MPC. | Rotation error is too high. |
| `weights.Qse3` | SE(3) log-error tracking weight. First 3 entries are translation, last 3 are rotation. | SE(3) MPC is too loose. |
| `weights.Ru` | Control magnitude penalty. | Controls are too aggressive. |
| `weights.Rdu` | Control-rate/smoothness penalty. | Control changes are too abrupt. |
| `weights.QpTerminal`, `weights.QrTerminal`, `weights.Qse3Terminal` | Terminal tracking weights. | End-of-horizon tracking is weak or oscillatory. |

General rule:

- Increase tracking weights to reduce error.
- Increase `Ru`/`Rdu` to make controls smoother and smaller.
- If control limits are active, increasing tracking weights may not improve
  tracking; it may only increase optimizer cost.

## MPCC Error and Progress Weights

These parameters affect `euclidean_mpcc` and `riemannian_mpcc`.

| Parameter | Meaning | Increase it when | Decrease it when |
| --- | --- | --- | --- |
| `mpcc.Qc` | Contouring-error weight. Can be scalar, 3x3 for Euclidean position contouring, or 6x6 for Riemannian SE(3) contouring. | The path-normal or orientation contour error is too large. | The controller refuses to make progress around curves. |
| `mpcc.Ql` | Lag-error weight. | The controller drifts along the tangent relative to its progress. | Progress becomes too conservative. |
| `mpcc.G` | SE(3) metric used by Riemannian MPCC. | You want to rebalance translation vs rotation in the Riemannian lag/contour decomposition. | The decomposition overemphasizes one part of the SE(3) tangent direction. |
| `mpcc.Ru` | MPCC control magnitude penalty. | MPCC commands are too aggressive. | MPCC is too sluggish. |
| `mpcc.Rdu` | MPCC control-rate penalty. | MPCC controls are noisy or discontinuous. | MPCC cannot respond quickly enough. |
| `mpcc.RpathVelocity` | 6D penalty on `u - u_ref(s)`, where `u_ref` is the full discrete reference body twist from `referenceBodyTwists`. | MPCC twist deviates too much from the fixed trajectory reference velocity. | MPCC needs more freedom to correct contouring error. |
| `mpcc.Rvprogress` | Penalty on progress speed deviation from `vprogressRef`. | MPCC progress oscillates or runs too fast/slow. | MPCC cannot adapt progress speed. |
| `mpcc.Rdvprogress` | Penalty on progress acceleration/change. | Progress speed changes abruptly. | MPCC needs faster progress adaptation. |
| `mpcc.progressReward` | Linear reward for forward progress in minimization. | MPCC is too slow or stops before the end. | MPCC sacrifices tracking to move forward. |
| `mpcc.progressTrackingWeight` | Soft penalty on deviation from fixed-speed nominal progress inside the MPCC horizon. | MPCC clock-progress lag is too large. | MPCC should be freer to slow down/speed up. |
| `mpcc.progressTerminalWeight` | Terminal penalty on `(s_N - 1)^2`. | MPCC does not finish the trajectory. | MPCC rushes to the end and tracking near the end degrades. |
| `mpcc.progressRateMin` | Lower bound on progress speed away from the end. | MPCC stalls mid-path. | QP infeasibility appears near difficult path sections. |
| `mpcc.progressRateMax` | Upper bound on progress speed. | MPCC progresses too slowly. | MPCC outruns the tracking controller. |
| `mpcc.progressUpperSlack` | Small slack allowing predicted progress to slightly exceed `s=1`. | QP is infeasible exactly at the end. | You need stricter terminal progress constraints. |

The MPCC objective includes both stage terms and a terminal progress term:

```math
J_{\mathrm{MPCC}}
\supset
Q_c e_c^2
+
Q_l e_l^2
+
R_u u^2
+
R_{\Delta u}\Delta u^2
+
\|u-u_{\mathrm{ref}}(s)\|^2_{R_{\mathrm{path}}}
+
R_{v_s}(v_s-v_{s,\mathrm{ref}})^2
+
R_{\Delta v_s}\Delta v_s^2
-
q_s v_s
+
Q_{s,N}(s_N-1)^2.
```

Here `mpcc.progressTerminalWeight` is \(Q_{s,N}\). This term is important for
making free-progress MPCC actually complete the trajectory instead of stopping
slightly before the end.

## Solver Options

The QP controllers use MATLAB `quadprog`:

```matlab
cfg.opts = optimoptions('quadprog', ...
    'Display', 'none', ...
    'Algorithm', 'interior-point-convex', ...
    'OptimalityTolerance', 1e-8, ...
    'StepTolerance', 1e-10, ...
    'ConstraintTolerance', 1e-9);
```

| Parameter | Tuning direction |
| --- | --- |
| `Display` | Use `'iter'` for debugging solver behavior. |
| `OptimalityTolerance` | Loosen for speed; tighten for accuracy. |
| `ConstraintTolerance` | Loosen only if tiny numerical infeasibility is acceptable. |
| `Algorithm` | Keep `'interior-point-convex'` unless you are comparing solver behavior. |

## PID Gains

PID gains are configured in `+rmpc/+config/defaultConfig.m`:

Current gains:

```matlab
pid.Kp = diag([1.45 1.25 1.30 9.0 9.0 7.0]);
pid.Ki = diag([0.04 0.04 0.04 0.20 0.20 0.15]);
pid.Kd = diag([0.16 0.14 0.16 0.35 0.35 0.28]);
```

Tuning direction:

- Increase `Kp` to reduce steady tracking error, but watch for overshoot.
- Increase `Kd` to damp oscillations, but too much can make controls noisy.
- Increase `Ki` only for persistent bias; too much integral can wind up.
- The integral state is clamped to `[-0.45, 0.45]` in each component.

These gains are finalized with the rest of the experiment configuration.

## Disturbance, Sanity Check, and Visualization

| Parameter | Meaning |
| --- | --- |
| `cfg.enableDisturbance` | Enables or disables the shared disturbance sequence. |
| `cfg.disturbance.type` | Disturbance/error type: `'progress_arma21'`/`'arma'`, `'translation_only'`, `'rotation_only'`, `'translation_rotation'`, or `'none'`. |
| `cfg.disturbance.progressInterval` | Progress spacing between disturbance events. Default `0.10` means events at `s=0.1,0.2,...,0.9`. |
| `cfg.disturbance.burstSteps` | Number of control steps in each disturbance burst. |
| `cfg.disturbance.armaA` | ARMA(2,1) autoregressive coefficients `[a1, a2]`. |
| `cfg.disturbance.armaB` | ARMA(2,1) moving-average coefficient `b1`. |
| `cfg.disturbance.transSigma` | Innovation standard deviation for translational disturbance. |
| `cfg.disturbance.rotSigma` | Innovation standard deviation for rotational disturbance. |
| `cfg.disturbance.maxTrans` | Per-axis clipping limit for translational disturbance. |
| `cfg.disturbance.maxRot` | Per-axis clipping limit for rotational disturbance. |
| `cfg.runMathSanity` | Runs feedforward rollout and limit consistency checks before the benchmark. |
| `cfg.enableRealtime` | Enables live visualization during simulation. |
| `cfg.colors` | Plot colors for selected controllers. |

The disturbance shape itself is generated in
`+rmpc/+simulation/makeDisturbance.m`.

For each event, the burst is generated as:

```math
d_k = a_1 d_{k-1} + a_2 d_{k-2} + \epsilon_k + b_1\epsilon_{k-1}.
```

Events are triggered when reference progress crosses multiples of
`progressInterval`. The terminal progress `s=1` is excluded, so the trajectory
end does not create a new disturbance event.

## Common Tuning Recipes

### MPCC does not finish the trajectory

Try this order:

1. Increase `mpcc.progressTerminalWeight`.
2. Increase `mpcc.progressReward`.
3. Increase `mpcc.progressRateMin`.
4. Increase `cfg.maxOverrunFactor`.
5. Check whether `limits.vMax`/`limits.wMax` are too restrictive.

### MPCC finishes but cuts corners

Try:

1. Increase `mpcc.Qc`.
2. Decrease `mpcc.progressReward`.
3. Decrease `mpcc.progressRateMax`.
4. Increase `mpcc.Rvprogress` so progress stays closer to the nominal rate.

### Rotation error is high

Try:

1. Increase `weights.Qr` for Euclidean/Quaternion MPC.
2. Increase the rotational diagonal entries of `weights.Qse3` for SE(3) MPC.
3. Increase the rotation-related entries of `mpcc.Qc` for MPCC.
4. Increase `limits.wMax` if angular velocity is saturating.

### QP is slow

Try:

1. Reduce `cfg.N`.
2. Increase `cfg.dt` if the dynamics remain accurate enough.
3. Loosen `OptimalityTolerance` and `StepTolerance`.
4. Reduce the number of controllers in `cfg.controllerKeys`.

### QP becomes infeasible near the end

Try:

1. Increase `mpcc.progressUpperSlack` slightly.
2. Reduce `mpcc.progressRateMin`.
3. Increase `cfg.completionTol`.
4. Check that `mpcc.progressRateMax` is not too small to reach the terminal
   tolerance.
