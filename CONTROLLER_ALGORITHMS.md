# Controller Algorithms

This document summarizes the controller algorithms used by the active
trajectory-tracking benchmark in `compare_se3_mpc.m`.

The control input is a body twist:

```math
u_k =
\begin{bmatrix}
v_k \\
\omega_k
\end{bmatrix}
\in \mathbb{R}^6 .
```

The plant rollout uses right multiplication:

```math
T_{k+1} = T_k \exp(\Delta t\, u_k^\wedge).
```

The reference is parameterized by:

```math
s(t) = \min(\dot{s}_{\mathrm{ref}}t, 1).
```

## 1. Euclidean PID

File: `+rmpc/+controllers/euclideanPidTrackingControl.m`

Tracking error:

```math
e_k =
\begin{bmatrix}
p_k - p_{d,k} \\
\mathrm{rpy}(R_k) - \mathrm{rpy}(R_{d,k})
\end{bmatrix}.
```

Reference feedforward starts from the full SE(3) body twist:

```math
u_{\mathrm{ref},k}
=
\frac{1}{\Delta t}
\log\left(T_{d,k}^{-1}T_{d,k+1}\right)^\vee .
```

The translational component is mapped to world velocity and the angular
component is mapped to RPY rate before PID feedback is applied.

PID command in Euclidean coordinates:

```math
\dot x_{\mathrm{cmd}}
=
\begin{bmatrix}
\dot p_{d,k} \\
\dot r_{d,k}
\end{bmatrix}
-K_p e_k
-K_i I_k
-K_d \dot e_k .
```

The command is mapped to body twist by:

```math
v = R_d^\top \dot p_{\mathrm{cmd}},
```

```math
\omega =
\begin{bmatrix}
1 & 0 & -\sin\theta \\
0 & \cos\phi & \sin\phi\cos\theta \\
0 & -\sin\phi & \cos\phi\cos\theta
\end{bmatrix}
\dot r_{\mathrm{cmd}},
```

where \(r=[\phi,\theta,\psi]^\top\) is roll-pitch-yaw.

The result is saturated by the configured velocity limits.

## 2. Lie Group PID

File: `+rmpc/+controllers/pidTrackingControl.m`

Pose error:

```math
e_k =
\log\left(T_{d,k}^{-1}T_k\right)^\vee
\in \mathbb{R}^6 .
```

Reference feedforward:

```math
u_{d,k} =
\frac{1}{\Delta t}
\log\left(T_{d,k}^{-1}T_{d,k+1}\right)^\vee .
```

PID law:

```math
\dot e_k \approx \frac{e_k - e_{k-1}}{\Delta t},
\qquad
I_k = \mathrm{clip}(I_{k-1} + \Delta t\,e_k).
```

```math
u_k =
u_{d,k}
- K_p e_k
- K_i I_k
- K_d \dot e_k .
```

The result is saturated by the configured velocity limits. The legacy key
`pid` maps to this controller; the explicit key is `liegroup_pid`.

## 3. Euclidean MPC

Files:

- `+rmpc/+controllers/solveEuclideanMPCControl.m`
- shared QP builder: `+rmpc/+controllers/solveTrackingMPC.m`

Tracking error:

```math
e_k =
\begin{bmatrix}
p_k - p_{d,k} \\
\mathrm{rpy}(R_k) - \mathrm{rpy}(R_{d,k})
\end{bmatrix}.
```

The control input is a body twist, so the local QP model maps it into
Euclidean coordinates:

```math
\dot p \approx R_d v,
```

```math
\frac{d}{dt}\mathrm{rpy}
\approx
E(\mathrm{rpy}_d)\omega ,
```

where:

```math
E(\phi,\theta,\psi) =
\begin{bmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi            & -\sin\phi \\
0 & \sin\phi/\cos\theta & \cos\phi/\cos\theta
\end{bmatrix}.
```

The QP minimizes:

```math
\sum_{j=1}^{N}
e_j^\top Q e_j
+
\sum_{j=0}^{N-1}
u_j^\top R_u u_j
+
(u_j-u_{j-1})^\top R_{\Delta u}(u_j-u_{j-1})
```

subject to the box constraints on translational and angular velocity.

## 4. Quaternion MPC

Files:

- `+rmpc/+controllers/solveQuaternionMPCControl.m`
- shared QP builder: `+rmpc/+controllers/solveTrackingMPC.m`

Tracking error:

```math
e_k =
\begin{bmatrix}
R_{d,k}^\top(p_k - p_{d,k}) \\
\log(q_{d,k}^{-1}\otimes q_k)
\end{bmatrix}.
```

The QP uses the local approximation:

```math
e_{j+1}
\approx
e_j
+
\Delta t\,(u_j-u_{d,j}).
```

The cost and input constraints are the same structure as Euclidean MPC.

## 5. SE(3) MPC

Files:

- `+rmpc/+controllers/solveRiemannianMPCControl.m`
- shared QP builder: `+rmpc/+controllers/solveTrackingMPC.m`

The active controller key is `se3_mpc`. The old key `riemannian_mpc` is kept
as a compatibility alias.

Tracking error is the local SE(3) logarithmic error:

```math
\eta_k =
\log\left(T_{d,k}^{-1}T_k\right)^\vee
\in \mathbb{R}^6 .
```

The QP model is:

```math
\eta_{j+1}
\approx
\eta_j
+
\Delta t\,(u_j-u_{d,j}).
```

The cost is:

```math
\sum_{j=1}^{N}
\eta_j^\top Q_{\mathrm{se3}}\eta_j
+
\sum_{j=0}^{N-1}
u_j^\top R_u u_j
+
(u_j-u_{j-1})^\top R_{\Delta u}(u_j-u_{j-1}).
```

This is a fixed-time trajectory-tracking MPC. It does not optimize normalized
path progress `s`.

## 6. Euclidean MPCC

File: `+rmpc/+controllers/solveEuclideanMPCCTrackingQP.m`

Decision variables:

```math
z =
\begin{bmatrix}
u_0,\ldots,u_{N-1},
v_{s,0},\ldots,v_{s,N-1}
\end{bmatrix}.
```

The progress dynamics are:

```math
s_{j+1}
=
s_j + \Delta t\,v_{s,j}.
```

Position error is decomposed with the Euclidean path tangent:

```math
e_p = p - p_d(s),
\qquad
\tau_p =
\frac{\partial p_d/\partial s}
{\|\partial p_d/\partial s\|}.
```

Lag and contour errors:

```math
e_l = \tau_p^\top e_p,
\qquad
e_c = e_p - \tau_p e_l.
```

Rotation error is separate:

```math
e_R =
\log\left(R_d(s)^\top R\right)^\vee.
```

The QP uses a local linearized model around the current progress warm start.
The stage cost is:

```math
q_c\|e_c\|^2
+
q_l e_l^2
+
e_R^\top Q_R e_R
+
u^\top R_u u
+
\Delta u^\top R_{\Delta u}\Delta u
+
R_{v_s}(v_s-v_{s,\mathrm{ref}})^2
+
R_{\Delta v_s}\Delta v_s^2
-
q_s v_s .
```

Constraints:

```math
u_{\min} \le u_j \le u_{\max},
```

```math
v_{s,\min}
\le
v_{s,j}
\le
v_{s,\max},
```

and a progress tube around the planned trajectory:

```math
s_{\mathrm{ref},j} - s_{\mathrm{lower}}
\le
s_j
\le
s_{\mathrm{ref},j} + s_{\mathrm{upper}}.
```

## 7. Riemannian MPCC

File: `+rmpc/+controllers/solveRiemannianMPCCTrackingQP.m`

Decision variables are the same as Euclidean MPCC:

```math
z =
\begin{bmatrix}
u_0,\ldots,u_{N-1},
v_{s,0},\ldots,v_{s,N-1}
\end{bmatrix}.
```

The pose error is computed in the tangent space of SE(3):

```math
\eta(s)
=
\log\left(T_d(s)^{-1}T\right)^\vee
\in \mathbb{R}^6 .
```

The left-trivialized reference tangent is:

```math
\tau(s)
\approx
\frac{1}{h}
\log\left(T_d(s)^{-1}T_d(s+h)\right)^\vee .
```

With a positive-definite metric:

```math
M = G,
```

the lag projection coefficient is:

```math
\delta s_l
=
\frac{\tau^\top M\eta}{\tau^\top M\tau}.
```

Lag vector and contour vector:

```math
\eta_l = \delta s_l\,\tau,
\qquad
\eta_c = \eta - \eta_l.
```

Scalar metric lag error:

```math
e_l =
\frac{\tau^\top M\eta}{\sqrt{\tau^\top M\tau}}.
```

Metric contour error:

```math
e_c^2 = \eta_c^\top M\eta_c.
```

The stage cost is:

```math
q_c\,\eta_c^\top M\eta_c
+
q_l\,e_l^2
+
u^\top R_u u
+
\Delta u^\top R_{\Delta u}\Delta u
+
R_{v_s}(v_s-v_{s,\mathrm{ref}})^2
+
R_{\Delta v_s}\Delta v_s^2
-
q_s v_s .
```

It uses the same input, progress-rate, and progress-tube constraints as
Euclidean MPCC.

This is the QP approximation of the SE(3)-based Riemannian MPCC described in
`README_SE3_MPCC_ERROR.md`.
