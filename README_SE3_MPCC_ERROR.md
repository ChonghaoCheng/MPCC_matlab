# SE(3)-Based Riemannian MPCC Error Decomposition

This module implements a local geometric error decomposition for model predictive contouring control (MPCC) on (SE(3)). It is designed for robotic pose tracking tasks where the end-effector pose follows a reference trajectory while both translational and rotational errors must be handled consistently.

The main idea is:

[
\text{pose error on } SE(3)
\quad
\longrightarrow
\quad
\text{Lie algebra error in } \mathfrak{se}(3)
\quad
\longrightarrow
\quad
\text{metric projection into lag and contouring errors}.
]

Unlike Euclidean MPCC, this method does not subtract two poses directly. Instead, it uses the relative transformation and the Lie logarithm to obtain a local tangent-space error.

---

## 1. Problem Setup

Let the current end-effector pose be

[
\mathbf{T} \in SE(3),
]

where

[
\mathbf{T}
==========

\begin{bmatrix}
\mathbf{R} & \mathbf{p}\
\mathbf{0}^{\top} & 1
\end{bmatrix},
\qquad
\mathbf{R}\in SO(3),
\qquad
\mathbf{p}\in\mathbb{R}^3.
]

Let the reference pose trajectory be parameterized by the MPCC progress variable (\theta):

[
\mathbf{T}_d(\theta)\in SE(3).
]

The goal is to decompose the pose tracking error into:

1. a **lag error**, which measures error along the reference trajectory direction;
2. a **contouring error**, which measures deviation away from the reference trajectory.

---

## 2. SE(3) Pose Error

In Euclidean MPCC, one may define the error as

[
\mathbf{e}
==========

## \mathbf{x}

\mathbf{x}_d(\theta).
]

For poses on (SE(3)), this is not geometrically meaningful. Instead, define the relative pose error as

[
\mathbf{T}_e(\theta)
====================

\mathbf{T}_d(\theta)^{-1}\mathbf{T}.
]

The Lie-logarithmic pose error is then

[
\boldsymbol{\eta}(\theta)
=========================

\log
\left(
\mathbf{T}_d(\theta)^{-1}\mathbf{T}
\right)^{\vee}
\in
\mathbb{R}^6.
]

We use the convention

[
\boldsymbol{\eta}
=================

\begin{bmatrix}
\boldsymbol{\rho}\
\boldsymbol{\phi}
\end{bmatrix},
]

where

[
\boldsymbol{\rho}\in\mathbb{R}^3
]

is the translational component, and

[
\boldsymbol{\phi}\in\mathbb{R}^3
]

is the rotational component.

The corresponding Lie algebra matrix is

[
\boldsymbol{\eta}^{\wedge}
==========================

\begin{bmatrix}
\boldsymbol{\phi}^{\wedge} & \boldsymbol{\rho}\
\mathbf{0}^{\top} & 0
\end{bmatrix}.
]

This error is a **local tangent-space representation** of the relative pose. It should not be interpreted as a global Euclidean displacement.

---

## 3. Reference Tangent Direction

The reference trajectory is a curve on (SE(3)):

[
\theta
\mapsto
\mathbf{T}_d(\theta).
]

Its derivative is

[
\frac{\partial \mathbf{T}*d(\theta)}{\partial \theta}
\in
T*{\mathbf{T}_d(\theta)}SE(3).
]

To compare this tangent vector with the error (\boldsymbol{\eta}), both must be represented in the same vector space. We use the left-trivialized reference tangent:

[
\boldsymbol{\tau}(\theta)
=========================

\left(
\mathbf{T}_d(\theta)^{-1}
\frac{\partial \mathbf{T}_d(\theta)}{\partial \theta}
\right)^{\vee}
\in
\mathbb{R}^6.
]

Therefore,

[
\boldsymbol{\eta}(\theta)\in\mathbb{R}^6,
\qquad
\boldsymbol{\tau}(\theta)\in\mathbb{R}^6.
]

Now the error and reference tangent are in the same Lie algebra coordinate space.

For implementation, if an analytic derivative is unavailable, the tangent can be approximated by finite difference:

[
\boldsymbol{\tau}(\theta)
\approx
\frac{1}{\Delta \theta}
\log
\left(
\mathbf{T}_d(\theta)^{-1}
\mathbf{T}_d(\theta+\Delta\theta)
\right)^{\vee}.
]

---

## 4. Riemannian Metric on (\mathfrak{se}(3))

A positive-definite matrix is used to define an inner product on (\mathfrak{se}(3)):

[
\mathbf{M}\in\mathbb{R}^{6\times 6},
\qquad
\mathbf{M}=\mathbf{M}^{\top},
\qquad
\mathbf{M}\succ 0.
]

For two vectors

[
\boldsymbol{\xi}_1,\boldsymbol{\xi}_2\in\mathbb{R}^6,
]

the metric inner product is

[
\left\langle
\boldsymbol{\xi}_1,
\boldsymbol{\xi}*2
\right\rangle*{\mathbf{M}}
==========================

\boldsymbol{\xi}_1^{\top}
\mathbf{M}
\boldsymbol{\xi}_2.
]

The metric norm is

[
\left|
\boldsymbol{\xi}
\right|_{\mathbf{M}}
====================

\sqrt{
\boldsymbol{\xi}^{\top}
\mathbf{M}
\boldsymbol{\xi}
}.
]

A common choice is

[
\mathbf{M}
==========

\begin{bmatrix}
w_p\mathbf{I}_3 & \mathbf{0}\
\mathbf{0} & w_R\mathbf{I}_3
\end{bmatrix},
\qquad
w_p>0,
\qquad
w_R>0.
]

Here:

[
w_p
]

weights translational error, and

[
w_R
]

weights rotational error.

This metric is necessary because translation and rotation have different physical units.

---

## 5. Lag Error Projection

The projection coefficient of the pose error along the reference tangent is

[
\delta\theta_l(\theta)
======================

\frac{
\boldsymbol{\tau}(\theta)^{\top}
\mathbf{M}
\boldsymbol{\eta}(\theta)
}{
\boldsymbol{\tau}(\theta)^{\top}
\mathbf{M}
\boldsymbol{\tau}(\theta)
}.
]

This coefficient measures how much of the pose error lies along the progress direction.

The lag-direction error vector is

[
\boldsymbol{\eta}_l(\theta)
===========================

\delta\theta_l(\theta)
\boldsymbol{\tau}(\theta).
]

The scalar metric-length lag error is

[
e_l(\theta)
===========

\frac{
\boldsymbol{\tau}(\theta)^{\top}
\mathbf{M}
\boldsymbol{\eta}(\theta)
}{
\sqrt{
\boldsymbol{\tau}(\theta)^{\top}
\mathbf{M}
\boldsymbol{\tau}(\theta)
}
}.
]

The difference is:

[
\delta\theta_l
]

is a progress-domain lag coefficient, while

[
e_l
]

is the lag error measured as a metric length.

---

## 6. Contouring Error

The contouring error is the metric-orthogonal residual after removing the lag-direction component:

[
\boldsymbol{\eta}_c(\theta)
===========================

## \boldsymbol{\eta}(\theta)

\boldsymbol{\eta}_l(\theta).
]

Equivalently,

[
\boldsymbol{\eta}(\theta)
=========================

\boldsymbol{\eta}_l(\theta)
+
\boldsymbol{\eta}_c(\theta).
]

The contouring error magnitude is

[
e_c(\theta)
===========

\sqrt{
\boldsymbol{\eta}_c(\theta)^{\top}
\mathbf{M}
\boldsymbol{\eta}_c(\theta)
}.
]

By construction,

[
\boldsymbol{\tau}(\theta)^{\top}
\mathbf{M}
\boldsymbol{\eta}_c(\theta)
===========================

0.

]

Therefore, (\boldsymbol{\eta}_c) is orthogonal to the reference tangent under the chosen metric.

---

## 7. MPCC Cost Function

A geometric MPCC cost can be defined as

[
\ell_{\mathrm{geo}}
===================

q_c
\boldsymbol{\eta}_c^{\top}
\mathbf{M}
\boldsymbol{\eta}_c
+
q_l
e_l^2,
]

where

[
q_c>0,
\qquad
q_l>0.
]

For a finite prediction horizon (N), the total cost can be written as

[
J
=

\sum_{k=0}^{N-1}
\left(
q_c
\boldsymbol{\eta}*{c,k}^{\top}
\mathbf{M}
\boldsymbol{\eta}*{c,k}
+
q_l
e_{l,k}^2
+
\mathbf{u}_k^{\top}
\mathbf{R}_u
\mathbf{u}_k
\right)
+
\ell_N.
]

The terminal cost may be chosen as

[
\ell_N
======

q_{c,N}
\boldsymbol{\eta}*{c,N}^{\top}
\mathbf{M}
\boldsymbol{\eta}*{c,N}
+
q_{l,N}
e_{l,N}^2.
]

---

## 8. Path-Constrained Extension

For path-constrained tasks, the trajectory-normal contouring error should be distinguished from the path-normal error.

The SE(3) contouring error is

[
\boldsymbol{\eta}_c.
]

It measures deviation from the desired pose trajectory.

The path-normal error may be defined as

[
e_n
===

\mathbf{n}^{\top}
\left(
\mathbf{p}
----------

\mathbf{p}_{\mathrm{path}}
\right),
]

where

[
\mathbf{n}\in\mathbb{R}^3
]

is the path normal.

If force sensing is available, the normal force error can be defined as

[
e_f
===

## F_n

F_d.
]

A path-constrained cost can therefore be written as

[
\ell
====

q_c
\boldsymbol{\eta}_c^{\top}
\mathbf{M}
\boldsymbol{\eta}_c
+
q_l
e_l^2
+
q_f
\left(
F_n-F_d
\right)^2
+
\mathbf{u}^{\top}
\mathbf{R}_u
\mathbf{u}.
]

Path constraints can also be enforced as a constraint:

[
F_{\min}
\leq
F_n
\leq
F_{\max}.
]

Or, in geometric path form,

[
e_n
\leq
0.
]

The key distinction is:

[
\boldsymbol{\eta}_c
]

is the trajectory-normal error in (SE(3)), while

[
e_n
\quad\text{or}\quad
e_f
]

describes path validity.

---

## 9. Algorithm Summary

Given:

[
\mathbf{T},
\qquad
\mathbf{T}_d(\theta),
\qquad
\frac{\partial \mathbf{T}_d(\theta)}{\partial \theta},
\qquad
\mathbf{M},
]

compute:

### Step 1: Relative pose

[
\mathbf{T}_e(\theta)
====================

\mathbf{T}_d(\theta)^{-1}
\mathbf{T}.
]

### Step 2: Lie-log pose error

[
\boldsymbol{\eta}(\theta)
=========================

\log
\left(
\mathbf{T}_e(\theta)
\right)^{\vee}.
]

### Step 3: Left-trivialized reference tangent

[
\boldsymbol{\tau}(\theta)
=========================

\left(
\mathbf{T}_d(\theta)^{-1}
\frac{\partial \mathbf{T}_d(\theta)}{\partial \theta}
\right)^{\vee}.
]

### Step 4: Lag projection coefficient

[
\delta\theta_l(\theta)
======================

\frac{
\boldsymbol{\tau}(\theta)^{\top}
\mathbf{M}
\boldsymbol{\eta}(\theta)
}{
\boldsymbol{\tau}(\theta)^{\top}
\mathbf{M}
\boldsymbol{\tau}(\theta)
}.
]

### Step 5: Lag-direction error vector

[
\boldsymbol{\eta}_l(\theta)
===========================

\delta\theta_l(\theta)
\boldsymbol{\tau}(\theta).
]

### Step 6: Contouring error vector

[
\boldsymbol{\eta}_c(\theta)
===========================

## \boldsymbol{\eta}(\theta)

\boldsymbol{\eta}_l(\theta).
]

### Step 7: Scalar errors

[
e_l(\theta)
===========

\frac{
\boldsymbol{\tau}(\theta)^{\top}
\mathbf{M}
\boldsymbol{\eta}(\theta)
}{
\sqrt{
\boldsymbol{\tau}(\theta)^{\top}
\mathbf{M}
\boldsymbol{\tau}(\theta)
}
},
]

[
e_c(\theta)
===========

\sqrt{
\boldsymbol{\eta}_c(\theta)^{\top}
\mathbf{M}
\boldsymbol{\eta}_c(\theta)
}.
]

---

## 10. Pseudocode

```python
def se3_mpcc_error_decomposition(T, T_ref, dT_ref_dtheta, M):
    """
    Compute SE(3)-based MPCC lag and contouring errors.

    Inputs:
        T:              current pose in SE(3), shape (4, 4)
        T_ref:          reference pose T_d(theta), shape (4, 4)
        dT_ref_dtheta:  derivative of reference pose w.r.t. theta, shape (4, 4)
        M:              SPD metric matrix, shape (6, 6)

    Outputs:
        eta:            Lie-log pose error, shape (6,)
        tau:            left-trivialized reference tangent, shape (6,)
        delta_theta_l:  progress-domain lag coefficient
        eta_l:          lag-direction error vector, shape (6,)
        eta_c:          contouring error vector, shape (6,)
        e_l:            scalar metric lag error
        e_c:            scalar metric contouring error
    """

    # Step 1: relative pose
    T_e = inv(T_ref) @ T

    # Step 2: Lie-log pose error
    eta = vee(log_SE3(T_e))

    # Step 3: left-trivialized reference tangent
    tau = vee(inv(T_ref) @ dT_ref_dtheta)

    # Step 4: metric projection
    denom = tau.T @ M @ tau
    delta_theta_l = (tau.T @ M @ eta) / denom

    # Step 5: lag vector
    eta_l = delta_theta_l * tau

    # Step 6: contouring vector
    eta_c = eta - eta_l

    # Step 7: scalar errors
    e_l = (tau.T @ M @ eta) / sqrt(denom)
    e_c = sqrt(eta_c.T @ M @ eta_c)

    return eta, tau, delta_theta_l, eta_l, eta_c, e_l, e_c
```

If the analytic derivative is unavailable, replace Step 3 with:

```python
T_ref_next = reference_pose(theta + dtheta)
tau = vee(log_SE3(inv(T_ref) @ T_ref_next)) / dtheta
```

---

## 11. Important Implementation Notes

### 11.1 Local validity

This method is local. The Lie logarithm

[
\log
\left(
\mathbf{T}_d(\theta)^{-1}
\mathbf{T}
\right)
]

is reliable when the relative pose is close to the identity.

For rotations, numerical issues may occur when the rotation angle approaches

[
\pi.
]

Therefore, the controller should operate in the local tracking regime.

### 11.2 Consistent trivialization

This implementation uses the left-trivialized convention:

[
\boldsymbol{\eta}
=================

\log
\left(
\mathbf{T}_d^{-1}\mathbf{T}
\right)^{\vee},
]

[
\boldsymbol{\tau}
=================

\left(
\mathbf{T}_d^{-1}
\frac{\partial \mathbf{T}_d}{\partial \theta}
\right)^{\vee}.
]

Do not mix this with the right-trivialized convention

[
\frac{\partial \mathbf{T}_d}{\partial \theta}
\mathbf{T}_d^{-1}.
]

Mixing left- and right-trivialized quantities makes the projection geometrically inconsistent.

### 11.3 Metric choice

The metric

[
\mathbf{M}
]

determines how translation and rotation are compared.

A simple choice is

[
\mathbf{M}
==========

\begin{bmatrix}
w_p\mathbf{I}_3 & \mathbf{0}\
\mathbf{0} & w_R\mathbf{I}_3
\end{bmatrix}.
]

A larger (w_p) penalizes translational error more strongly.

A larger (w_R) penalizes rotational error more strongly.

### 11.4 SE(3) geodesic limitation

This method uses the Lie logarithm as a local coordinate map. It should not be claimed as a globally exact Riemannian geodesic decomposition on (SE(3)).

A precise description is:

[
\text{local, left-trivialized, metric-dependent SE(3) error decomposition}.
]

---

## 12. Output Interpretation

The algorithm returns:

[
\boldsymbol{\eta}
]

the full local pose error in (\mathfrak{se}(3));

[
\boldsymbol{\eta}_l
]

the component of the pose error along the reference trajectory direction;

[
\boldsymbol{\eta}_c
]

the component of the pose error orthogonal to the reference trajectory direction;

[
e_l
]

the scalar lag error;

[
e_c
]

the scalar contouring error.

The core property is

[
\boldsymbol{\eta}
=================

\boldsymbol{\eta}_l
+
\boldsymbol{\eta}_c,
]

with

[
\boldsymbol{\tau}^{\top}
\mathbf{M}
\boldsymbol{\eta}_c
===================

0.

]

This is the SE(3) analogue of the lag-contouring error decomposition used in Euclidean MPCC.

```
```

