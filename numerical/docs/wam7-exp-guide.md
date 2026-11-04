# Two-DOF WAM Experiments: Reproducing FIC1, NAC1, and IAC1

[English](wam7-exp-guide.md) | [简体中文](zh-cn/wam7-exp-guide.md)

## 1. Controller expressions

All variables below are expressed in a fixed two-dimensional task plane. $p,v$ are the end-effector position and velocity, and $p_d$ is a fixed target on the contact line. The normal $n$ points toward free space; $f\geq0$ is the scalar compressive force exerted by the environment on the robot. The dynamics convention is:

$$
\begin{aligned}
M_2\ddot r+h_2 &= \tau_a+J^{\mathsf T}nf, \\
\Lambda\ddot p+\eta &= F+nf, \\
h_2 &= C_2\dot r+G_2.
\end{aligned}
$$

Use the following angle convention, with radians internally:

$$
\begin{aligned}
n(\theta) &= \begin{bmatrix}-\sin\theta \\ \cos\theta\end{bmatrix}, \\
T(\theta) &= \begin{bmatrix}\cos\theta \\ \sin\theta\end{bmatrix}.
\end{aligned}
$$

### FIC1: the true normal

$$
\begin{aligned}
u_p &= \ddot p_d+k_v(\dot p_d-v)+k_p(p_d-p), \\
u_f &= f_d+k_f(f_d-f), \\
F &= \eta+\Lambda u_p-n(\theta_{\mathrm{true}})u_f, \\
\tau_a &= J^{\mathsf T}F.
\end{aligned}
$$

For the fixed-target regulation task, set $\dot p_d=\ddot p_d=0$. The main comparison uses $k_f=0$.

### NAC1: a fixed inaccurate normal

Use the same law as FIC1, replacing only the normal:

$$
\begin{aligned}
n_{\mathrm{fix}} &= n(\theta_{\mathrm{fix}}), \qquad \dot\theta_{\mathrm{fix}}=0, \\
F &= \eta+\Lambda u_p-n_{\mathrm{fix}}u_f, \\
\tau_a &= J^{\mathsf T}F.
\end{aligned}
$$

The standard case uses $\theta_{\mathrm{fix}}-\theta_{\mathrm{true}}=+5^\circ$, with no estimator state.

### IAC1: an adaptive effective normal

This is the fixed-target, constant-desired-force version implemented in the existing simulation, with $k_f=0$. The nominal center $\theta_c$ and $T_c=T(\theta_c)$ remain fixed:

$$
\begin{aligned}
u_p &= k_p(p_d-p)-k_v v, \\
\beta &= -\gamma T_c^{\mathsf T}\Lambda v, \\
\bar\theta &= \hat\theta+\beta, \\
\bar n &= n(\bar\theta), \\
F &= \eta+\Lambda u_p-\bar n f_d, \\
\tau_a &= J^{\mathsf T}F, \\
\dot{\hat\theta} &= \gamma T_c^{\mathsf T}
\left[\Lambda u_p+\dot\Lambda v+\bar n(f-f_d)\right], \\
\hat\theta(0) &= \bar\theta(0)+\gamma T_c^{\mathsf T}\Lambda(0)v(0).
\end{aligned}
$$

Integrate only $\hat\theta$; use $\bar\theta$ for the commanded direction. IAC1 does not use the true normal, true angle error, or measured acceleration online. Compute $\dot\Lambda$ from the robot model and joint position/velocity.

## 2. Simulations to reproduce

The main entry point is [`fic1_nac1_iac1_simulation`](../fic1_nac1_iac1_simulation.m). Run it from the `hmfc` repository root:

```matlab
setup_project;
comparison = fic1_nac1_iac1_simulation;
```

All three controllers share the initial position, zero initial velocity, a fixed target on the contact line, and the gains: $8\,\mathrm{cm}$ tangential target displacement, $k_p=25\,\mathrm{s}^{-2}$, $k_v=10\,\mathrm{s}^{-1}$, $f_d=10\,\mathrm{N}$, $k_f=0$, $6\,\mathrm{s}$ duration, and $0.002\,\mathrm{s}$ output spacing. FIC1 uses the true normal; NAC1 holds a $+5^\circ$ error; IAC1 starts with a $+5^\circ$ effective error, using $\gamma=0.4\,\mathrm{rad}/(\mathrm{N}\cdot\mathrm{s})$ and a nominal sector of $\pm6^\circ$.

| Controller | Original 2R simulation at 6 s | Behavior to reproduce on the WAM |
| --- | --- | --- |
| FIC1 | Position error approximately 0; force 10 N | Motion/force control baseline with a known normal |
| NAC1 | Tangential error −56.656 mm; force 10.5691 N | Motion bias caused by a fixed normal error |
| IAC1 | Position error approximately 0; force approximately 10 N; effective angle error approximately 0 | Normal correction, with position and force approaching the baseline |

The controller sources are [`fic_simulation.m`](../fic_simulation.m) (`controllerIds=1`), [`nac1_simulation.m`](../nac1_simulation.m), and [`iac1_simulation.m`](../iac1_simulation.m). Results default to `numerical/results/fic1_nac1_iac1/`.

The supplementary large-angle comparison is [`fic1_nac1_iac1_max_angle_simulation`](../fic1_nac1_iac1_max_angle_simulation.m): the initial NAC/IAC error is $+15^\circ$, and the original model's sector half-width is approximately $16.981237^\circ$. NAC1 leaves the original certified region, while IAC1 still converges. Recompute this sector using the WAM inertia; do not transfer it directly.

The original simulations use ideal 2R parameters and already-established frictionless rigid contact. Reproduce the comparison trends on the WAM; predict its specific steady-state biases and convergence rates using the WAM model.

## 3. Required conversions from simulation to the existing code

### 3.1 Seven-joint WAM → two-DOF model → two-dimensional task coordinates

The existing code drives joints **2 and 4** (C++ indices **1 and 3**). Let $E=[e_2,e_4]$ and $r=[q_2,q_4]^{\mathsf T}$; fix the remaining joints at $q_{\mathrm{lock}}$, with its active components set to zero. Use the fixed orthonormal task-plane basis $B=[e_x,e_z]$, after checking that the actual locked configuration produces this motion plane:

$$
\begin{aligned}
q_7 &= q_{\mathrm{lock}}+Er, \\
M_2 &= E^{\mathsf T}M_7E, \\
h_2 &= E^{\mathsf T}h_7(q_7,E\dot r), \\
J &= B^{\mathsf T}J_{v,7}E, \\
p &= B^{\mathsf T}(p_{\mathrm{tool,base}}-o_{\mathrm{base}}), \\
v &= J\dot r.
\end{aligned}
$$

$M_7,h_7$ must include all seven links, the wrist, and the tool payload; locking joints does not remove these masses. $J$ is an invertible 2×2 translational Jacobian. If the other joints are only held by PD control, this is an approximate model.

### 3.2 Gravity compensation → full task-space dynamics compensation

Use the same definitions as the simulation:

$$
\begin{aligned}
K &= J^{-1}, \\
\Lambda &= K^{\mathsf T}M_2K, \\
\eta &= J^{-\mathsf T}h_2-\Lambda\dot J\dot r, \\
\dot K &= -K\dot JK, \\
\dot\Lambda &= \dot K^{\mathsf T}M_2K
+K^{\mathsf T}\dot M_2K+K^{\mathsf T}M_2\dot K.
\end{aligned}
$$

$\dot J,\dot M_2$ are model time derivatives along the joint velocity. See [`hmfc_robot_model.m`](../shared/hmfc_robot_model.m) for the definitions, but replace the model parameters with those of the WAM. The existing `hybrid_control/dynamics.hpp` handles only the first four joints and scales acceleration, so it cannot directly serve as the full WAM7 model here.

All three controllers can equivalently output the total active-joint torque:

$$
\tau_a=h_2+M_2J^{-1}(u_p-\dot J\dot r)
-J^{\mathsf T}n_{\mathrm{used}}u_f.
$$

For IAC1, $n_{\mathrm{used}}=\bar n,\quad u_f=f_d$. If the existing `wam.gravityCompensate()` remains enabled, subtract the gravity contribution actually added separately by the system:

$$
\tau_{\mathrm{user},a}=\tau_a-E^{\mathsf T}g_{\mathrm{builtin},7}.
$$

Keep the existing holding strategy for the other joints. Insert the computed $\tau_{\mathrm{user},a}$ directly into the seven-joint command; do not apply another $J^{\mathsf T}$ mapping.

### 3.3 Existing force feedback → positive scalar compressive force

The current FP feedback negates the force after frame conversion and uses the force exerted by the robot on the environment. The controllers above use the opposite convention:

$$
\begin{aligned}
F_{\mathrm{env,base}} &= R_{\mathrm{base,sensor}}F_{\mathrm{sensor,corrected}}, \\
F_{\mathrm{env}}^{(2)} &= B^{\mathsf T}F_{\mathrm{env,base}}, \\
f &= n_{\mathrm{cal}}^{\mathsf T}F_{\mathrm{env}}^{(2)}, \\
f_d &= +10\,\mathrm{N}.
\end{aligned}
$$

IAC1 receives only the scalar $f$, not $n_{\mathrm{cal}}$. Under approximately frictionless contact, $\lVert F_{\mathrm{env}}^{(2)}\rVert$ can also approximate the normal force; with tangential friction, the resultant magnitude is not an accurate normal force. Express sensor force, position, and the Jacobian in the same frame, and correct force readings for the tool payload.

### 3.4 Existing PD/PI and references → a shared regulation task

| Existing code | Required conversion |
| --- | --- |
| [`hybrid_fp_ft/controller.hpp`](../../../contact-control/include/hybrid_fp_ft/controller.hpp) | Replace the axis-selected PD/PI with the laws in Section 1; retain full two-dimensional inertia coupling and omit the original force integral |
| [`hybrid_ft/joint_torque.hpp`](../../../contact-control/include/hybrid_ft/joint_torque.hpp) | Accept active-joint torques, avoid duplicate gravity compensation, and retain the other joints' holding logic |
| [`src/hybrid_fp_ft.cpp`](../../../contact-control/src/hybrid_fp_ft.cpp) | Connect the model and positive compressive-force channel; replace move/sine references with a fixed target on the contact line, $p_d=p_0+T_{\mathrm{cal}}(0.08\,\mathrm{m})$ |

Use the same reference, initial state, and model for all three controllers. Match the fixed NAC1 angle to the initial effective IAC1 angle. The $k_p,k_v$ units are $\mathrm{s}^{-2},\mathrm{s}^{-1}$; do not directly reuse the original FP gains in $\mathrm{N}/\mathrm{m}$ and $\mathrm{N}\cdot\mathrm{s}/\mathrm{m}$. Integrate IAC1 using the execution manager's actual period, and initialize it with $\Lambda(0),v(0)$ from the same cycle.
