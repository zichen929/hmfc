# Two-DOF WAM Experiments: Reproducing FIC1, NAC1, and IAC1

[English](wam7-exp-guide.md) | [简体中文](zh-cn/wam7-exp-guide.md)

## 1. Controller expressions

All variables below are expressed in a fixed two-dimensional task plane. `p,v` are the end-effector position and velocity, and `pd` is a fixed target on the contact line. The normal `n` points toward free space; `f≥0` is the scalar compressive force exerted by the environment on the robot. The dynamics convention is:

```text
M2*ddr + h2 = tau_a + J'*n*f
Lambda*ddp + eta = F + n*f
h2 = C2*dr + G2
```

Use the following angle convention, with radians internally:

```text
n(theta) = [-sin(theta); cos(theta)]
T(theta) = [ cos(theta); sin(theta)]
```

### FIC1: the true normal

```text
up    = ddpd + kv*(dpd-v) + kp*(pd-p)
uf    = fd + kf*(fd-f)
F     = eta + Lambda*up - n(theta_true)*uf
tau_a = J'*F
```

For the fixed-target regulation task, set `dpd=ddpd=0`. The main comparison uses `kf=0`.

### NAC1: a fixed inaccurate normal

Use the same law as FIC1, replacing only the normal:

```text
n_fix = n(theta_fix)                  # theta_fix remains constant
F     = eta + Lambda*up - n_fix*uf
tau_a = J'*F
```

The standard case uses `theta_fix-theta_true=+5°`, with no estimator state.

### IAC1: an adaptive effective normal

This is the fixed-target, constant-desired-force version implemented in the existing simulation, with `kf=0`. The nominal center `thetaC` and `Tc=T(thetaC)` remain fixed:

```text
up        = kp*(pd-p) - kv*v
beta      = -gamma*Tc'*Lambda*v
barTheta  = hatTheta + beta
nbar      = n(barTheta)
F         = eta + Lambda*up - nbar*fd
tau_a     = J'*F

dhatTheta = gamma*Tc'*(Lambda*up + dLambda*v + nbar*(f-fd))
hatTheta0 = barTheta0 + gamma*Tc'*Lambda0*v0
```

Integrate only `hatTheta`; use `barTheta` for the commanded direction. IAC1 does not use the true normal, true angle error, or measured acceleration online. Compute `dLambda` from the robot model and joint position/velocity.

## 2. Simulations to reproduce

The main entry point is [`fic1_nac1_iac1_simulation`](../fic1_nac1_iac1_simulation.m). Run it from the `hmfc` repository root:

```matlab
setup_project;
comparison = fic1_nac1_iac1_simulation;
```

All three controllers share the initial position, zero initial velocity, a fixed target on the contact line, and the gains: `8 cm` tangential target displacement, `kp=25 s^-2`, `kv=10 s^-1`, `fd=10 N`, `kf=0`, `6 s` duration, and `0.002 s` output spacing. FIC1 uses the true normal; NAC1 holds a `+5°` error; IAC1 starts with a `+5°` effective error, using `gamma=0.4 rad/(N·s)` and a nominal sector of `±6°`.

| Controller | Original 2R simulation at 6 s | Behavior to reproduce on the WAM |
| --- | --- | --- |
| FIC1 | Position error approximately 0; force 10 N | Motion/force control baseline with a known normal |
| NAC1 | Tangential error −56.656 mm; force 10.5691 N | Motion bias caused by a fixed normal error |
| IAC1 | Position error approximately 0; force approximately 10 N; effective angle error approximately 0 | Normal correction, with position and force approaching the baseline |

The controller sources are [`fic_simulation.m`](../fic_simulation.m) (`controllerIds=1`), [`nac1_simulation.m`](../nac1_simulation.m), and [`iac1_simulation.m`](../iac1_simulation.m). Results default to `numerical/results/fic1_nac1_iac1/`.

The supplementary large-angle comparison is [`fic1_nac1_iac1_max_angle_simulation`](../fic1_nac1_iac1_max_angle_simulation.m): the initial NAC/IAC error is `+15°`, and the original model's sector half-width is approximately `16.981237°`. NAC1 leaves the original certified region, while IAC1 still converges. Recompute this sector using the WAM inertia; do not transfer it directly.

The original simulations use ideal 2R parameters and already-established frictionless rigid contact. Reproduce the comparison trends on the WAM; predict its specific steady-state biases and convergence rates using the WAM model.

## 3. Required conversions from simulation to the existing code

### 3.1 Seven-joint WAM → two-DOF model → two-dimensional task coordinates

The existing code drives joints **2 and 4** (C++ indices **1 and 3**). Let `E=[e2,e4]` and `r=[q2;q4]`; fix the remaining joints at `q_lock`, with its active components set to zero. Use the fixed orthonormal task-plane basis `B=[ex,ez]`, after checking that the actual locked configuration produces this motion plane:

```text
q7 = q_lock + E*r
M2 = E'*M7*E
h2 = E'*h7(q7,E*dr)
J  = B'*Jv7*E
p  = B'*(p_tool_base-o_base)
v  = J*dr
```

`M7/h7` must include all seven links, the wrist, and the tool payload; locking joints does not remove these masses. `J` is an invertible 2×2 translational Jacobian. If the other joints are only held by PD control, this is an approximate model.

### 3.2 Gravity compensation → full task-space dynamics compensation

Use the same definitions as the simulation:

```text
K       = solve(J,I)
Lambda  = K'*M2*K
eta     = solve(J',h2) - Lambda*dJ*dr
dK      = -K*dJ*K
dLambda = dK'*M2*K + K'*dM2*K + K'*M2*dK
```

`dJ/dM2` are model time derivatives along the joint velocity. See [`hmfc_robot_model.m`](../shared/hmfc_robot_model.m) for the definitions, but replace the model parameters with those of the WAM. The existing `hybrid_control/dynamics.hpp` handles only the first four joints and scales acceleration, so it cannot directly serve as the full WAM7 model here.

All three controllers can equivalently output the total active-joint torque:

```text
tau_a = h2 + M2*solve(J,up-dJ*dr) - J'*n_used*uf
```

For IAC1, `n_used=nbar, uf=fd`. If the existing `wam.gravityCompensate()` remains enabled, subtract the gravity contribution actually added separately by the system:

```text
tau_user_a = tau_a - E'*g_builtin7
```

Keep the existing holding strategy for the other joints. Insert the computed `tau_user_a` directly into the seven-joint command; do not apply another `J^T` mapping.

### 3.3 Existing force feedback → positive scalar compressive force

The current FP feedback negates the force after frame conversion and uses the force exerted by the robot on the environment. The controllers above use the opposite convention:

```text
F_env_base = R_base_sensor*F_sensor_corrected    # branch before negation
F_env_2d  = B'*F_env_base
f         = n_cal'*F_env_2d                    # independently calibrated normal-force channel
fd        = +10 N
```

IAC1 receives only the scalar `f`, not `n_cal`. Under approximately frictionless contact, `norm(F_env_2d)` can also approximate the normal force; with tangential friction, the resultant magnitude is not an accurate normal force. Express sensor force, position, and the Jacobian in the same frame, and correct force readings for the tool payload.

### 3.4 Existing PD/PI and references → a shared regulation task

| Existing code | Required conversion |
| --- | --- |
| [`hybrid_fp_ft/controller.hpp`](../../../contact-control/include/hybrid_fp_ft/controller.hpp) | Replace the axis-selected PD/PI with the laws in Section 1; retain full two-dimensional inertia coupling and omit the original force integral |
| [`hybrid_ft/joint_torque.hpp`](../../../contact-control/include/hybrid_ft/joint_torque.hpp) | Accept active-joint torques, avoid duplicate gravity compensation, and retain the other joints' holding logic |
| [`src/hybrid_fp_ft.cpp`](../../../contact-control/src/hybrid_fp_ft.cpp) | Connect the model and positive compressive-force channel; replace move/sine references with a fixed target on the contact line, `pd=p0+T_cal*0.08` |

Use the same reference, initial state, and model for all three controllers. Match the fixed NAC1 angle to the initial effective IAC1 angle. The `kp/kv` units are `s^-2/s^-1`; do not directly reuse the original FP gains in `N/m` and `N·s/m`. Integrate IAC1 using the execution manager's actual period, and initialize it with `Lambda0,v0` from the same cycle.
