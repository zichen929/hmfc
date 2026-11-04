# Shared numerical simulation interface

[English](NUMERICAL_API.md) | [简体中文](zh-cn/NUMERICAL_API.md)

FIC, NAC and IAC accept one scalar `options` struct. Their robot model is shared; their controller laws, contact solves and integrations remain separate. All three solve the original joint dynamics together with the rigid acceleration constraint. Shared helpers handle parameter grouping, option validation, public metrics, region reporting, summary tables and CSV columns.

```matlab
options = struct('makePlots',false,'saveResults',false);
ficOptions = options;
ficOptions.controllerIds = 1;
fic = fic_simulation(ficOptions);
nac = nac1_simulation(options);
iac = iac1_simulation(options);

run = iac.runs(1);
plot(run.t,run.error);
disp(run.metrics);
disp(run.checks);
```

## Derivative and desired-value names

In symbolic names, each leading `d` denotes one time derivative; a trailing `d` denotes the desired value. Descriptive public names such as `position`, `velocity` and `referenceVelocity` retain their full-word meanings.

| Actual signal | Desired signal | Meaning |
| --- | --- | --- |
| `q`, `dq`, `ddq` | `qd`, `dqd`, `ddqd` | Joint position, velocity, acceleration |
| `p`, `dp`, `ddp` | `pd`, `dpd`, `ddpd` | Cartesian position, velocity, acceleration |
| `xt`, `dxt`, `ddxt` | `xtd`, `dxtd`, `ddxtd` | Intrinsic position, velocity, acceleration |
| `f` / `force`, `df` | `fd`, `dfd` | Force and its derivative; `fd` is the desired force |

Model derivatives are `dJ`, `dM`, `dK` and `dLambda`; `dJdq` explicitly means `dJ*dq`. FIC exposes `dV`, with the positive-derivative metric `maxPositive_dV`. IAC uses `dhatTheta`, `dmomentum`, `dz` and `dzTheory`. `dstate` is the ODE state derivative; `timeStep` is sample spacing. Cartesian input-reference fields are `pInputd`, `dpInputd` and `ddpInputd` so their desired-value suffix remains last.

The public joint-acceleration field is now `runs(k).ddq` in the numerical suite and `cases(k).ddq` in the physical suite. Simscape exposes the `ddq` port and `ddq_log` workspace signal; its CSV columns use `ddq1_radps2`, `ddq2_radps2`, `pxd_m` and `pyd_m`.

Existing MAT/CSV archives retain their original schema. Regenerate results for the new names. For scripts reading old results, migrate `qdd→ddq`, `Jdot→dJ`, `Mdot→dM`, `LambdaDot→dLambda`, `Jdotdq→dJdq`, `Vdot→dV`, `maxPositiveVdot→maxPositive_dV`, `zDot→dz`, `zDotTheory→dzTheory`, `lambdaDotFiniteDifference→dLambdaFiniteDifference`, and `pdInput/dpdInput/ddpdInput→pInputd/dpInputd/ddpInputd`. The desired references `pd/dpd/ddpd`, `xtd/dxtd/ddxtd` and `fd` already follow the rule.

## Shared robot model

| Function | Responsibility |
| --- | --- |
| `hmfc_forward_kinematics(q,robot)` | Cartesian endpoint position |
| `hmfc_inverse_kinematics(position,robot)` | Positive-elbow IK and physical workspace validation |
| `hmfc_jacobian(q,robot)` | Endpoint Jacobian |
| `hmfc_robot_model(q,dq,robot)` | `M`, `dM`, `C`, `G`, `J`, `dJ`, `dJdq`, `Lambda`, `dLambda`, `eta`, `q`, `dq` |
| `hmfc_robot_state(position,velocity,robot)` | Reconstruct joint state and the model from Cartesian measurements |

These files live in `numerical/shared/` and contain no contact normal, control gain or certificate-region limit. Each numerical runner retains its own contact solver and region policy, including NAC's option to continue outside the original certificate region. Initial joint-state setup also uses the shared IK and Jacobian. The analytical, equilibrium and finite-difference checks remain independent references for the common model.

## Per-step evaluation

Each public entry point validates its options and delegates a single controller's simulation to the local `run_controller(options)`. Plotting uses `make_plots(results)` in all three runners; FIC retains `plot_case_pair` and `plot_diagnostics` as its internal plotting helpers. NAC and IAC write their text verification reports through `write_report(results)`; IAC's former local name was `write_summary`.

Every runner uses a local `evaluate` function for both ODE integration and sampled postprocessing. IAC also uses it for its terminal events, with the current case's reference. The common flow is:

1. `measured_state(x,tangentialVelocity,robot,plant)` returns Cartesian position, velocity and the reconstructed robot model. All runners use `p0+T*x` and `T*tangentialVelocity` directly, while retaining their own region policies.
2. `reference_command(t,task,mode,controller)` generates the controller's native reference inputs. FIC 1/4 receive Cartesian vectors; FIC 2/3 receive intrinsic scalars. Coordinate conversions remain inside the FIC controller. NAC and IAC generate Cartesian regulation references with zero derivatives.
3. `controller_command` returns reference/error signals, `up`, `forceCommand`, `F` and `tau`. It also returns `F0` and the vector `forceFeedback`, describing the affine command `F=F0+forceFeedback*force` for the contact solve.
4. `contact_plant(cmd,model,n)` solves the original joint dynamics and rigid constraint using `F0`, `forceFeedback` and `model.dJdq`. FIC/NAC first construct the force-independent command, solve the instantaneous feedback loop, and then reevaluate the controller with the solved scalar force. IAC has zero `forceFeedback` and additionally evaluates its estimator update.

## Options and grouped parameters

| Option | Default | Meaning |
| --- | --- | --- |
| `makePlots` | `true` | Create figures independently of saving |
| `saveResults` | `true` | Save MAT, CSV, reports and enabled figures; `false` creates no output directory or files |
| `visible` | `'on'` | Figure visibility, `'on'` or `'off'` |
| `outputDir` | Controller-specific folder under `numerical/results/` | Destination when saving |
| `duration` | `6` | Seconds; at least 6 to retain the terminal-convergence checks |
| `initialVelocity` | `0` | Main-case tangential velocity in m/s |
| `controllerIds` | `1:4` | FIC only: distinct controller IDs in requested order |

NAC retains its fixed-angle and region-enforcement options. IAC retains its initial-angle and sector options. Its invariant-manifold check intentionally uses 0.02 m/s, and its at-rest check intentionally uses zero velocity and zero target displacement.

Every standalone result contains `name`, `source`, `options`, `robot`, `plant`, `controller`, `task`, `odeOptions`, `runs`, `summary`, `checks` and `allChecksPassed`. `robot` holds physical link parameters; `plant` holds true contact geometry and domain; `controller` holds gains and controller-specific data; `task` holds the reference origin, axis and target. Only FIC's controller receives true geometry.

For multiple FIC IDs, `runs` collects all selected runs. The result additionally contains the corresponding `ficN` results and pairwise equivalence checks. `controller.id` identifies a standalone FIC controller. Optional diagnostics missing from another controller are empty in the collected array; each `ficN` retains its complete native fields.

## Common run fields

All sampled arrays use one row per output time. SI units apply; angles are radians.

| Field | Shape / meaning |
| --- | --- |
| `controller`, `name`, `mode`, `kf` | Controller label, scenario name, reference mode and force gain |
| `t` | N×1 output times |
| `state` | N×2 `[x_t, dx_t]`; IAC uses N×3 `[x_t, dx_t, hat_theta]` |
| `initialState` | Column vector equal to the first state row transposed |
| `x` | N×1 tangential displacement, equal to `state(:,1)` |
| `position`, `velocity`, `acceleration` | N×2 Cartesian plant signals |
| `reference`, `referenceVelocity`, `referenceAcceleration` | N×2 Cartesian references |
| `positionError`, `velocityError` | N×2 reference minus measured signal |
| `error` | N×1 signed tangential position error |
| `force`, `forceCommand` | N×1 physical reaction and scalar force command |
| `angle` | N×1 true FIC normal angle, fixed NAC angle or effective IAC estimate |
| `tau`, `q`, `dq`, `ddq` | N×2 joint torque, position, velocity and acceleration |
| `jointResidual`, `cartesianResidual` | N×2 signed joint-dynamics residual (N m) and Cartesian-dynamics residual (N) |
| `operatingRegionSatisfied`, `firstRegionExitTime` | Original certificate-region status; exit time is `NaN` if no exit |
| `metrics`, `checks` | Common metrics plus controller-specific metrics; verification results including `passed` |

Common checks include `maxJointResidual`, `maxCartesianResidual`, `maxGap`, `maxNormalVelocity`, `maxNormalAcceleration`, `operatingRegionSatisfied` and `passed`.

The two dynamics residuals are `M*ddq+C*dq+G-tau-J'*n*force` and `Lambda*acceleration+eta-F-n*force`. Their common maximum checks take the infinity norm of each row, then the maximum over time: `max(vecnorm(run.jointResidual,inf,2))` and likewise for `cartesianResidual`.

FIC and IAC store the sampled minimum and maximum eigenvalues of `Lambda` as `minLambda` and `maxLambda`. Maximum absolute contact gap is named `maxGap`, including FIC's additional metrics. NAC records the force difference `forceResidual` in N and checks `maxForceResidual < 1e-9` N. IAC records the dimensionless `forceRatioResidual = rho-rhoFormula` and checks `maxForceRatioResidual < 1e-10`. These are distinct quantities with separate tolerances; they are not the force-tracking error relative to `fd`.

Common metrics include `finalError`, `finalForce`, `minForce`, `maxForceError`, `tailRmsError`, `finalTangentialVelocity`, `finalAngleError`, `peakTorque`, `operatingRegionSatisfied` and `firstRegionExitTime`. The exit time is in seconds and uses the same field name in `run`, `metrics` and standalone `summary` tables. `summary` adds `controller`, `name`, `mode` and `kf` columns to the per-run metrics.

FIC stores its analytical error trajectories as `analyticPositionError` and `analyticVelocityError`. The corresponding maximum deviations are `maxAnalyticPositionError` and `maxAnalyticVelocityError`. Its error-dynamics identity uses the sampled `errorDynamicsResidual` and the maximum check `maxErrorDynamicsResidual`.

FIC retains analytical-solution, gain-invariance, intrinsic-coordinate and pairwise-equivalence checks. NAC retains nonlinear equilibrium predictions and the zero-mismatch analytical check. IAC retains `hatTheta`, `beta`, `barTheta`, `angleError`, the angle-error derivatives, envelope and certificate, with momentum, manifold-invariance and sector checks. `angleError` is a diagnostic signal; the ODE integrates `hatTheta`.

The three-controller runners expose their three primary cases as `runs` and retain complete standalone results in `fic1`, `nac1` and `iac1`. Their comparison summary retains its SI-labelled comparison columns. Both runners support `saveResults=false`.

## CSV and migration

All standalone CSVs start with the same columns:

```text
time_s,x_t_m,dx_t_mps,motion_error_m,force_N,force_command_N,normal_angle_rad,tau1_Nm,tau2_Nm
```

IAC appends `hat_theta_rad`, `beta_rad`, `bar_theta_rad` and `angle_error_rad`.

For older MAT results, FIC's `dynamicResidual` becomes `jointResidual`; NAC's `plantResidual` becomes `jointResidual`; IAC's `plantResidual` becomes `cartesianResidual`. Both common residual fields now store two signed components per sample. Older NAC/IAC residuals were scalar infinity norms; compute those from new results with `vecnorm(run.jointResidual,inf,2)` or `vecnorm(run.cartesianResidual,inf,2)`. FIC's metric `maxJointDynamicsResidual` is now `maxJointResidual`, and its joint/Cartesian maximum residual metrics use the same infinity-norm convention as NAC and IAC. Saved archives retain their original fields.

IAC's `minEig/maxEig` become `minLambda/maxLambda`. FIC's `metrics.maxContactPositionError` and its copied check become `maxGap`. The former check `maxForceIdentityResidual` becomes `maxForceResidual` for NAC (N) and `maxForceRatioResidual` for IAC (dimensionless), preserving their respective `1e-9` N and `1e-10` tolerances. IAC now also stores the sampled `forceRatioResidual` used by that check.

FIC's `exactEp/exactEv` become `analyticPositionError/analyticVelocityError`; `maxAnalyticError` becomes `maxAnalyticPositionError`; `errorResidual` becomes `errorDynamicsResidual`. The common metric and standalone summary column `firstRegionExitTime_s` become `firstRegionExitTime`, matching the sampled run metadata; values remain in seconds. These name changes also apply to copied FIC check fields. Existing MAT/CSV archives retain their original names. The NAC/IAC `rho` diagnostics retain their existing definitions.

The positional FIC API has been replaced by `fic_simulation(struct('controllerIds',ids,...))`. Use `runs` instead of `cases`, grouped parameters instead of `config`, and `controller` instead of NAC's former `control`. FIC/NAC/IAC all use `state`; IAC's former `z` diagnostic is now `angleError`. FIC's former `p/dp/pdd/pd/dpd/ep/ev/f/uf` fields are now `position/velocity/acceleration/reference/referenceVelocity/positionError/velocityError/force/forceCommand`. NAC uses the same reference and force-command names. The comparison runner's former `primary` field is now `runs`.
