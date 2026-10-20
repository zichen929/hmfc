# HMFC: MATLAB Simulations of Hybrid Motion/Force Control

[English](README.md) | [简体中文](README.zh.md)

This project uses a two-link robot in a vertical plane with frictionless rigid contact to numerically verify full-information control (FIC), non-adaptive control with a fixed inaccurate normal (NAC), and immersion and invariance (I&I) adaptive control (IAC). It compares motion errors, contact forces, normal estimates, and joint torques, and checks the implementations against constrained dynamics and theoretical identities.

Only base MATLAB is required. The project has been verified in **MATLAB R2026a**. Simulink, Robotics Toolbox, and Optimization Toolbox are not required. The theoretical source is `obsidian-vault/04-writing/tcst26/TCST26E3.md` in the neighboring Obsidian repository; that repository is not a runtime dependency.

## Quick start

Set the MATLAB working directory to the project root, then choose a simulation:

```matlab
% Standard case: rerun and compare FIC 1, NAC 1, and IAC 1, with +5° initial error
comparison = fic1_nac1_iac1_simulation;

% Max angle case: sector near the half-width supremum, with +15° initial error
wide = fic1_nac1_iac1_max_angle_simulation;

% Run NAC 1 only
nacResults = nac1_simulation;

% Run IAC 1 only
iacResults = iac1_simulation;

% Run FIC 1–4 and check pairwise equivalence of the four implementations
ficResults = fic1234_simulation;
```

By default, figures are displayed and results are saved. The three-controller entry points perform fresh, separate ODE integrations, check the common conditions, and compare each controller's main regulation case. Individual controllers can also run independently.

## Controllers and entry points

| Controller | Normal information and control approach |
| --- | --- |
| FIC 1–4 | The true normal is known. Different reference coordinates and error representations are used; equivalence is checked for the same task. |
| NAC 1 | Uses the FIC 1 law with the true normal replaced by a fixed initial estimate, without an angle update. |
| IAC 1 | Starts with an inaccurate normal, integrates an internal estimate, and uses a momentum correction to construct and adapt the effective estimate. |

| Purpose | MATLAB entry point | Default result directory |
| --- | --- | --- |
| FIC 1 only | `fic1_simulation` | `results/fic1/` |
| FIC 2, 3, or 4 only | `fic2_simulation`, `fic3_simulation`, `fic4_simulation` | `results/fic2/`, `fic3/`, or `fic4/`, respectively |
| Compare FIC 1 and FIC 2 | `fic12_simulation` | `results/fic12/` |
| Compare FIC 1–3 | `fic123_simulation` | `results/fic123/` |
| Compare FIC 1–4 | `fic1234_simulation` | `results/fic1234/` |
| NAC 1 only | `nac1_simulation` | `results/nac1/` |
| IAC 1 only | `iac1_simulation` | `results/iac1/` |
| Standard +5° three-controller comparison | `fic1_nac1_iac1_simulation` | `results/fic1_nac1_iac1/` |
| Max angle / +15° three-controller comparison | `fic1_nac1_iac1_max_angle_simulation` | `results/fic1_nac1_iac1_max_angle/` |

FIC 1–4 share the FIC model and verification code. NAC 1 and IAC 1 each contain their own independent model and controller implementation. The three-controller entry points handle execution, condition matching, and presentation.

## Common model and conditions

| Item | Setting |
| --- | --- |
| Robot | Vertical planar 2R arm; link lengths 0.6 and 0.5 m; masses 2 and 1.5 kg |
| Centers of mass and moments of inertia | Uniform rods; gravity, Coriolis terms, and configuration-dependent inertia are retained explicitly |
| True contact surface | Horizontal line `y=0.4 m`, with true normal `[0;1]` |
| Initial state | Contact point `[0.6;0.4] m`; zero initial velocity in the main case; contact already established |
| Regulation reference | `[0.68;0.4] m`, an 8 cm displacement along the surface |
| Motion gains and desired force | `kp=25`, `kv=10`, `fd=10 N`; the main three-controller comparison uses `kf=0` |
| IAC parameters | `gamma=0.4`; sector center at 0° |
| Integration and output | `ode45`; 6 s by default; 0.002 s output spacing; `MaxStep=0.01` |
| Original certified operating region | Tangential displacement `x_t` in (−0.04,0.16) m and elbow angle `q2` in (70°,115°) |

The ODE uses minimal coordinates on the true contact surface to avoid normal constraint drift. The reaction is solved from the robot dynamics and rigid constraint together; it is not assigned the desired force, and no contact penalty spring is introduced. IAC integrates the internal state `hat_theta` and computes the effective estimate from the actual momentum correction, rather than directly integrating the derived parameter-error equation.

The individual entry points also run supplementary checks: FIC includes regulation, tracking, and `kf=0,2`; NAC includes regulation with both force gains; IAC includes the opposite initial tilt, manifold invariance, and regulation at the initial position. Three-controller comparison figures and summary tables select only the respective main regulation cases.

## Standard and max angle cases

| Item | Standard case | Max angle case |
| --- | --- | --- |
| Fixed NAC angle / initial effective IAC angle | +5° | +15° |
| Normal angle used by FIC | True angle, 0° | True angle, 0° |
| IAC sector half-width | 6° | Approximately 16.9812372138° |
| Sector configuration | Default closed sector | Supremum of approximately 16.9812382138°, minus a positive `1e-6°` margin |
| NAC check of the original certified region | An exit raises an error | The original region is retained; physical integration continues and the exit is recorded explicitly |

The sector supremum is computed automatically from the current inertia bounds. At equality, the uniform contact-force lower bound becomes zero, so the max angle case keeps a positive margin on the admissible side. This changes the sector and initial estimate for that case without changing the standalone or standard comparison defaults.

In the verified +15° main case, NAC 1 leaves the original certified region at approximately 0.400 s and then continues on the same exact, reachable, nonsingular robot model. A region exit means leaving the region used for the theoretical analysis; it does not itself imply contact loss or divergence. The original region's guarantees cannot be applied directly to the NAC trajectory after its exit. IAC 1 and FIC 1 continue to enforce the original region checks.

Read these fields together in the three-controller return value:

```matlab
wide.summary                      % Errors, contact forces, angles, and region status
wide.allChecksPassed              % Whether enabled numerical consistency checks passed
wide.allStayedInOperatingRegion   % Whether all three stayed in the original region
wide.nac1.cases(1).firstRegionExitTime
wide.angleAnalysis                % Sector calculation for the max angle case
```

`allChecksPassed=true` does not imply `allStayedInOperatingRegion=true`. In the +15° case, NAC's region status is false, as noted in the figures and reports.

## Headless execution and custom outputs

The NAC, IAC, and three-controller entry points accept an options struct:

```matlab
% Save figures without opening figure windows
comparison = fic1_nac1_iac1_simulation(struct('visible','off'));
wide = fic1_nac1_iac1_max_angle_simulation(struct('visible','off'));

% Disable plotting while still saving results
comparison = fic1_nac1_iac1_simulation(struct('makePlots',false));

% Select a three-controller output directory
comparison = fic1_nac1_iac1_simulation( ...
    struct('outputDir',fullfile(pwd,'results','custom_comparison')));

% IAC 1 only: compute and verify without saving or plotting
iacResults = iac1_simulation(struct('makePlots',false,'saveResults',false));
```

FIC entry points accept positional arguments `makePlots, outputDir`:

```matlab
ficResults = fic1234_simulation(false);
ficResults = fic1234_simulation(true,fullfile(pwd,'results','custom_fic'));
```

To run from a terminal in the project root:

```bash
matlab -batch "comparison = fic1_nac1_iac1_simulation(struct('visible','off'));"
```

See the documentation below for all options and controller equations.

## Results and verification

Results are generated in each entry point's `results/` subdirectory. The directory is ignored by `.gitignore`. Reusing an output directory updates files with matching names.

The three-controller directory mainly contains:

- `comparison.png`, `comparison.fig`: comparisons of position, motion error, contact force, and normal angle.
- `summary.csv`, `verification.txt`: main-case metrics and check reports for the three controllers.
- `fic1_primary.csv`, `nac1_primary.csv`, `iac1_primary.csv`: main-case samples with a common set of fields.
- `fic1_nac1_iac1_results.mat`: complete results, including `fic1`, `nac1`, `iac1`, `primary`, and `summary`.
- `fic1/`, `nac1/`, `iac1/`: raw outputs from the three independently executed simulations.
- `initial_angle_analysis.txt`: an additional max angle output recording the supremum, actual half-width, and numerical margin.

The combined runner assigns a separate subdirectory to each constituent simulation, preserving standalone default outputs. The previous +6° max angle results are archived in `results/fic1_nac1_iac1_max_angle_6deg/`.

Built-in checks cover joint and Cartesian dynamics residuals, contact constraints, positive reaction, motion-error identities, operating regions, and controller-specific predictions: analytical error solutions and implementation equivalence for FIC; nonlinear equilibrium points for NAC; and the effective angle-error equation, exponential envelope, and manifold invariance for IAC. The three-controller runner also matches model parameters, initial states, references, gains, and output sampling times.

The model assumes exactly known dynamics, ideal actuators, frictionless rigid contact, and exact scalar force measurements where required. Analysis starts with established contact and zero normal velocity. It excludes contact establishment, impacts, mode transitions after contact loss, measurement noise, and torque saturation.

## Comparison previews

Figure snapshots are stored alongside the documentation in `docs/figures/`, independently of the Git-ignored `results/` directory. See each detailed guide for its full result gallery; FIC groups can be expanded. Simulations still write raw outputs to `results/`.

**Standard +5° three-controller comparison**

![Standard +5° three-controller comparison](docs/figures/fic1_nac1_iac1/comparison.png)

**Max angle / +15°: NAC leaves the original region; IAC stays inside and converges**

![Max angle / +15°: NAC leaves the original region; IAC stays inside and converges](docs/figures/fic1_nac1_iac1_max_angle/comparison.png)

## Files and detailed documentation

| File | Purpose |
| --- | --- |
| `fic_simulation.m` | Shared model, control laws, integration, plotting, and verification for FIC 1–4 |
| `fic_compare_simulation.m` | Pairwise FIC comparisons and equivalence checks |
| `fic*_simulation.m` | Individual and combined FIC entry points |
| `nac1_simulation.m` | Independent NAC 1 model, controller, and verification |
| `iac1_simulation.m` | Independent IAC 1 model, adaptive law, and verification |
| `fic1_nac1_iac1_simulation.m` | Three-controller execution, condition matching, and comparison outputs |
| `fic1_nac1_iac1_max_angle_simulation.m` | Comparison configuration with a near-supremum sector and +15° initial angle |

- [FIC 1–4: equations, conditions, and equivalence checks](docs/FIC.md)
- [NAC 1: fixed normal, steady bias, and options](docs/NAC1.md)
- [IAC 1: momentum correction, adaptive law, and sector conditions](docs/IAC1.md)
- [FIC 1 / NAC 1 / IAC 1: standard and max angle comparisons](docs/FIC1_NAC1_IAC1.md)
