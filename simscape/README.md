# HMFC Simscape simulations

[English](README.md) | [简体中文](docs/zh-cn/README.md)

This folder contains the native Simscape Multibody FIC1 model, controller, builder, batch comparison, tests, documentation, result snapshots, saved outputs and generated files. The physical arm solves rigid-contact joint dynamics; each batch run independently integrates the numerical FIC1 reference for comparison.

Open [hmfc.prj](../hmfc.prj) for the complete project, or enter `simscape/` and initialize this simulation family:

```matlab
setup_simscape;
open_system('hmfc_fic1_simscape.slx');
physical = fic1_simscape_simulation;
run_simscape_tests;
```

`setup_simscape` is repeatable, does not load a model or change the working folder, and loads the required numerical source paths. Initialization does not require Simscape. Physical execution requires Simulink, Simscape and Simscape Multibody. Shared parameters and the ODE reference remain in `../numerical/`.

```text
simscape/
├── hmfc_fic1_simscape.slx          # Ready-to-run physical model
├── build_hmfc_simscape.m          # Rebuild model from readable source
├── fic1_simscape_controller.m     # Embedded controller source
├── fic1_simscape_simulation.m     # Four-case batch run and ODE comparison
├── plot_fic1_simscape_results.m   # Saved-result plots
├── hmfc_simscape_root.m          # Resolve paths independently of pwd
├── setup_simscape.m
├── run_simscape_tests.m
├── tests/                        # Setup and physical integration checks
├── docs/                         # English/Chinese guides and figures
├── results/                      # Saved outputs; ignored by Git
├── cache/                        # Simulink cache; ignored by Git
└── codegen/                      # Generated code; ignored by Git
```

Default batch results go to `simscape/results/fic1_simscape/`, including `ode_reference/`. `options.outputDir` still selects custom outputs. Initialization and the MATLAB Project both put cache and generated code under `simscape/cache/` and `simscape/codegen/`. These generated folders, tests and documentation are excluded from the runtime search path; generated folders are also excluded from managed project files.

Original results and figures have been moved without changing their contents. Historical MAT files retain their original provenance metadata. Old root-level cache files are preserved under `simscape/cache/legacy_root/`; new builds use the configured cache folder.

The MATLAB Project provides physical model and comparison shortcuts in the Simscape group, plus a test shortcut in Tests. The physical integration test uses temporary outputs and closes only the model it loads. Close an already-open physical model before running that test. The test is skipped when Simscape Multibody is unavailable.

See the [detailed physical model and verification guide](docs/FIC1_SIMSCAPE.md) for construction, equations, solver settings, logging and result galleries.
