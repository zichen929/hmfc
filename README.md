# HMFC: Hybrid Motion/Force Control

[English](README.md) | [简体中文](docs/zh-cn/README.md)

This repository compares FIC, NAC1 and IAC1 on a planar two-link robot with rigid contact. All numerical simulation code, shared models, tests, documentation, example figures and numerical outputs are organized under [`numerical/`](numerical/README.md). The optional physical model, tests, documentation and generated files are organized under [`simscape/`](simscape/README.md).

Open [hmfc.prj](hmfc.prj) in MATLAB, or initialize from the repository root:

```matlab
setup_project;
comparison = fic1_nac1_iac1_simulation;
run_numerical_tests;
```

For base-MATLAB numerical work alone, enter `numerical/` and run `setup_numerical`. Use `setup_simscape` inside `simscape/` for physical simulation setup. All setup functions preserve the working folder. The MATLAB Project configures only the repository root, `numerical/`, `numerical/shared/` and `simscape/` as source paths. It labels both test suites correctly and provides numerical comparison, physical model, physical comparison and test shortcuts. Generated outputs are ignored by Git and excluded from managed project files.

```text
hmfc/
├── hmfc.prj, setup_project.m
├── numerical/           # Entry points, shared models, tests, docs and results
├── simscape/            # Physical model, tests, docs, results, cache and codegen
├── docs/                # Project overview translations
└── resources/project/   # Native MATLAB Project metadata
```

Numerical outputs default to `numerical/results/`. Physical outputs default to `simscape/results/fic1_simscape/`; Simulink cache and generated code use `simscape/cache/` and `simscape/codegen/`. Custom `options.outputDir` values remain supported.

- [Numerical quick start and results](numerical/README.md)
- [Common numerical API and robot model](numerical/docs/NUMERICAL_API.md)
- [Simscape quick start and tests](simscape/README.md)
- [Physical Simscape model and verification](simscape/docs/FIC1_SIMSCAPE.md)

Numerical simulations require only base MATLAB. The physical model additionally requires Simulink, Simscape and Simscape Multibody. The simulations and tests have been verified with MATLAB R2026a.
