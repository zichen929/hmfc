# HMFC：混合运动/力控制

[English](../../README.md) | [简体中文](README.md)

本仓库在刚性接触的平面两连杆机器人上比较 FIC、NAC1 和 IAC1。数值仿真的代码、公共模型、测试、文档、示例图片和结果统一放在 [`numerical/`](../../numerical/docs/zh-cn/README.md)，物理模型、测试、文档和生成文件统一放在 [`simscape/`](../../simscape/docs/zh-cn/README.md)。

在 MATLAB 中打开根目录的 [hmfc.prj](../../hmfc.prj)，或在仓库根目录运行：

```matlab
setup_project;
comparison = fic1_nac1_iac1_simulation;
run_numerical_tests;
```

仅做数值仿真时，进入 `numerical/` 后执行 `setup_numerical` 即可。进入 `simscape/` 后执行 `setup_simscape` 可初始化物理仿真；所有初始化函数均不改变工作目录。MATLAB Project 仅将根目录、`numerical/`、`numerical/shared/`、`simscape/` 设为源码路径，正确分类测试文件，并提供数值对比、物理模型、物理对照和两类测试快捷入口；生成结果不作为项目管理文件，且由 Git 忽略。

```text
hmfc/
├── hmfc.prj, setup_project.m
├── numerical/           # 数值入口、公共模型、测试、文档和结果
├── simscape/            # 物理模型、测试、文档、结果、缓存和生成代码
├── docs/                # 项目概览的翻译
└── resources/project/   # MATLAB Project 原生元数据
```

数值结果默认写入 `numerical/results/`；物理结果默认写入 `simscape/results/fic1_simscape/`，Simulink 缓存和生成代码分别在 `simscape/cache/`、`simscape/codegen/`。仍可通过 `options.outputDir` 指定自定义位置。

- [数值仿真使用说明和结果](../../numerical/docs/zh-cn/README.md)
- [统一数值接口和公共机器人模型](../../numerical/docs/zh-cn/NUMERICAL_API.md)
- [Simscape 使用说明与测试](../../simscape/docs/zh-cn/README.md)
- [物理模型与验证](../../simscape/docs/zh-cn/FIC1_SIMSCAPE.md)

数值仿真只需基础 MATLAB；物理模型额外需要 Simulink、Simscape、Simscape Multibody。仿真及测试已在 MATLAB R2026a 验证。
