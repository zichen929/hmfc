# HMFC Simscape 仿真

[English](../../README.md) | [简体中文](README.md)

本目录集中保存原生 Simscape Multibody FIC1 模型、控制器、构建器、批量对照入口、测试、文档、示例图片、结果和生成文件。物理机械臂求解刚性接触关节动力学，批量入口重新积分数值 FIC1 作为对照。

可打开仓库根目录的 [hmfc.prj](../../../hmfc.prj)，或进入 `simscape/` 后执行：

```matlab
setup_simscape;
open_system('hmfc_fic1_simscape.slx');
physical = fic1_simscape_simulation;
run_simscape_tests;
```

`setup_simscape` 可重复调用，不加载模型、不改变工作目录，并配置所需的数值源码路径。初始化不要求安装 Simscape；物理运行需要 Simulink、Simscape、Simscape Multibody。共用参数和 ODE 对照仍位于 `numerical/`。

```text
simscape/
├── hmfc_fic1_simscape.slx          # 可直接运行的物理模型
├── build_hmfc_simscape.m          # 从可读源码重建模型
├── fic1_simscape_controller.m     # 内嵌控制器源文件
├── fic1_simscape_simulation.m     # 四工况运行和 ODE 对照
├── plot_fic1_simscape_results.m   # 已保存结果绘图
├── hmfc_simscape_root.m          # 不依赖工作目录的路径定位
├── setup_simscape.m
├── run_simscape_tests.m
├── tests/                        # 初始化与物理积分测试
├── docs/                         # 中英文指南和图片
├── results/                      # 仿真输出；Git 忽略
├── cache/                        # Simulink 缓存；Git 忽略
└── codegen/                      # 生成代码；Git 忽略
```

批量结果默认写入 `simscape/results/fic1_simscape/`，包含 `ode_reference/` 数值对照。仍可用 `options.outputDir` 自定义输出。初始化脚本与 MATLAB Project 都将缓存和生成代码分别设为 `simscape/cache/`、`simscape/codegen/`。测试、文档和生成目录不加入运行路径，生成目录也不作为项目管理文件。

原有结果和图片已按原内容迁移，历史 MAT 文件保留原始来源元数据。旧根目录缓存归档到 `simscape/cache/legacy_root/`，新构建使用配置后的缓存位置。

Project 的 Simscape 分组提供物理模型和批量对照快捷入口，Tests 分组提供物理测试入口。物理积分测试使用临时输出，只关闭测试自身加载的模型；运行前请关闭已打开的同名物理模型。未安装 Simscape Multibody 时会跳过该项测试。

模型构建、方程、求解器、日志和结果图见[物理模型与验证详解](FIC1_SIMSCAPE.md)。
