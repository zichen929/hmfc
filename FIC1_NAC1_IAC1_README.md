# FIC 1 / NAC 1 / IAC 1 三控制器对比仿真

独立入口是 `fic1_nac1_iac1_simulation.m`。单独运行 NAC 1 的入口仍为 `nac1_simulation.m`，说明见 [NAC1_README.md](NAC1_README.md)。

## 运行

```matlab
% 只运行 NAC 1：
nacResults = nac1_simulation;

% 重新运行三个控制器，生成三者的对比结果：
comparison = fic1_nac1_iac1_simulation;
```

三控制器入口也支持：

```matlab
comparison = fic1_nac1_iac1_simulation(struct('visible','off'));
comparison = fic1_nac1_iac1_simulation(struct('makePlots',false));
comparison = fic1_nac1_iac1_simulation(struct('outputDir',fullfile(pwd,'my_comparison')));
```

## 运行与比较的内容

该入口依次调用三个独立实现，进行新的 ODE 积分，不读取以往保存的仿真结果。控制器的模型和控制律仍由各自文件维护，单控制器入口不依赖本对比入口。

对比选取三者相同的**主调节工况**：水平接触面、零初速、8 cm 目标位移、`kp=25`、`kv=10`、`fd=10 N`、`kf=0`、6 秒积分和 0.002 秒输出间隔。FIC 1 使用真实法向；NAC 1 始终固定为 +5°；IAC 1 从 +5° 开始自适应。运行后自动检查机器人参数、真实几何、初始状态、参考、增益、运行区域、输出时间及 NAC/IAC 初始估计的一致性。

各独立入口原有的补充验证也会执行，但三控制器对比图和汇总表只选取各自的主工况。不会将三个控制器的轨迹断言为相同；固定法向误差导致的 NAC 1 稳态偏差正是比较内容之一。

## 独立的输出目录

| 入口 | 默认结果目录 |
| --- | --- |
| `nac1_simulation` | `results/nac1/`，仅 NAC 1 |
| `fic1_nac1_iac1_simulation` | `results/fic1_nac1_iac1/`，三控制器联合运行与对比 |

三控制器目录中包含：

- `comparison.png/.fig`：三者的位置、运动误差、接触力和法向角对比。
- `summary.csv`、`verification.txt`：三行主工况指标和一致性检查报告。
- `fic1_primary.csv`、`nac1_primary.csv`、`iac1_primary.csv`：统一字段的主工况采样数据。
- `fic1_nac1_iac1_results.mat`：完整返回值，包含 `results.fic1`、`results.nac1`、`results.iac1`、`results.primary` 和 `results.summary`。
- `fic1/`、`nac1/`、`iac1/`：本次分别执行三个独立仿真的原始输出。

该入口将子仿真输出显式指定到上述子目录，单独 NAC 1、IAC 1、FIC 1 的默认输出目录不会被覆盖。三控制器运行只显示和导出对比图，不额外打开单控制器的补充工况图。
