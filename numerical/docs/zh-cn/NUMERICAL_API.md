# 数值仿真统一接口

[English](../NUMERICAL_API.md) | [简体中文](NUMERICAL_API.md)

FIC、NAC、IAC 均接收一个标量 `options` 结构体，参数分别组织为 `robot`、`plant`、`controller`、`task`。三者共用机器人模型，控制律、接触求解和积分仍分别实现；接触动力学均求解原始关节动力学与刚性加速度约束。公共辅助函数负责参数分组、选项验证、指标、区域状态、汇总和 CSV 列。

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

## 导数与期望值的命名

符号命名中，每个前缀 `d` 表示一次时间求导，后缀 `d` 表示期望值。`position`、`velocity`、`referenceVelocity` 等完整英文描述字段继续按其字面含义使用。

| 实际信号 | 期望信号 | 含义 |
| --- | --- | --- |
| `q`、`dq`、`ddq` | `qd`、`dqd`、`ddqd` | 关节位置、速度、加速度 |
| `p`、`dp`、`ddp` | `pd`、`dpd`、`ddpd` | 笛卡尔位置、速度、加速度 |
| `xt`、`dxt`、`ddxt` | `xtd`、`dxtd`、`ddxtd` | 内禀位置、速度、加速度 |
| `f` / `force`、`df` | `fd`、`dfd` | 接触力及其导数；`fd` 表示期望力 |

模型导数统一为 `dJ`、`dM`、`dK`、`dLambda`，`dJdq` 明确表示乘积 `dJ*dq`。FIC 输出 `dV`，正导数指标为 `maxPositive_dV`；IAC 使用 `dhatTheta`、`dmomentum`、`dz`、`dzTheory`。`dstate` 表示 ODE 状态导数，`timeStep` 表示采样间隔。输入参考字段使用 `pInputd`、`dpInputd`、`ddpInputd`，将期望值后缀放在最后。

数值结果的关节加速度字段为 `runs(k).ddq`，物理结果为 `cases(k).ddq`。Simscape 的端口和工作区日志改为 `ddq`、`ddq_log`，CSV 列改为 `ddq1_radps2`、`ddq2_radps2`、`pxd_m`、`pyd_m`。

已有 MAT/CSV 归档保留原格式，重新运行会生成新格式。读取旧数据的脚本需迁移：`qdd→ddq`、`Jdot→dJ`、`Mdot→dM`、`LambdaDot→dLambda`、`Jdotdq→dJdq`、`Vdot→dV`、`maxPositiveVdot→maxPositive_dV`、`zDot→dz`、`zDotTheory→dzTheory`、`lambdaDotFiniteDifference→dLambdaFiniteDifference`，以及 `pdInput/dpdInput/ddpdInput→pInputd/dpInputd/ddpInputd`。`pd/dpd/ddpd`、`xtd/dxtd/ddxtd` 和 `fd` 已符合该规则。

## 公共机器人模型

| 函数 | 职责 |
| --- | --- |
| `hmfc_forward_kinematics(q,robot)` | 末端笛卡尔位置 |
| `hmfc_inverse_kinematics(position,robot)` | 正肘角逆解与物理工作空间检查 |
| `hmfc_jacobian(q,robot)` | 末端雅可比矩阵 |
| `hmfc_robot_model(q,dq,robot)` | `M`、`dM`、`C`、`G`、`J`、`dJ`、`dJdq`、`Lambda`、`dLambda`、`eta`、`q`、`dq` |
| `hmfc_robot_state(position,velocity,robot)` | 从笛卡尔测量重建关节状态和机器人模型 |

这些文件位于 `numerical/shared/`，不包含接触法向、控制增益或认证区域限制。每个数值仿真仍保留自己的接触求解和区域策略，包括 NAC 越出原认证区域后继续积分的选项。初始关节状态也使用公共逆运动学和雅可比函数。解析解、平衡点和有限差分检查继续作为公共模型的独立验证依据。

## 单步计算流程

各公共入口验证选项后，均由局部函数 `run_controller(options)` 执行单控制器仿真。三者绘图入口统一为 `make_plots(results)`，FIC 内部继续使用 `plot_case_pair` 和 `plot_diagnostics` 细分绘图任务。NAC、IAC 的文本校验报告均由 `write_report(results)` 输出；IAC 原局部函数名为 `write_summary`。

三套仿真均通过局部函数 `evaluate` 完成 ODE 积分和采样结果后处理中的单步计算。IAC 的终止事件也使用该函数，并传入当前工况的参考。共同流程如下：

1. `measured_state(x,tangentialVelocity,robot,plant)` 返回笛卡尔位置、速度和重建的机器人模型。三者均直接采用 `p0+T*x` 和 `T*tangentialVelocity`，各自保留原有区域检查策略。
2. `reference_command(t,task,mode,controller)` 生成控制器原生参考输入。FIC 1/4 接收笛卡尔向量，FIC 2/3 接收内禀标量，坐标转换仍由 FIC 控制器完成。NAC、IAC 生成导数为零的笛卡尔调节参考。
3. `controller_command` 返回参考、误差、`up`、`forceCommand`、`F` 和 `tau`，并返回接触求解所需的 `F0` 和向量 `forceFeedback`，对应仿射命令 `F=F0+forceFeedback*force`。
4. `contact_plant(cmd,model,n)` 使用 `F0`、`forceFeedback` 和 `model.dJdq` 联立求解原始关节动力学与刚性约束。FIC/NAC 先构造不含反馈力的命令，求解瞬时反馈环，再将求出的标量接触力传回控制器。IAC 的 `forceFeedback` 为零，并另外计算估计器更新。

## 选项与参数分组

| 公共选项 | 默认值 | 含义 |
| --- | --- | --- |
| `makePlots` | `true` | 是否创建图窗，与保存开关独立 |
| `saveResults` | `true` | 是否保存 MAT、CSV、报告及启用的图；为 `false` 时不创建输出目录或文件 |
| `visible` | `'on'` | 图窗可见性，可用 `'on'`、`'off'` |
| `outputDir` | `numerical/results/` 下对应控制器目录 | 保存位置 |
| `duration` | `6` | 仿真秒数；至少为 6，以保留末端收敛校验 |
| `initialVelocity` | `0` | 主工况初始切向速度，m/s |
| `controllerIds` | `1:4` | 仅 FIC：按给定顺序选择不重复编号 |

NAC 保留固定估计角和区域约束选项；IAC 保留初始估计角和扇区选项。IAC 的流形不变性工况仍使用 0.02 m/s，其原地调节工况仍使用零初速度和零目标位移。

所有独立仿真返回 `name`、`source`、`options`、`robot`、`plant`、`controller`、`task`、`odeOptions`、`runs`、`summary`、`checks`、`allChecksPassed`。`robot` 存放连杆物理参数，`plant` 存放真实接触几何与运行区域，`controller` 存放增益和控制器专属参数，`task` 存放参考原点、轴和目标。只有 FIC 控制器接收真实几何。

多 FIC 运行时，`runs` 汇集所有所选控制器的工况，并额外返回对应 `ficN` 结果及两两等价性检查。单 FIC 编号位于 `controller.id`。汇集数组中，不适用的专属诊断字段为空，各 `ficN` 内保留完整原生字段。

## 工况公共字段

采样数组每行对应一个输出时刻，使用 SI 单位，角度为弧度。

| 字段 | 形状及含义 |
| --- | --- |
| `controller`、`name`、`mode`、`kf` | 控制器名称、工况名、参考模式、力反馈增益 |
| `t` | N×1 输出时刻 |
| `state` | FIC/NAC 为 N×2 `[x_t, dx_t]`；IAC 为 N×3 `[x_t, dx_t, hat_theta]` |
| `initialState` | 首行状态的转置列向量 |
| `x` | N×1 切向位移，等于 `state(:,1)` |
| `position`、`velocity`、`acceleration` | N×2 笛卡尔位置、速度、加速度 |
| `reference`、`referenceVelocity`、`referenceAcceleration` | N×2 笛卡尔参考 |
| `positionError`、`velocityError` | N×2 参考减实际信号 |
| `error` | N×1 有符号切向位置误差 |
| `force`、`forceCommand` | N×1 真实接触反力和标量力命令 |
| `angle` | N×1：FIC 真实法向角、NAC 固定角、IAC 有效估计角 |
| `tau`、`q`、`dq`、`ddq` | N×2 关节力矩、位置、速度、加速度 |
| `jointResidual`、`cartesianResidual` | N×2 带符号的关节动力学残差（N m）和笛卡尔动力学残差（N） |
| `operatingRegionSatisfied`、`firstRegionExitTime` | 原认证区域状态；未越界时退出时间为 `NaN` |
| `metrics`、`checks` | 公共及专属指标、包含 `passed` 的校验结果 |

公共校验字段包括 `maxJointResidual`、`maxCartesianResidual`、`maxGap`、`maxNormalVelocity`、`maxNormalAcceleration`、`operatingRegionSatisfied` 和 `passed`。

两种动力学残差分别为 `M*ddq+C*dq+G-tau-J'*n*force` 和 `Lambda*acceleration+eta-F-n*force`。共同的最大残差检查均先取每行无穷范数，再取时间上的最大值，即 `max(vecnorm(run.jointResidual,inf,2))`，`cartesianResidual` 同理。

FIC、IAC 的 `Lambda` 矩阵逐点最小、最大特征值均命名为 `minLambda`、`maxLambda`。最大绝对接触间隙统一命名为 `maxGap`，包括 FIC 的附加指标。NAC 保存单位为 N 的力差 `forceResidual`，检查 `maxForceResidual < 1e-9` N；IAC 保存无量纲的 `forceRatioResidual = rho-rhoFormula`，检查 `maxForceRatioResidual < 1e-10`。两者使用各自单位对应的独立阈值，均区别于相对于期望力 `fd` 的力跟踪误差。

公共指标包括 `finalError`、`finalForce`、`minForce`、`maxForceError`、`tailRmsError`、`finalTangentialVelocity`、`finalAngleError`、`peakTorque`、`operatingRegionSatisfied`、`firstRegionExitTime`。退出时间单位为秒，在 `run`、`metrics` 和独立仿真的 `summary` 表中使用相同字段名。`summary` 在每个工况指标之外增加 `controller`、`name`、`mode`、`kf` 列。

FIC 的解析误差轨迹保存为 `analyticPositionError`、`analyticVelocityError`，对应最大偏差指标为 `maxAnalyticPositionError`、`maxAnalyticVelocityError`。误差动力学恒等式的逐点残差为 `errorDynamicsResidual`，最大残差检查为 `maxErrorDynamicsResidual`。

FIC 保留解析解、增益不变性、内禀坐标和两两等价性检查；NAC 保留非线性平衡点预测及零角差解析解检查；IAC 保留 `hatTheta`、`beta`、`barTheta`、`angleError`、角误差导数、包络和认证参数，以及动量、流形不变性和扇区校验。`angleError` 是诊断信号，ODE 实际积分 `hatTheta`。

三控制器比较入口用 `runs` 存放三个主工况，用 `fic1`、`nac1`、`iac1` 保留完整独立结果。比较汇总表保留原有带 SI 单位后缀的指标列，标准与最大角入口均支持 `saveResults=false`。

## CSV 与迁移

所有独立仿真的 CSV 前九列统一为：

```text
time_s,x_t_m,dx_t_mps,motion_error_m,force_N,force_command_N,normal_angle_rad,tau1_Nm,tau2_Nm
```

IAC 另追加 `hat_theta_rad`、`beta_rad`、`bar_theta_rad`、`angle_error_rad`。

读取旧 MAT 结果时，FIC 的 `dynamicResidual` 改为 `jointResidual`，NAC 的 `plantResidual` 改为 `jointResidual`，IAC 的 `plantResidual` 改为 `cartesianResidual`。两种公共残差现在每个采样点都保存两个带符号分量；旧 NAC/IAC 残差为标量无穷范数，可用 `vecnorm(run.jointResidual,inf,2)` 或 `vecnorm(run.cartesianResidual,inf,2)` 从新结果获得。FIC 指标 `maxJointDynamicsResidual` 改为 `maxJointResidual`，关节和笛卡尔最大残差指标均采用与 NAC、IAC 相同的无穷范数。已有归档保留原字段。

IAC 的 `minEig/maxEig` 改为 `minLambda/maxLambda`。FIC 的 `metrics.maxContactPositionError` 及复制到校验中的同名字段统一改为 `maxGap`。原校验字段 `maxForceIdentityResidual` 在 NAC 中改为 `maxForceResidual`（N），在 IAC 中改为 `maxForceRatioResidual`（无量纲），分别保留 `1e-9` N 和 `1e-10` 的阈值。IAC 同时新增保存该校验使用的逐点 `forceRatioResidual`。

FIC 的 `exactEp/exactEv` 改为 `analyticPositionError/analyticVelocityError`，`maxAnalyticError` 改为 `maxAnalyticPositionError`，`errorResidual` 改为 `errorDynamicsResidual`。公共指标及独立仿真汇总表列名 `firstRegionExitTime_s` 改为 `firstRegionExitTime`，与工况元数据一致，数值单位仍为秒。复制到 FIC 校验中的字段同步更名。已有 MAT/CSV 归档保留原字段。NAC/IAC 的 `rho` 诊断字段继续使用原有定义。

旧 FIC 位置参数改为 `fic_simulation(struct('controllerIds',ids,...))`；`cases` 改为 `runs`，`config` 改为分组参数，NAC 的 `control` 改为 `controller`。三者均用 `state` 表示积分状态，IAC 原 `z` 诊断字段改名为 `angleError`。FIC 原 `p/dp/pdd/pd/dpd/ep/ev/f/uf` 分别改为 `position/velocity/acceleration/reference/referenceVelocity/positionError/velocityError/force/forceCommand`；NAC 使用相同的参考和力命令字段名。比较入口原 `primary` 改为 `runs`。
