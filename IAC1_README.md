# IAC 1 独立 MATLAB ODE 仿真

入口为 `iac1_simulation.m`，只依赖 MATLAB 基础功能。模型、控制器、估计器、接触反力求解和绘图均为文件内的局部函数，没有调用或修改任何 FIC 代码。

源文档的实际文件名为 `obsidian-vault/04-writing/tcst26/TCST26E3.md`，一级标题是 `TCST26`；本实现对应其中的 **IAC 1: Task-Space Control, Uncertain Surface Angle** 及两连杆机器人附录。

## 运行

在本目录执行：

```matlab
results = iac1_simulation;
```

无界面运行并保存图像：

```matlab
results = iac1_simulation(struct('visible','off'));
```

只计算和校验：

```matlab
results = iac1_simulation(struct('makePlots',false,'saveResults',false));
```

可以覆盖 `initialAngleDeg`、`initialVelocity`、`duration`、`outputDir`、`makePlots`、`saveResults` 和 `visible`。初始角度指定的是有效估计 `bar_theta(0)`，以度为单位，必须位于 ±6° 扇区；控制方程内部全部使用弧度。`duration` 至少为 6 秒，以便执行末端收敛检查。增大初始速度等操作仍须满足机器人运行区域，越界会停止并报错。

## 工况与模型

- 真实接触面为水平直线 `y = 0.4 m`，环境位于其下方，真实法向 `n = [0;1]`，真实角度为 0°。
- 初始接触位置 `p0 = [0.6;0.4] m`；初始法向速度为零，接触已建立。
- 主工况初始有效估计为 +5°，即 `bar_n(0) = [-sin(5°);cos(5°)]`；初始速度为零。
- 定点调节目标为 `p_d = [0.68;0.4] m`，期望法向力为 10 N。
- 已知模型为竖直平面两转动关节机器人：杆长 0.6、0.5 m，质量 2、1.5 kg，质心位于杆中点，转动惯量取均匀细杆值；显式保留重力、科氏项和构型相关的笛卡尔惯量。
- `k_p = 25`、`k_v = 10`、`gamma = 0.4`，名义角度中心为 0°，先验扇区为 ±6°。

同时运行三项交叉验证：−5° 初始偏差；有效初始角度为零但初始切向速度为 0.02 m/s 的流形不变性；目标即初始位置、初始速度为零但有 +5° 偏差的调节工况。最后一项没有外加运动激励，估计误差仍会引起短暂切向运动。

参考点落在真实接触线上是文档的题设假定。本仿真事先给定这样的参考点，控制器没有使用真实法向投影参考或误差信号。

## 方程实现

ODE 状态仅为 `[x_t; dot_x_t; hat_theta]`。真实刚性接触通过最小坐标 `p = p0 + T*x_t` 实现；利用固定肘形的逆运动学恢复 `q`，利用 `J\dot_p` 恢复 `dot_q`。没有罚弹簧、接触阻尼或法向约束稳定化。

控制器实现：

```text
m_p       = Lambda * dot_p
beta      = -gamma * T_c' * m_p
bar_theta = hat_theta + beta
bar_n     = R(bar_theta) * n0
u_p       = k_p * (p_d - p) - k_v * dot_p
F         = eta + Lambda * u_p - bar_n * f_d
tau       = J' * F
g_p       = F - eta + dot_Lambda * dot_p
dot_hat   = gamma * T_c' * (g_p + bar_n * f)
```

`dot_Lambda` 由已知模型解析求导计算，代码还用独立中心差分检查它。初始内部估计按 `hat_theta(0) = bar_theta(0) + gamma*T_c'*Lambda(0)*dot_p(0)` 设置，包含非零初始动量的情形。

物理模型通过以下线性约束方程求实际加速度及标量接触反力，然后将反力作为力传感器输入送给估计器：

```text
[Lambda  -n] [ddot_p] = [F - eta]
[n'       0] [f     ]   [0      ]
```

控制器和估计器的参数接口没有真实 `n`、真实角度、参数误差 `z` 或实测加速度；真实几何仅用于物理模型、离线认证和验证。没有将文档推导出的 `dot_z` 作为 ODE，也没有直接用理论反力公式生成传感器读数。

## 验证与输出

在 `q2` 属于 [70°,115°] 的区域，由两连杆惯量的迹和行列式给出解析的统一惯量上下界，据此计算文档中的 `chi_Lambda`、`rho_min`、接触力下界及角度指数包络。这些界不依赖轨迹采样拟合。切向坐标另限制在 [−0.04,0.16] m；代码检查轨迹，并用 ODE 事件监视运行区域、估计扇区及正接触反力。

每个工况自动检查：

- 笛卡尔和关节动力学、动量方程的残差。
- 由实际控制器得到的 `dot_z` 与理论误差方程的一致性，以及积分后有效角度的中心差分校验。
- 反力关系 `f/f_d = cos(z) + r_Lambda*sin(z)`、切向误差动力学。
- 接触力严格为正且不小于认证下界；角度不离开扇区且满足指数包络。
- 法向间隙、法向速度和机器人惯量范围。
- 末端角度误差小于 `1e-6 rad`，切向误差小于 `1e-6 m`，速度小于 `1e-6 m/s`，力误差小于 `1e-5 N`。
- 零有效初始误差工况的流形不变性。

默认结果保存在 `results/iac1/`：完整 `iac1_results.mat`、四个工况的 CSV、包含误差与残差的 `verification.txt`，以及 `iac1_summary.png`、可编辑的 `iac1_summary.fig`。函数返回的 `results.runs` 包含全部信号与检查结果。

这些是文档理想条件下的数值验证：精确模型、精确标量力测量、理想执行器和已建立的无摩擦刚性接触。仿真不包含建立接触、碰撞、脱离后的运动模式或测量噪声；负反力不会被截成零来掩盖接触失效。
