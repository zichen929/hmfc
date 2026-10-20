# FIC 1 / FIC 2 / FIC 3 / FIC 4：MATLAB ODE 数值验证

依据 `../obsidian-vault/04-writing/tcst26/TCST26E3.md` 中的 **FIC 1: Task-Space Tracking**、**FIC 2: Task-Space Tracking with Intrinsic Reference**、**FIC 3: Intrinsic Coordinate Tracking** 和 **FIC 4: Intrinsic Coordinate Tracking with Task Space Reference**，使用附录的竖直平面 2R 机械臂，验证定点调节、时变轨迹跟踪及四种控制器的等价性。仅需基础 MATLAB，无需 Simulink、Robotics Toolbox 或 Optimization Toolbox。

四个 FIC 的主调节工况与独立的 `iac1_simulation.m` 对齐：相同的机器人、水平接触面、初始位置和速度、参考位置、运动增益、期望接触力、积分时长及输出采样时刻。FIC 已知真实法向，IAC 1 的主工况从 +5° 有效估计偏差开始；FIC 的 `regulation, kf=0` 是对应的全信息基准。参数在各实现内独立定义，两个仿真不互相调用或加载设置。IAC 1 的说明见 [IAC1_README.md](IAC1_README.md)。

独立入口 `nac1_simulation` 提供 **NAC 1**：相同工况下，将 FIC 1 的真实法向替换成始终固定的 +5° 初始估计法向，用于观察无自适应时的位置和力偏差。运行方法和验证方程见 [NAC1_README.md](NAC1_README.md)。

## 运行

按要运行的控制器选择入口：

| 目的 | MATLAB 命令 | 默认结果目录 |
| --- | --- | --- |
| 单独 FIC 1 | `fic1Results = fic1_simulation;` | `results/fic1/` |
| 单独 NAC 1 | `nacResults = nac1_simulation;` | `results/nac1/` |
| 单独 IAC 1 | `iacResults = iac1_simulation;` | `results/iac1/` |
| FIC 1、NAC 1、IAC 1 联合运行与对比 | `comparison = fic1_nac1_iac1_simulation;` | `results/fic1_nac1_iac1/` |

三控制器对比入口会重新执行三个独立仿真，核对工况，并单独保存对比图与汇总数据，详见 [FIC1_NAC1_IAC1_README.md](FIC1_NAC1_IAC1_README.md)。

在 MATLAB 中切换至本目录，执行：

```matlab
results = fic1234_simulation;
```

也可关闭绘图，或指定输出目录：

```matlab
results = fic1234_simulation(false);
results = fic1234_simulation(true, fullfile(pwd, 'my_results'));
```

也可分别运行，原有入口保留：

```matlab
fic1Results = fic1_simulation;  % 单独运行，默认保存至 results/fic1/
fic2Results = fic2_simulation;  % 单独运行，默认保存至 results/fic2/
fic3Results = fic3_simulation;  % 单独运行，默认保存至 results/fic3/
fic4Results = fic4_simulation;  % 单独运行，默认保存至 results/fic4/
fic12Results = fic12_simulation; % 仅比较 FIC 1、2，默认保存至 results/fic12/
fic123Results = fic123_simulation; % 仅比较 FIC 1、2、3，保存至 results/fic123/
```

四控制器联合运行默认输出到 `results/fic1234/`，同时显示各控制器的结果图和并列对比图：

- `fic1/`、`fic2/`、`fic3/`、`fic4/`：分别保存各控制器的全部结果。
- `comparison/regulation_kf_0.png/.fig`、`regulation_kf_2.png/.fig`：定点调节的四列并列对比，从左到右为 FIC 1、2、3、4。
- `comparison/tracking_kf_0.png/.fig`、`tracking_kf_2.png/.fig`：轨迹跟踪的四列并列对比。
- `comparison/equivalence_i_j.png/.fig`（配对为 1–2、1–3、1–4、2–3、2–4、3–4）：每对控制器在四组匹配工况下的位置、加速度命令、接触力和关节力矩差异。
- `comparison/summary.csv`：六对控制器、每对四组工况的最大差异，共 24 行，含 `controllerA`、`controllerB` 列。
- `summary.csv`：四种控制器共 16 组工况的指标，含 `controller` 列。
- `fic1234_results.mat`：联合结果；`results.fic1`、`results.fic2`、`results.fic3`、`results.fic4` 为各控制器完整数据，`results.comparisonSummary` 为差异表。

每个控制器的结果目录包含：

- `regulation.png/.fig`：定点调节的轨迹、运动误差、接触力、Lyapunov 函数及关节力矩。
- `tracking.png/.fig`：时变参考下的对应结果。
- `diagnostics.png/.fig`：力误差、约束残差、运动误差方程残差、关节动力学残差、解析解偏差及非奇异性裕量。
- `summary.csv`：四组工况的数值指标。
- `fic1_results.mat`、`fic2_results.mat`、`fic3_results.mat` 或 `fic4_results.mat`：参数、积分器设置、完整采样信号、解析解、指标和检查结果。

命令行运行（无图形窗口）：

```bash
matlab -batch "set(groot,'defaultFigureVisible','off'); results = fic1234_simulation;"
```

## 控制律与刚性接触求解

FIC 1 接收 $p_d,\dot p_d,\ddot p_d$ 三个笛卡尔参考向量。FIC 2 接收 $x_{t,d},\dot x_{t,d},\ddot x_{t,d}$ 三个内禀参考标量，先根据真实几何转换：

$$
p_d=p_0+Tx_{t,d},\qquad \dot p_d=T\dot x_{t,d},\qquad \ddot p_d=T\ddot x_{t,d}.
$$

两者随后采用原文的笛卡尔反馈律：

$$
u_p=\ddot p_d+k_v(\dot p_d-\dot p)+k_p(p_d-p),\qquad
u_f=f_d+k_f(f_d-f),
$$
$$
F=\eta+\Lambda u_p-nu_f,\qquad \tau=J^TF.
$$

FIC 3 同样接收内禀参考，但直接从测得的笛卡尔位置、速度计算标量误差：

$$
x_t=T^T(p-p_0),\quad \dot x_t=T^T\dot p,\quad
e_t=x_{t,d}-x_t,\quad \dot e_t=\dot x_{t,d}-\dot x_t,
$$
$$
u_t=\ddot x_{t,d}+k_v\dot e_t+k_pe_t,\qquad u_p=Tu_t.
$$

FIC 3 的 `intrinsicControl` 函数不接收笛卡尔参考或笛卡尔误差。其笛卡尔参考、误差只用于统一的结果展示和验证。定点调节时参考的一、二阶导数均为零，以上公式自动退化为原文的调节器。

FIC 3 的结果额外保存 `xt`、`dxt`、`xtd`、`dxtd`、`ddxtd`、`et`、`det`、`ut`，以及标量误差方程和笛卡尔误差映射的残差。

FIC 4 接收笛卡尔参考，并先转换为内禀参考：

$$
x_{t,d}=T^T(p_d-p_0),\qquad
\dot x_{t,d}=T^T\dot p_d,\qquad
\ddot x_{t,d}=T^T\ddot p_d.
$$

随后调用与 FIC 3 相同的 `intrinsicControl`，形成 $u_t$ 和 $u_p=Tu_t$。FIC 4 保存上述内禀信号，另保存 `pdInput`、`dpdInput`、`ddpdInput`、`referenceNormalOffset` 和 `referenceProjectionResidual`。其 `.pd`、`.dpd` 为投影到接触线后的有效参考；默认输入位于接触线上，两种参考在数值精度内一致。对于带法向偏移的输入，原文的目标是投影 $p_{d,\mathcal S}=p_0+TT^T(p_d-p_0)$。

`fic_simulation.m` 共用机械臂、刚性接触求解、数值检查及独立结果绘图。FIC 1、4 调用笛卡尔参考生成器，FIC 2、3 调用内禀参考生成器；四者分别执行 ODE 积分。`fic_compare_simulation.m` 按相同模式、增益和采样时间进行所有两两比较，不复用另一控制器的仿真轨迹。

`ode45` 积分状态为 $z=[s,v]^T$，其中 $p=p_0+Ts$、$\dot p=Tv$。使用固定正肘角分支的逆运动学恢复 $q$，再由 $\dot q=J^{-1}Tv$ 得到关节速度。该坐标选择从几何上精确满足接触约束，避免对刚性接触引入弹簧、阻尼或约束稳定化参数。初始 $s=0$，因此 $p_0=p(0)$，与笔记定义一致。

瞬时力反馈使控制器与接触反力形成代数环。令

$$
F_0=\eta+\Lambda u_p-n(1+k_f)f_d,\qquad \tau_0=J^TF_0,\qquad J_n=n^TJ.
$$

每次 ODE 右端求值时，联立原始关节动力学及法向加速度约束：

$$
\begin{bmatrix}
M&-(1+k_f)J_n^T\\
J_n&0
\end{bmatrix}
\begin{bmatrix}\ddot q\\f\end{bmatrix}
=
\begin{bmatrix}
\tau_0-C\dot q-G\\
-n^T\dot J\dot q
\end{bmatrix}.
$$

随后由 $\ddot p=J\ddot q+\dot J\dot q$ 计算 $\dot v=T^T\ddot p$。接触力 $f$ 来自这个联立方程，程序没有把 $f=f_d$ 或 $\ddot p=u_p$ 直接赋给被控对象，也没有使用上一时刻的力替代瞬时力反馈。

## 参数与工况

参数集中在 `fic_simulation.m` 的 `configuration` 函数中。笔记没有给出数值参数，以下为用于验证的示例选择，动力学公式采用其附录。

| 项目 | 默认值 |
| --- | --- |
| 连杆长度 $l_1,l_2$ | 0.6、0.5 m |
| 连杆质量 $m_1,m_2$ | 2、1.5 kg |
| 质心与转动惯量 | $l_{c,i}=l_i/2$，$I_i=m_i l_i^2/12$ |
| 重力加速度 | 9.81 m/s²，保留并补偿配置相关的重力项 |
| 接触线方向 | 水平直线 $y=0.4$ m，$n=[0;1]$，$T=[1;0]$ |
| 初始接触位置与关节位置 | $p_0=[0.6;0.4]$ m；$q(0)$ 由同一正肘角分支的逆运动学求出 |
| 初始切向速度 | 0 m/s，初始法向速度为零 |
| 增益 | $k_p=25$、$k_v=10$；主基准 $k_f=0$，补充测试 $k_f=2$ |
| 期望接触力 | $f_d=10$ N |
| 定点参考 | $p_d=p_0+0.08T=[0.68;0.4]$ m |
| 跟踪参考 | $p_d(t)=p_0+T[0.08+0.045\sin(0.8t)+0.02\cos(1.3t)]$ |
| 运行区域 | $x_t\in(-0.04,0.16)$ m，$q_2\in(70°,115°)$ |
| 积分时长、输出间隔 | 6 s、0.002 s，与 IAC 1 的默认采样时刻一致 |
| `ode45` 设置 | `RelTol=1e-9`、`AbsTol=1e-11`、`MaxStep=0.01`；IAC 1 的内部估计状态另使用 `AbsTol=1e-12` |

每种控制器分别对两种参考和两种力反馈增益运行，共四组仿真；四控制器联合运行共 16 组。FIC 1、4 的笛卡尔参考生成器为 `cartesianReference`；FIC 2、3 的内禀参考生成器为 `intrinsicReference`，默认参考表示同一运动任务。轨迹跟踪和 $k_f=2$ 保留为 FIC 补充测试；IAC 1 的负偏角、流形不变性和原地调节则是其估计器专项测试，这些补充项不表示同一参考或同一初始速度。

## 验证判据及解释

笔记预测

$$
\ddot e_p+k_v\dot e_p+k_pe_p=0,\qquad (1+k_f)(f_d-f)=0.
$$

运动误差极点为二重极点 $-5$。程序使用矩阵指数 `expm` 独立计算上述误差方程的解析解，与从完整机械臂动力学得到的 ODE 轨迹比较。

程序自动断言：

- 终端位置、速度误差及其与解析解的最大偏差分别小于 $10^{-7}$（按对应 SI 单位）。
- 力误差、法向位置/速度/加速度残差、关节动力学及运动误差方程残差分别小于 $10^{-8}$（按对应 SI 单位）。
- 使用笛卡尔约束公式独立重构的接触力与联立方程结果一致。
- 对 $V=(\|\dot e_p\|^2+k_p\|e_p\|^2)/2$，满足 $\dot V=-k_v\|\dot e_p\|^2$ 至数值精度。
- 轨迹保持在上述切向坐标与肘角运行区域，笛卡尔惯量正定；求解时还逐次检查反力为正，验证维持单边接触。
- 同一参考下两种 $k_f$ 的状态轨迹差小于 $10^{-7}$。
- FIC 3、4 还检查 $\ddot e_t+k_v\dot e_t+k_pe_t=0$、$e_p=Te_t$ 和 $\dot e_p=T\dot e_t$，残差小于 $10^{-8}$。
- 联合运行时，所有控制器配对的匹配工况中，笛卡尔参考、状态、位置、速度、加速度、运动命令、力命令、接触力及关节力矩的最大差异分别小于 $10^{-7}$（按对应 SI 单位）。

在笔记的精确模型、无摩擦、已建立的刚性单边接触和无延迟标量力测量假设下，力通道是代数通道，预期从初始时刻即有 $f=f_d>0$，不存在力误差的动态衰减过程。因此两种 $k_f$ 的理想结果应重合。图中的机器精度级力误差不代表力振荡。

真实几何下，FIC 2、4 的参考转换精确，FIC 3、4 的 $Te_t=e_p$、$T\dot e_t=\dot e_p$ 也精确，因此同参考、同增益的四种控制器结果应在数值精度内重合。并列图每行统一坐标范围；差异图中零差异按 $10^{-16}$ 的显示下限绘制，实际数值见 `comparison/summary.csv`。

接触几何残差接近零是所选降阶坐标保证的；动力学、反力和解析误差解的对照则用于检查控制律与动力学求解的一致性。惯量和雅可比指标是在数值求值/采样点检查，不构成对整个工作空间的证明。数值仿真用于核对理想模型下的理论预测，不替代稳定性证明；模型误差、柔性接触、噪声、时延、力矩饱和与接触建立过程均不在本次模型中。

## 本机实测结果

使用 MATLAB R2026a 运行与 IAC 1 对齐后的默认参数，四种控制器共 16 组工况、24 项两两等价性检查全部通过。下面汇总四种实现的结果；机器精度级差异来自参考转换和标量反馈的运算顺序。

| 指标 | 结果 |
| --- | --- |
| 6 s 定点调节位置误差 | 约 2.32e-13 m |
| 6 s 轨迹跟踪位置误差 | 约 5.85e-13 m |
| 全部工况最大接触力误差 | 约 1.42e-14 N |
| 定点调节与解析解的最大位置偏差 | 约 3.20e-12 m |
| 轨迹跟踪与解析解的最大位置偏差 | 约 4.68e-12 m |
| 所有配对的最大位置差 | 约 4.58e-16 m |
| 所有配对的最大关节力矩差范数 | 约 1.25e-14 N m |

接触力始终为正，约为 10 N；全部轨迹满足与 IAC 1 相同的运行区域。默认联合结果目录的 `scenario_alignment.txt` 记录了独立结果文件之间的工况核对结果，`summary.csv` 和 `comparison/summary.csv` 保存完整数值指标。

![FIC 1、FIC 2、FIC 3 与 FIC 4 在 IAC 1 同一主调节工况下的并列结果](results/fic1234/comparison/regulation_kf_0.png)
