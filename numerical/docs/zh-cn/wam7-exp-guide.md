# WAM 两自由度实验：FIC1、NAC1、IAC1 复现指南

[English](../wam7-exp-guide.md) | [简体中文](wam7-exp-guide.md)

## 1. 三组控制器的表达式

以下变量均在固定二维任务平面内。$p,v$ 为末端位置和速度，$p_d$ 为接触线上的固定目标；$n$ 指向自由空间，$f\geq0$ 为环境对机器人的标量压缩力。动力学约定为：

$$
\begin{aligned}
M_2\ddot r+h_2 &= \tau_a+J^{\mathsf T}nf, \\
\Lambda\ddot p+\eta &= F+nf, \\
h_2 &= C_2\dot r+G_2.
\end{aligned}
$$

统一角度定义（内部使用弧度）：

$$
\begin{aligned}
n(\theta) &= \begin{bmatrix}-\sin\theta \\ \cos\theta\end{bmatrix}, \\
T(\theta) &= \begin{bmatrix}\cos\theta \\ \sin\theta\end{bmatrix}.
\end{aligned}
$$

### FIC1：使用真实法向

$$
\begin{aligned}
u_p &= \ddot p_d+k_v(\dot p_d-v)+k_p(p_d-p), \\
u_f &= f_d+k_f(f_d-f), \\
F &= \eta+\Lambda u_p-n(\theta_{\mathrm{true}})u_f, \\
\tau_a &= J^{\mathsf T}F.
\end{aligned}
$$

本次固定目标调节取 $\dot p_d=\ddot p_d=0$，主对比取 $k_f=0$。

### NAC1：使用固定错误法向

与 FIC1 相同，只替换法向：

$$
\begin{aligned}
n_{\mathrm{fix}} &= n(\theta_{\mathrm{fix}}), \qquad \dot\theta_{\mathrm{fix}}=0, \\
F &= \eta+\Lambda u_p-n_{\mathrm{fix}}u_f, \\
\tau_a &= J^{\mathsf T}F.
\end{aligned}
$$

普通工况取 $\theta_{\mathrm{fix}}-\theta_{\mathrm{true}}=+5^\circ$，没有估计器状态。

### IAC1：使用自适应的有效法向

对应现有仿真的固定目标、常数期望力版本，取 $k_f=0$。名义中心 $\theta_c$ 和 $T_c=T(\theta_c)$ 固定：

$$
\begin{aligned}
u_p &= k_p(p_d-p)-k_v v, \\
\beta &= -\gamma T_c^{\mathsf T}\Lambda v, \\
\bar\theta &= \hat\theta+\beta, \\
\bar n &= n(\bar\theta), \\
F &= \eta+\Lambda u_p-\bar n f_d, \\
\tau_a &= J^{\mathsf T}F, \\
\dot{\hat\theta} &= \gamma T_c^{\mathsf T}
\left[\Lambda u_p+\dot\Lambda v+\bar n(f-f_d)\right], \\
\hat\theta(0) &= \bar\theta(0)+\gamma T_c^{\mathsf T}\Lambda(0)v(0).
\end{aligned}
$$

只积分 $\hat\theta$，命令方向使用 $\bar\theta$。IAC1 在线不使用真实法向、真实角度误差或实测加速度；$\dot\Lambda$ 由机器人模型及关节位置/速度计算。

## 2. 需要复现的仿真

主入口为 [`fic1_nac1_iac1_simulation`](../../fic1_nac1_iac1_simulation.m)，在 `hmfc` 根目录运行：

```matlab
setup_project;
comparison = fic1_nac1_iac1_simulation;
```

三者共用初始位置、零初速、接触线上的固定目标和增益：目标切向位移 $8\,\mathrm{cm}$，$k_p=25\,\mathrm{s}^{-2}$、$k_v=10\,\mathrm{s}^{-1}$、$f_d=10\,\mathrm{N}$、$k_f=0$，运行 $6\,\mathrm{s}$，输出间隔 $0.002\,\mathrm{s}$。FIC1 使用真实法向；NAC1 固定偏差 $+5^\circ$；IAC1 初始有效偏差 $+5^\circ$，$\gamma=0.4\,\mathrm{rad}/(\mathrm{N}\cdot\mathrm{s})$、名义扇区 $\pm6^\circ$。

| 控制器 | 原 2R 仿真在 6 s 时的结果 | WAM 上需要观察的现象 |
| --- | --- | --- |
| FIC1 | 位置误差约 0，力 10 N | 已知法向的运动/力控制基准 |
| NAC1 | 切向误差 −56.656 mm，力 10.5691 N | 固定法向偏差导致运动偏置 |
| IAC1 | 位置误差约 0，力约 10 N，有效角度误差约 0 | 法向修正，位置和力向基准靠近 |

控制器源码分别为 [`fic_simulation.m`](../../fic_simulation.m)（`controllerIds=1`）、[`nac1_simulation.m`](../../nac1_simulation.m)、[`iac1_simulation.m`](../../iac1_simulation.m)。结果默认保存到 `numerical/results/fic1_nac1_iac1/`。

补充的大偏角对比入口为 [`fic1_nac1_iac1_max_angle_simulation`](../../fic1_nac1_iac1_max_angle_simulation.m)：NAC/IAC 初始偏差 $+15^\circ$，原模型扇区半宽约 $16.981237^\circ$。其中 NAC1 越出原认证区域，IAC1 仍收敛。该扇区必须按 WAM 惯量重新计算，不能直接移植。

原仿真使用理想 2R 参数和已建立的无摩擦刚性接触。WAM 应复现上述对比趋势，具体稳态偏差和收敛速度按 WAM 模型重新预测。

## 3. 从仿真到现有代码需要的转换

### 3.1 七关节 WAM → 两自由度模型 → 二维任务坐标

现有代码只驱动关节 **2、4**（C++ 索引 **1、3**）。设 $E=[e_2,e_4]$，$r=[q_2,q_4]^{\mathsf T}$；其余关节固定在 $q_{\mathrm{lock}}$（主动分量置零）。设固定任务平面正交基 $B=[e_x,e_z]$，并核对实际锁定姿态确实产生该运动平面：

$$
\begin{aligned}
q_7 &= q_{\mathrm{lock}}+Er, \\
M_2 &= E^{\mathsf T}M_7E, \\
h_2 &= E^{\mathsf T}h_7(q_7,E\dot r), \\
J &= B^{\mathsf T}J_{v,7}E, \\
p &= B^{\mathsf T}(p_{\mathrm{tool,base}}-o_{\mathrm{base}}), \\
v &= J\dot r.
\end{aligned}
$$

$M_7,h_7$ 应包含完整七连杆、腕部和工具负载；锁定不等于删除这些质量。$J$ 是可逆的 2×2 平移雅可比。若其余关节只是 PD 保持，上式为近似模型。

### 3.2 重力补偿 → 完整任务空间动力学补偿

按与仿真一致的定义计算：

$$
\begin{aligned}
K &= J^{-1}, \\
\Lambda &= K^{\mathsf T}M_2K, \\
\eta &= J^{-\mathsf T}h_2-\Lambda\dot J\dot r, \\
\dot K &= -K\dot JK, \\
\dot\Lambda &= \dot K^{\mathsf T}M_2K
+K^{\mathsf T}\dot M_2K+K^{\mathsf T}M_2\dot K.
\end{aligned}
$$

$\dot J,\dot M_2$ 是模型沿关节速度的时间导数。定义参考见 [`hmfc_robot_model.m`](../../shared/hmfc_robot_model.m)，但 WAM 必须换成自己的模型参数。现有 `hybrid_control/dynamics.hpp` 只处理前四关节且缩放了加速度，不能直接作为这里的完整 WAM7 模型。

三组控制器均可等价输出主动关节总力矩：

$$
\tau_a=h_2+M_2J^{-1}(u_p-\dot J\dot r)
-J^{\mathsf T}n_{\mathrm{used}}u_f.
$$

IAC1 中 $n_{\mathrm{used}}=\bar n,\quad u_f=f_d$。若保留现有 `wam.gravityCompensate()`，应减去系统实际另行叠加的重力贡献：

$$
\tau_{\mathrm{user},a}=\tau_a-E^{\mathsf T}g_{\mathrm{builtin},7}.
$$

其他关节继续采用原保持策略。已经计算出的 $\tau_{\mathrm{user},a}$ 直接装入七关节命令，不再经过一次 $J^{\mathsf T}$ 映射。

### 3.3 当前力反馈 → 正压缩标量力

当前 FP 反馈在坐标转换后取负，使用“机器人对环境”的力；上述控制器使用相反约定：

$$
\begin{aligned}
F_{\mathrm{env,base}} &= R_{\mathrm{base,sensor}}F_{\mathrm{sensor,corrected}}, \\
F_{\mathrm{env}}^{(2)} &= B^{\mathsf T}F_{\mathrm{env,base}}, \\
f &= n_{\mathrm{cal}}^{\mathsf T}F_{\mathrm{env}}^{(2)}, \\
f_d &= +10\,\mathrm{N}.
\end{aligned}
$$

IAC1 只接收标量 $f$，不接收 $n_{\mathrm{cal}}$。近似无摩擦时也可用 $\lVert F_{\mathrm{env}}^{(2)}\rVert$ 近似法向力；有切向摩擦时不能将合力幅值当作准确法向力。传感器力、位置和雅可比必须转换到同一坐标系，并修正工具负载造成的力读数。

### 3.4 现有 PD/PI 与参考 → 三控制器共用的调节任务

| 现有代码 | 需要的转换 |
| --- | --- |
| [`hybrid_fp_ft/controller.hpp`](../../../../contact-control/include/hybrid_fp_ft/controller.hpp) | 用第 1 节控制律替换选轴 PD/PI；保留完整二维惯量耦合，不叠加原力积分 |
| [`hybrid_ft/joint_torque.hpp`](../../../../contact-control/include/hybrid_ft/joint_torque.hpp) | 接收主动关节力矩并处理重力去重，保留其他关节的保持逻辑 |
| [`src/hybrid_fp_ft.cpp`](../../../../contact-control/src/hybrid_fp_ft.cpp) | 连接模型与正压缩力通道；将 move/sine 改为接触线上的固定目标 $p_d=p_0+T_{\mathrm{cal}}(0.08\,\mathrm{m})$ |

三者使用同一参考、初态和模型；NAC1 固定角度与 IAC1 初始有效角度一致。$k_p,k_v$ 单位为 $\mathrm{s}^{-2},\mathrm{s}^{-1}$，不能直接沿用原 FP 的 $\mathrm{N}/\mathrm{m}$、$\mathrm{N}\cdot\mathrm{s}/\mathrm{m}$ 增益。IAC1 按 execution manager 实际周期积分，并在同周期的 $\Lambda(0),v(0)$ 下初始化。
