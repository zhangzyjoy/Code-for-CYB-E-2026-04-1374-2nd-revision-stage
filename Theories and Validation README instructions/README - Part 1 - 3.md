# 1. $\text{  }$ Fixed-time rotational disturbance observer (FxTDO) <br/>

**Note for content organization:**

This markdown file **< README - Part 1 - 3.md >** includes comparative simulation studies against some representitive methods published recently in IEEE Transactions in **Chapter 1.4**. <br/> 

Please refer to **< README - Part 1 - 1.md >** for the Lyapunov-based theoretical instruction for rotational control scheme design according to the stability proof in **Chapter 1.1**, and also includes the implementation procedure of the proposed control scheme in **Chapter 1.2**. <br/>

Refer to **< README - Part 1 - 2.md >** for the parameter settings and validation of the proposed method in **Chapter 1.3**. <br/>

## 1.4 $\text{  }$ Validation for Comparison <br/>

Some approaches published in IEEE Transactions in recent years are introduced for comparison in this chapter. The proposed rotational FxTDO is compared against the following approaches : <br/>

[1]	Y. Yu, H. Xu, and X. Yao, “Disturbance rejection event-triggered robust model predictive control for tracking of constrained uncertain robotic manipulators,” *IEEE Transactions on Cybernetics*, vol. 54, no. 6, pp. 3540–3552, Jun. 2024. <br/>
[2]	K. Zhao, J. Zhang, D. Ma, and Y. Xia, “Composite disturbance rejection attitude control for quadrotor with unknown disturbance,” *IEEE Transactions on Industrial Electronics*, vol. 67, no. 8, pp. 6894–6903, Aug. 2020. <br/>
[3]	X. Liu, X. Zhang, F. Xu, S. Gu, and J. Zhang, “Fixed-time angle tracking control for multi-DOF manipulator driven by pneumatic artificial muscles,” *IEEE Transactions on Industrial Electronics*, vol. 72, no. 4, pp. 4137–4146, Apr. 2025. <br/>

Each agent in the UAV team is considered as a quadrotor, and the rotational dynamics that contains the disturbance term is denoted as

$$
\begin{aligned}
\dot \varpi _i = ( \Lambda _i ) ^{-1} ( -( \varpi _i ) _{\times} \Lambda _i \varpi _i + \tau _i + d _i ^{\varpi} )
\end{aligned}
\quad\quad(4.1)
$$<br/>

The rotational FxTDO in Chapter 2.2 is proposed for estimating the **external disturbance term $d _i ^{ \varpi }$**. The **first-order rotational disturbance observers** from the three aforementioned references are employed as comparison against the rotational FxTDO. The detailed implementation procedure of the first-order rotational disturbance observers in Ref.[1] ~ Ref.[3] are elaborated as follows for comparison. <br/> <br/>

**Firstly, consider the first-order external disturbance observer in Ref.[1] :** <br/>

[1]	Y. Yu, H. Xu, and X. Yao, “Disturbance rejection event-triggered robust model predictive control for tracking of constrained uncertain robotic manipulators,” *IEEE Transactions on Cybernetics*, vol. 54, no. 6, pp. 3540–3552, Jun. 2024. <br/>

In paper *IEEE TCYB (2024)*, the disturbance observer for Eq.(4.1) is depicted as

$$
\begin{aligned}
\begin{cases}
& \dot { \hat { \varepsilon } } _{ i } ^{ \varpi } = - { ( \Lambda _{ i } ) } ^{ -1 } r _{ i } ^{ \varpi } ( \hat { \varepsilon } _{ i } ^{ \varpi } - { ( \varpi _{ i } ) } _{ \times } \Lambda _{ i } \varpi _{ i } + \tau _{ i } + r _{ i } ^{ \varpi } \varpi _{ i } ) \\
& \hat { d } _{ i } ^{ \varpi } = \hat { \varepsilon } _{ i } ^{ \varpi } + r _{ i } ^{ \varpi } \varpi _{ i }
\end{cases}
\end{aligned}
\quad\quad(4.2)
$$<br/>


**Secondly, consider the first-order external disturbance observer in Ref.[2] :** <br/>

[2]	K. Zhao, J. Zhang, D. Ma, and Y. Xia, “Composite disturbance rejection attitude control for quadrotor with unknown disturbance,” *IEEE Transactions on Industrial Electronics*, vol. 67, no. 8, pp. 6894–6903, Aug. 2020. <br/>

The rotational dynamics Eq.(4.1) is reformulated as $\Lambda _{ i } \dot \varpi _{ i } = -( \varpi _{ i } ) _{\times} \Lambda _{ i } \varpi _{ i } + \tau _{ i } + d _{ i } ^{\varpi}$. Define the first-order state variable as $x _{ i } ^{ 1 } = \Lambda _{ i } \varpi _{ i }$ and the second-order state variable as $x _{ i } ^{ 2 } = d _{ i } ^{ \varpi }$. For simplicity we define an auxiliary control input other than $\tau _{ i }$ as $\tilde { \tau } _{ i } = - { ( \varpi _{ i } ) } _{ \times } \Lambda _{ i } \varpi _{ i } + \tau _{ i }$, $b _{ i } = I _{ 3 }$, and $\dot { x } _{ i } ^{ 2 } = \dot { d } _{ i } ^{ \varpi } = D _{ i } ^{ \varpi }$. The extended state system is denoted as

$$
\begin{aligned}
\begin{cases}
& \dot { x } _{ i } ^{ 1 } = x _{ i } ^{ 2 } + b _{ i } \tilde { \tau } _{ i } \\
& \dot { x } _{ i } ^{ 2 } = D _{ i } ^{ \varpi }
\end{cases}
\end{aligned}
$$<br/>

To facilitate the design of the disturbance observer, a piecewise continuous function is employed as

$$
\begin{aligned}
\phi ( x _{ k } ) = 
\begin{cases}
& -1 , \quad \quad x _{ k } \le - \pi / 2 \\
& sin ( x _{ k } ), \quad  - \pi / 2 \le x _{ k } \le \pi / 2 \\
& 1 , \quad \quad x _{ k } \ge \pi / 2
\end{cases}
\end{aligned}
$$<br/>

and the vector form is defined as $\phi ( x ) = { [ \phi ( x _{ 1 } ) , \phi ( x _{ 2 } ), ... , \phi ( x _{ n } ) ] ^{ T } }$

We introduce $\hat { x } _{ i } ^{ 1 }$ and $\hat { x } _{ i } ^{ 2 }$ to estimate $x _{ i } ^{ 1 }$ and $x _{ i } ^{ 2 }$, respectively. The rotational disturbance observation is provided as

$$
\begin{aligned}
\begin{cases}
& \dot { \hat { x } } _{ i } ^{ 1 } = - { ( \varpi _{ i } ) } _{ \times } \Lambda _{ i } \varpi _{ i } + \tau _{ i } + \hat { x } _{ i } ^{ 2 } + \frac { l _{ i } ^{ 1 } } { \epsilon _{ i } } ( x _{ i } ^{ 1 } - \hat { x } _{ i } ^{ 1 } ) \\
& \dot { \hat { x } } _{ i } ^{ 2 } = \frac { l _{ i } ^{ 2 } } { { ( \epsilon _{ i } ) } ^{ 2 } } ( x _{ i } ^{ 1 } - \hat { x } _{ i } ^{ 1 } ) + \frac { 1 } { \epsilon _{ i } } \phi ( \kappa _{ i } \cdot \frac { x _{ i } ^{ 1 } - \hat { x } _{ i } ^{ 1 } } { \epsilon _{ i } } )
\end{cases}
\end{aligned}
$$<br/>

Substitute $x _{ i } ^{ 1 } = \Lambda _{ i } \varpi _{ i }$, $x _{ i } ^{ 2 } = d _{ i } ^{ \varpi }$, $\hat { x } _{ i } ^{ 1 } = \Lambda _{ i } \hat { \varpi } _{ i }$ and $\hat { x } _{ i } ^{ 2 } = \hat { d } _{ i } ^{ \varpi }$ into the observer above and then yield the proper form of the first-order rotational disturbance observer in paper *IEEE TIE (2020)* as

$$
\begin{aligned}
\begin{cases}
& \Lambda _{ i } \dot { \hat { \varpi } } _{ i } = - { ( \varpi _{ i } ) } _{ \times } \Lambda _{ i } \varpi _{ i } + \tau _{ i } + \hat { d } _{ i } ^{ \varpi } + \frac { l _{ i } ^{ 1 } } { \epsilon _{ i } } \Lambda _{ i } ( \varpi _{ i } - \hat { \varpi } _{ i } ) \\
& \dot { \hat { d } } _{ i } ^{ \varpi } = \frac { l _{ i } ^{ 2 } } { { ( \epsilon _{ i } ) } ^{ 2 } } \Lambda _{ i } ( \varpi _{ i } - \hat { \varpi } _{ i } ) + \frac { 1 } { \epsilon _{ i } } \phi ( \kappa _{ i } \cdot \Lambda _{ i } \frac { \varpi _{ i } - \hat { \varpi } _{ i } } { \epsilon _{ i } } )
\end{cases}
\end{aligned}
\quad\quad(4.3)
$$<br/>


**Thirdly, consider the first-order external disturbance observer in Ref.[3] :** <br/>

[3]	X. Liu, X. Zhang, F. Xu, S. Gu, and J. Zhang, “Fixed-time angle tracking control for multi-DOF manipulator driven by pneumatic artificial muscles,” *IEEE Transactions on Industrial Electronics*, vol. 72, no. 4, pp. 4137–4146, Apr. 2025. <br/>

The rotational dynamics Eq.(4.1) is reformulated as $\Lambda _{ i } \dot \varpi _{ i } = -( \varpi _{ i } ) _{\times} \Lambda _{ i } \varpi _{ i } + \tau _{ i } + d _{ i } ^{\varpi}$. Select $p _{ 1 } > 0$, $\lambda _{ 1 } > 0$, $\lambda _{ 2 } > 0$, $1 / 2 < \mu < 1$. In paper *IEEE TIE (2025)*, the rotational disturbance observer is formulated as

$$
\begin{aligned}
\begin{cases}
& \Lambda _{ i } \dot { \hat { \varpi } } _{ i } = - { ( \varpi _{ i } ) } _{ \times } \Lambda _{ i } \varpi _{ i } + \tau _{ i } + \hat { d } _{ i } ^{ \varpi } + \lambda _{ 1 } ( \frac { \Lambda _{ i } ( \varpi _{ i } - \hat { \varpi } _{ i } ) } { p _{ 1 } } + \lt \frac { \Lambda _{ i } ( \varpi _{ i } - \hat { \varpi } _{ i } ) } { p _{ 1 } } \gt ^{ \mu } + \lt \frac { \Lambda _{ i } ( \varpi _{ i } - \hat { \varpi } _{ i } ) } { p _{ 1 } } \gt ^{ 2 - \mu } ) \\
& \hat { d } _{ i } ^{ \varpi } = \frac { \lambda _{ 2 } } { p _{ 1 } } ( \frac { \Lambda _{ i } ( \varpi _{ i } - \hat { \varpi } _{ i } ) } { p _{ 1 } } + \lt \frac { \Lambda _{ i } ( \varpi _{ i } - \hat { \varpi } _{ i } ) } { p _{ 1 } } \gt ^{ 2 \mu - 1 } + \lt \frac { \Lambda _{ i } ( \varpi _{ i } - \hat { \varpi } _{ i } ) } { p _{ 1 } } \gt ^{ 3 - 2 \mu } )
\end{cases}
\end{aligned}
\quad\quad(4.4)
$$<br/>

where $\lt \cdot \gt$ is defined as an operator to apply on each entry of a vector. <br/>

The comparison simulation is promoted by reproducing the disturbance observer from Ref.[1], and the simulation results are depicted by <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Rotational%20Disturbance%20Observation/Results%20IEEE_TCYB_2024/Disturbunce%20Observation%20IEEE_TCYB_2024.png) <br/> <br/>

Fig.1 $\quad$ Rotational Disturbance Observation by Ref.[1]. <br/> <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Rotational%20Disturbance%20Observation/Results%20IEEE_TIE_2020/Disturbunce%20Observation%20IEEE_TIE_2020.png) <br/> <br/>

Fig.2 $\quad$ Rotational Disturbance Observation by Ref.[2]. <br/> <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Rotational%20Disturbance%20Observation/Results%20IEEE_TIE_2025/Disturbunce%20Observation%20IEEE_TIE_2025.png) <br/> <br/>

Fig.3 $\quad$ Rotational Disturbance Observation by Ref.[3]. <br/> <br/>

