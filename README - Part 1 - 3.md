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

Each agent in the UAV team is considered as a quadrotor, and its rotational subsystem can be decoupled as

$$
\begin{aligned}
\begin{cases}
&\dot R ( Q _i ) = R ( Q _i ) ( \varpi _i ) _{\times}  \\
&\dot \varpi _i = ( \Lambda _i ) ^{-1} ( -( \varpi _i ) _{\times} \Lambda _i \varpi _i + \tau _i + d _i ^{\varpi} )
\end{cases}
\end{aligned}
$$<br/>

The rotational FxTDO in Chapter 2.2 is proposed for estimating the **external disturbance term $d _i ^{ \varpi }$**. The **first-order rotational disturbance observers** from the three aforementioned references are employed as comparison against the rotational FxTDO. The detailed implementation procedure of the first-order rotational disturbance observers in Ref.[1] ~ Ref.[3] are elaborated as follows for comparison. <br/>

[1]	Y. Yu, H. Xu, and X. Yao, “Disturbance rejection event-triggered robust model predictive control for tracking of constrained uncertain robotic manipulators,” *IEEE Transactions on Cybernetics*, vol. 54, no. 6, pp. 3540–3552, Jun. 2024. <br/> <br/>

In the paper IEEE TCYB (2024), the second-order system dynamics can be formulated as

$$
\begin{aligned}
\begin{cases}
& \dot { x } _{ 1 } = x _{ 2 } \\
& \dot { x } _{ 2 } = - M ^{ -1 } ( x _{ 1 } ) H ( x _{ 1 }, x _{ 2 } ) + M ^{ -1 } ( x _{ 1 } ) \tau + M ^{ -1 } ( x _{ 1 } ) d
\end{cases}
\end{aligned}
$$<br/>



