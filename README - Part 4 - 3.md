# 4. $\text{  }$ Fixed-time translational disturbance observer (FxTDO) <br/>

This markdown file **< README - Part 4 - 3.md >** includes comparative simulation studies in **Chapter 4.4** against some representitive disturbance observers published recently in IEEE Transactions. <br/> 

Please refer to **< README - Part 4 - 1.md >** for the Lyapunov-based theoretical instruction for the disturbance observer design according to the stability proof in **Chapter 4.1**, and also for the implementation procedure of the proposed disturbance observers in **Chapter 4.2**. <br/>

Refer to **< README - Part 4 - 2.md >** for the parameter settings and validation of the proposed disturbance observer in **Chapter 4.3**. <br/> <br/>


## 4.4 $\text{  }$ Validation for comparison

Some approaches published in IEEE Transactions in recent years are introduced for comparison in this chapter. The proposed translational FxTDO is compared against the following approaches : <br/>

[1]	K. Zhao, J. Zhang, D. Ma, and Y. Xia, “Composite nonlinear extended state observer-based trajectory tracking control for quadrotor under input constraints,” *IEEE Transactions on Circuits and Systems I, Regular Papers*, vol. 70, no. 10, pp. 4126–4136, Oct. 2023. <br/>
[2]	A. Zou, Y. Liu, Z. Hou, and Z. Hu, “Practical predefined-time output-feedback consensus tracking control for multiagent systems,” *IEEE Transactions on Cybernetics*, vol. 53, no. 8, pp. 5311–5322, Aug. 2023. <br/>
[3]	B. Tian, H. Zhang, Z. Wang, and H. Yan, “Fixed-time formation-containment of nonlinear systems using intermittent output and connectivity,” *IEEE Transactions on Industrial Electronics*, vol. 72, no. 1, pp. 845–856, Jan. 2025. <br/>

Each agent in the UAV team is considered as a quadrotor, and the translational dynamics that contains the disturbance term is denoted as

$$
\begin{aligned}
\begin{cases}
&\dot p _i = v _i  \\
&\dot v _i = - g \overline e _3 + u _{ i } + \frac { T _i } { m _i } R ( Q _i ^c ) ( R ( Q _i ^e ) - I _3 ) \overline e _3 + d _i ^v
\end{cases}
\end{aligned}
\quad\quad(4.1)
$$<br/>

The translational FxTDO in Chapter 4.2 is proposed for estimating the **external disturbance term $d _i ^{ v }$**. The **first-order translational disturbance observers** from the three aforementioned references are employed as comparison against the translational FxTDO. The detailed implementation procedure of the second-order translational disturbance observers in Ref.[1] ~ Ref.[3] are elaborated as follows for comparison. <br/> <br/>

**Firstly, consider the first-order external disturbance observer in Ref.[1] :** <br/>

[1]	K. Zhao, J. Zhang, D. Ma, and Y. Xia, “Composite nonlinear extended state observer-based trajectory tracking control for quadrotor under input constraints,” *IEEE Transactions on Circuits and Systems I, Regular Papers*, vol. 70, no. 10, pp. 4126–4136, Oct. 2023. <br/>

The extended state variables in the disturbance observer are defined as $x _i ^1 = p _i$, $x _i ^2 = v _i$, and $x _i ^3 = d _i ^v$. Moreover, the estimation value of the extended states are denoted as $\hat { x } _i ^1 = \hat { p } _i$, $\hat { x } _i ^2 = \hat { v } _i$, and $\hat { x } _i ^3 = \hat { d } _i ^v$. In the paper *IEEE TCAS-I (2023)*, the disturbance observer for Eq.(4.1) is depicted as

$$
\begin{aligned}
\begin{cases}
& { \dot { \hat { x } } _i ^1 } = { \hat { x } _i ^2 + l _1 [ \frac { x _i ^1 - \hat { x } _i ^1 } { \epsilon _i } + \epsilon _i l _0 \ell _1 G ( \frac { x _i ^1 - \hat { x } _i ^1 } { \ell _{ 1 } { ( \epsilon _i ) } ^{ 2 } } ) ] } \\
& { \dot { \hat { x } } _i ^2 } = \hat { x } _i ^3 + l _2 [ \frac { x _i ^1 - \hat { x } _i ^1 } { { ( \epsilon _i ) } ^{ 2 } } + { ( l _0 ) } ^2 \ell _{ 2 } G ( \frac { x _i ^1 - \hat { x } _i ^1 } { \ell _{ 2 } { ( \epsilon _i ) } ^2 } ) ] \\
& \quad \quad + [ - g \overline { e } _3 + u _i + \frac { T _i } { m _i } R ( Q _i ^c ) ( R ( Q _i ^e ) - I _3 ) \overline { e } _3 ] \\
& { \dot { \hat { x } } _i ^3 } = { l _3 [ \frac { x _i ^1 - \hat { x } _i ^1 } { { ( \epsilon _i ) } ^3 } + \frac { { ( l _0 ) } ^3 } { \ell _{ 3 } \epsilon _i } G ( \ell _3 \frac { x _i ^1 - \hat { x } _i ^1 } { { ( \epsilon _i ) } ^2 } ) ] }
\end{cases}
\end{aligned}
$$<br/>

where the nonlinear function $G ( s ) = \tanh ( \lvert s \rvert ^{ 1 - \alpha } sign ( s ) )$ is defined with $0 < \alpha < 1$, and $\tilde { x } _1 ( t ) = x _1 ( t ) - \hat { x } _1 ( t )$ denotes the observation error. Substitute $\hat { x } _i ^j$ and $x _i ^j$ with the corresponding definition and further yield the external disturbance observer in *IEEE TCAS-I ( 2023 )* as

$$
\begin{aligned}
\begin{cases}
& { \dot { \hat { p } } _i } = { \hat { v } _i + l _1 [ \frac { p _i - \hat { p } _i } { \epsilon _i } + \epsilon _{ i } l _{ 0 } \ell _{ 1 } G ( \frac { p _i - \hat { p } _i } { \ell _{ 1 } { ( \epsilon _i ) } ^{ 2 } } ) ] } \\
& { \dot { \hat { v } } _i } = \hat { d } _i ^{ v } + l _{ 2 } [ { ( l _0 ) } ^2 \ell _{ 2 } G ( \frac { p _i - \hat { p } _i } { \ell _{ 2 } { ( \epsilon _i ) } ^2 } ) ] \\
& \quad \quad + [ - g \overline { e } _3 + u _i + \frac { T _i } { m _i } R ( Q _i ^c ) ( R ( Q _i ^e ) - I _3 ) \overline { e } _3 ] \\
& { \dot { \hat { d } } _i ^v } = { l _3 [ \frac { p _i - \hat { p } _i } { { ( \epsilon _i ) } ^3 } + \frac { { ( l _0 ) } ^3 } { \ell _{ 3 } \epsilon _i } G ( \ell _3 \frac { p _i - \hat { p } _i } { { ( \epsilon _i ) } ^2 } ) ] }
\end{cases}
\end{aligned}
\quad\quad(4.2)
$$<br/>




