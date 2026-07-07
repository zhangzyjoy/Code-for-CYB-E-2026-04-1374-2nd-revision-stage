We upload **Matlab/Simulink** source code for **simulation** and **Python/ROS** code package for **experimental validation** in this webpage link to enhance the instructions on the relationship between theoretical contribution and further verification according to the reviewers' suggestions for our revised manuscript for **IEEE Transactions on Cybernetics** with a submission ID **CYB-E-2026-04-1374**. <br/> 

**Note : Source code and instructions for the first revision under Revised & Resubmit decision can be accessed at https://github.com/zhangzyjoy/Code-for-manuscript-revision-stage.git.** <br/>

This link demonstrates the proposed control approach for the formation control issue of uncrewed aerial vehicle (UAV) teams. <br/>

To address the reviewer's comments, the logical connection between theory and validation has been strengthened. The narrative follows a systematic progression. <br/> 
First, the theoretical basis for controller design is introduced using fundamental lemmas. <br/> 
Next, the derived parameter bounds for system stability are presented. <br/> 
Then, the parameter configurations for simulation or experimental validation are specified. <br/> 
Finally, the simulation or experimental validation results are presented and discussed. <br/> <br/> 

**Each section in this link follows the logic above, where the link theory-validation is enhanced.** <br/> 

**This link includes six main sections.** <br/>
Each of the latter five ones indicates a specific module of the proposed control scheme. <br/>
1. Significant lemmas and deductions. <br/>
2. Fixed-time rotational disturbance observer (FxTDO). <br/>
3. Nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC). <br/>
4. Practical fixed-time distributed state observer (PFxTDSO). <br/>
5. Fixed-time translational disturbance observer (FxTDO). <br/>
6. Practical fixed-time decentralized formation controller (PFxTDFC). <br/> <br/>

Each section contains four chapters, including : <br/>
1. Theories and design principles. <br/>
2. Implementation procedure. <br/>
3. Parameter settings and validation. <br/>
4. Validation for Comparison. <br/> <br/>

**Note : the validation part can be divided into numerical simulation validation and experimental validation.** <br/> **Specifically, the rotational control (NLSMAC) and rotational disturbance observation (Rotational FxTDO) are validated only through numerical simulations, whereas the translational control (PFxTDFC), translational distributed observer (PFxTDSO), and translational disturbance observer (Translational FxTDO) are validated through both numerical simulations and experiments.** <br/> <br/>

# 1. Significant Lemmas and Deductions <br/>

## 1.1 Fixed-time / Practical fixed-time stability lemmas <br/>

Consider a nonlinear system as <br/>

$$
\begin{aligned}
\dot x ( t ) = f ( x ( t ) ) , x ( t _0 ) = x _0
\end{aligned}
\quad\quad(1.1)
$$<br/>

where $t _0$ is the initial time and $x ( t _0 )$ is the state vector, and $f ( x ( t ) )$ is a nonlinear function of system dynamics. <br/>

**Lemma 1 (Fixed-Time Stability)** : If a nonlinear function $V ( t , x ) : \mathbb R \times \mathbb R ^n \to \mathbb R ^+ \cup \lbrace 0 \rbrace$ is defined as positive definite and radially unbounded, and $\dot V ( t , x ) \le - a _1 ( V ( t, x ) ^ { b _1 } ) - a _2 ( V ( t, x ) ^ { b _2 } )$ holds such that $a _1 >0$, $a _2 >0$, $b _1 > 1$, $0 < b _2 < 1$, respectively, then the origin of (1) is fixed-time stable and the settling time is bounded by <br/>

$$
\begin{aligned}
T _s \le \overline T _s = \frac { 1 } { a _1 ( b _1 - 1 )} + \frac { 1 } { a _2 ( 1 - b _2 ) }
\end{aligned}
$$<br/>

**Lemma 2 (Practical Fixed-Time Stability)** : If a radially unbounded positive definite function satisfies $\dot V ( t , x ) \le - a _1 ( V ( t, x ) ^ { b _1 } ) - a _2 * ( V ( t, x ) ^ { b _2 } ) + c _0$ such that $a _1 >0$, $a _2 >0$, $b _1 > 1$, $0 < b _2 < 1$, and $c _0 > 0$, the origin of (1) is practical fixed-time stable. There exists a constant $0 < \eta _0 < 1$ such that the settling time fulfills

$$
\begin{aligned}
T _s \le \overline T _s = \frac { 1 } { a _1 \eta _0 ( b _1 - 1 )} + \frac { 1 } { a _2 \eta _0 ( 1 - b _2 ) }
\end{aligned}
$$<br/>

**Lemma 3 (Proposed Fixed-Time Stability Criteria)** : If a radially unbounded positive definite function $V ( t , x )$ satisfies $\dot V \le -k \frac { V ^{ ( p + 2 ) / 2 } } { \tanh ( \gamma V ^{ 1 / 2 } ) }$ such that $k > 0$, $\gamma > 0$, $0 < p < 1$, then the origin of $x ( t )$ converges in fixed time upper bounded by

$$
\begin{aligned}
T _s \le \overline T _s = \frac { 2 ^{ p + 2 } } { \gamma ^{ p + 2 } k p ( 1 - p ) }
\end{aligned}
$$<br/>


## 1.2 Design of nonlinear smooth sigmoid function <br/>

A newly introduced **nonlinear smooth sigmoid vector** is defined as <br/>

$$
\begin{aligned}
\vartheta ( \overline x, \alpha, \gamma ) = [ \vartheta _k ( \overline x _1, \alpha, \gamma ), ..., \vartheta _k ( \overline x _n, \alpha, \gamma ) ] ^T
\end{aligned}
\quad\quad(1.2)
$$<br/>

with each entry denoted as <br/>

$$
\begin{aligned}
\vartheta _k ( \overline x _k, \alpha, \gamma ) = \lvert x _k \rvert ^ \alpha \lambda _k ( x _k, \gamma )
\end{aligned}
\quad\quad(1.3)
$$<br/>

$\lambda _k ( x _k, \gamma )$ is a sigmoid-like bounded function defined as <br/>

$$
\begin{aligned}
\lambda _k ( x _k, \gamma ) = -1 + \frac 1 {1 + \exp ( - \gamma x _k )}
\end{aligned}
\quad\quad(1.4)
$$<br/>

where $\gamma > 0$ controls the growth rate around the zero crossing. <br/>

**Lemma 4** : For a vector $\overline x = [ x _1, ..., x _n ] ^T \in \mathbb R ^n$ , $\alpha > 0$, and $\gamma > 0$, define $\vartheta ( \overline x , \alpha, \gamma ) = [ \vartheta _1 ( x _1, \alpha, \gamma ), ..., \vartheta _n ( x _n, \alpha, \gamma ) ]^T$ with each entries $\vartheta _k ( x _k, \alpha, \gamma )$ as the form of Eq.(3), the inequalities are yielded as

$$
\begin{aligned}
\begin{cases}
&- ( \overline x ) ^T \vartheta ( \overline x, \alpha, \gamma ) \le - 2 ^{ \frac { \alpha + 1 } { 2 } } K _{ \alpha } ( \frac { ( \overline x ) ^T \overline x } { 2 } ) ^{ \frac { \alpha + 1 } { 2 } } ), \quad\quad ( 0 < \alpha \le 1 ) \\
&- ( \overline x ) ^T \vartheta ( \overline x, \alpha, \gamma ) \le - 2 ^{ \frac { \alpha + 1 } { 2 } } n ^{ \frac { 1 - \alpha } { 2 } } K _{ \alpha } ( \frac { ( \overline x ) ^T \overline x } { 2 } ) ^{ \frac { \alpha + 1 } { 2 } } ), \quad\quad ( \alpha > 1 )
\end{cases}
\end{aligned}
\quad\quad(1.5)$$<br/>

where $K _{ \alpha } = min \lbrace \frac { 1 } { \overline \epsilon } , \underline \epsilon \rbrace$, $\overline \epsilon$ and $\underline \epsilon$ are positive constants. <br/>

## 1.3 System dynamics of a quadrotor UAV <br/>

Each agent in the UAV team is considered as a quadrotor, which can be decoupled into rotational and translational subsystems as

$$
\begin{aligned}
\begin{cases}
&\dot R ( Q _i ) = R ( Q _i ) ( \varpi _i ) _{\times}  \\
&\dot \varpi _i = ( \Lambda _i ) ^{-1} ( -( \varpi _i ) _{\times} \Lambda _i \varpi _i + \tau _i + d _i ^{\varpi} )
\end{cases}
\end{aligned}
\quad\quad(1.6)$$<br/>

$$
\begin{aligned}
\begin{cases}
&\dot p _i = v _i  \\
&\dot v _i = - g \overline e _3 + \frac { T _i } { m _i } R ( Q _i ) \overline e _3 + d _i ^v
\end{cases}
\end{aligned}
\quad\quad(1.7)$$<br/>

Each quadcopter UAV node is decoupled into rotational and translational subsystems. <br/> <br/> 

Details are illustrated as follows. <br/>
<1> For the translational subsystem, a distributed practical fixed-time formation controller (PFxTDFC) is proposed to achieve formation consensus. <br/> 
<2> Practical fixed-time distributed state observers (PFxTDSO) are developed to estimate the desired velocity and position for each follower UAV and to maintain fully decentralized realization. <br/> 
<3> Leveraging the logarithmic mapping of rotational errors in Lie algebra space, a nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC) is developed to attain practical fixed-time singularity-free anti-disturbance attitude tracking in rotational subsystem. <br/> 
<4> Fixed-time disturbance observers (FxTDO) is promoted to compensate external disturbances in both rotational and translational subsystems. <br/> <br/> 

# 2. Fixed-time rotational disturbance observer (FxTDO) <br/>

In this section, a disturbance observer is developed to estimate the unknown disturbance $d _i ^{\varpi}$ in the rotational dynamics **Eq.(1.6)** and the observation value $\hat d _i ^{ \varpi }$ can be utilized as compensation term in the design of torque control input in the rotational subsystem. <br/>

## 2.1 Theories and design principles <br/>

Define a **virtual angular velocity tracking variable** as <br/>

$$
\begin{aligned}
\dot \sigma _i ^{ \varpi } = ( \Lambda _i ) ^{-1} ( -( \varpi _i ) _{\times} \Lambda _i \varpi _i + \tau _i + h _i ^{ \varpi, 3 } \overline \sigma _i ^{ \varpi } )
\end{aligned}
\quad\quad(2.1)
$$<br/>

Define the **auxiliary angular velocity tracking error** as $\overline \sigma _i ^{ \varpi } = \varpi _i - \sigma _i ^{ \varpi }$. Subtract **Eq.(2.1)** from the second differential equations of **Eq.(1.6)** and yield the first derivative of $\overline \sigma _i ^{ \varpi }$ as <br/>

$$
\begin{aligned}
\dot {\overline { \sigma } } _i ^{ \varpi } = ( \Lambda _i ) ^{-1} ( d _i ^{ \varpi } - h _i ^{ \varpi, 3 } \overline \sigma _i ^{ \varpi } )
\end{aligned}
$$<br/>

Introduce an observation value of $\overline { \sigma } _i ^{ \varpi }$, the auxiliary angular velocity tracking error, as $\tilde {\overline { \sigma } } _i ^{ \varpi } = \overline { \sigma } _i ^{ \varpi } - \hat {\overline { \sigma } } _i ^{ \varpi }$. From $\tilde d _i ^{ \varpi } = d _i ^{ \varpi } - \hat d _i ^{ \varpi } = h _i ^{ \varpi, 3 } \tilde {\overline { \sigma } } _i ^{ \varpi }$ , it can be derived that the observation error $\tilde d _i ^{ \varpi }$ holds the same convergence behavior with $\tilde {\overline { \sigma } } _i ^{ \varpi }$. <br/>

In order to achieve $\tilde d _i ^{ \varpi } \to 0$ , an equivalent deduction of $\tilde {\overline { \sigma } } _i ^{ \varpi } \to 0$ are required to be attained in fixed time. A Lyapunov candidate is chosen as $V _i ^{ d, \varpi } = \frac {1} {2} ( \tilde {\overline { \sigma } } _i ^{ \varpi } )^T \tilde {\overline { \sigma } } _i ^{ \varpi }$. According to **Lemma 1**, the inequality $\dot V _i ^{ d, \varpi } \le - a _i ^{ d, 1 } ( V _i ^{ d, \varpi } ) ^{ b _i ^{ d, 1 } } - a _i ^{ d, 2 } ( V _i ^{ d, \varpi } ) ^{ b _i ^{ d, 2 } }$ should be fulfilled to achieve a fixed-time stable observation. <br/>

According to **Lemma 4**, choose parameters as $\alpha _1 ^{ \varpi } > 1$, $0 < \alpha _2 ^{ \varpi } < 1$, $\mu _d ^{ \varpi }$, and then yield <br/>

$$
\begin{aligned}
\begin{cases}
&( \tilde {\overline { \sigma } } _i ^{ \varpi } )^T \vartheta ( \tilde {\overline { \sigma } } _i ^{ \varpi }, \alpha _1 ^{ \varpi }, \mu _d ^{ \varpi } ) \le - 2 ^{ \frac { \alpha _1 ^{ \varpi } + 1 } { 2 } } 3 ^{ \frac { 1 - \alpha _1 ^{ \varpi } } { 2 } } K _{ \alpha } ^{ d, \varpi } ( \frac { 1 } { 2 } ( ( \tilde {\overline { \sigma } } _i ^{ \varpi } ) ^T \tilde {\overline { \sigma } } _i ^{ \varpi } ) ^{ \frac { \alpha _1 ^{ \varpi } + 1 } { 2 } } ) \\
&( \tilde {\overline { \sigma } } _i ^{ \varpi } )^T \vartheta ( \tilde {\overline { \sigma } } _i ^{ \varpi }, \alpha _2 ^{ \varpi }, \mu _d ^{ \varpi } ) \le - 2 ^{ \frac { \alpha _2 ^{ \varpi } + 1 } { 2 } } K _{ \alpha } ^{ d, \varpi } ( \frac { 1 } { 2 } ( ( \tilde {\overline { \sigma } } _i ^{ \varpi } ) ^T \tilde { \overline { \sigma } } _i ^{ \varpi } ) ^{ \frac { \alpha _2 ^{ \varpi } + 1 } { 2 } } )
\end{cases}
\end{aligned}
$$<br/>

Substitute the definition of the **Lyapunov function** $V _i ^{ d, \varpi } = \frac {1} {2} ( \tilde {\overline { \sigma } } _i ^{ \varpi } )^T \tilde {\overline { \sigma } } _i ^{ \varpi }$, and the inequalities above can be thereafter yielded as

$$
\begin{aligned}
\begin{cases}
&( \tilde {\overline { \sigma } } _i ^{ \varpi } )^T \vartheta ( \tilde {\overline { \sigma } } _i ^{ \varpi }, \alpha _1 ^{ \varpi }, \mu _d ^{ \varpi } ) \le - 2 ^{ \frac { \alpha _1 ^{ \varpi } + 1 } { 2 } } 3 ^{ \frac { 1 - \alpha _1 ^{ \varpi } } { 2 } } K _{ \alpha } ^{ d, \varpi } ( ( V _i ^{ d, \varpi } ) ^{ \frac { \alpha _1 ^{ \varpi } + 1 } { 2 } } ) \\
&( \tilde {\overline { \sigma } } _i ^{ \varpi } )^T \vartheta ( \tilde {\overline { \sigma } } _i ^{ \varpi }, \alpha _2 ^{ \varpi }, \mu _d ^{ \varpi } ) \le - 2 ^{ \frac { \alpha _2 ^{ \varpi } + 1 } { 2 } } K _{ \alpha } ^{ d, \varpi } ( ( V _i ^{ d, \varpi } ) ^{ \frac { \alpha _2 ^{ \varpi } + 1 } { 2 } } )
\end{cases}
\end{aligned}
\quad\quad(2.2)
$$<br/>

The **coefficients** in **Lemma 1** for fixed-time stability analysis can be chosen as <br/>

$$
\begin{aligned}
\begin{cases}
&a _i ^{ d, 1 } = 2 ^{ \frac { \alpha _1 ^{ \varpi } + 1 } { 2 } } 3 ^{ \frac { 1 - \alpha _1 ^{ \varpi } } { 2 } } K _{ \alpha } ^{ d, \varpi } h _i ^{ \varpi, 1 } \\
&b _i ^{ d, 1 } = \frac { \alpha _1 ^{ \varpi } + 1 } { 2 } \\
&a _i ^{ d, 2 } = 2 ^{ \frac { \alpha _2 ^{ \varpi } + 1 } { 2 } } K _{ \alpha } ^{ d, \varpi } h _i ^{ \varpi, 2 } \\
&b _i ^{ d, 2 } = \frac { \alpha _2 ^{ \varpi } + 1 } { 2 }
\end{cases}
\end{aligned}
\quad\quad(2.3)
$$<br/>

then the **adaptive updating law** of the **auxiliary angular velocity observation error** is designed as <br/>

$$
\begin{aligned}
\dot { \tilde {\overline { \sigma } } } _i = - h _i ^{ \varpi, 1 } \vartheta ( \tilde {\overline { \sigma } } _i ^{ \varpi }, \alpha _1 ^{ \varpi }, \mu _d ^{ \varpi } ) - h _i ^{ \varpi, 2 } \vartheta ( \tilde {\overline { \sigma } } _i ^{ \varpi }, \alpha _2 ^{ \varpi }, \mu _d ^{ \varpi } )
\end{aligned}
\quad\quad(2.4)
$$<br/>

Since the **auxiliary angular velocity observation vector** is defined as $\tilde {\overline { \sigma } } _i ^{ \varpi } = \overline { \sigma } _i ^{ \varpi } - \hat {\overline { \sigma } } _i ^{ \varpi }$, the adaptive law is derived as <br/>

$$
\begin{aligned}
\dot { \hat {\overline { \sigma } } } _i ^{ \varpi } = \dot {\overline { \sigma } } _i ^{ \varpi } + h _i ^{ \varpi, 1 } \vartheta ( \tilde {\overline { \sigma } } _i ^{ \varpi }, \alpha _1 ^{ \varpi }, \mu _d ^{ \varpi } ) + h _i ^{ \varpi, 2 } \vartheta ( \tilde {\overline { \sigma } } _i ^{ \varpi }, \alpha _2 ^{ \varpi }, \mu _d ^{ \varpi } )
\end{aligned}
$$<br/>

where the explicit expression of $\dot {\overline { \sigma } } _i ^{ \varpi }$ is hard to obtain because the unknown disturbance exists in $\dot { \overline { \sigma } } _i ^{ \varpi } = ( \Lambda _i ) ^ { -1 } ( d _i ^{ \varpi } - h _i ^{ \varpi, 3 } \overline { \sigma } _i ^{ \varpi } )$ . Therefore, $\dot {\overline { \sigma } } _i ^{ \varpi }$ is derived by applying a differentiator on $\overline { \sigma } _i ^{ \varpi }$ . <br/>

Since the **disturbance observation error** is depicted as

$$
\begin{aligned}
\tilde d _i ^{ \varpi } = h _i ^{ \varpi, 3 } \tilde {\overline { \sigma } } _i ^{ \varpi } = d _i ^{ \varpi } - \hat d _i ^{ \varpi }
\end{aligned}
$$<br/>

and thus yield <br/>

$$
\begin{aligned}
\hat d _i ^{ \varpi } = d _i ^{ \varpi } - h _i ^{ \varpi, 3 } \tilde { \overline { \sigma } } _i ^{ \varpi }
\end{aligned}
\quad\quad(2.5)
$$<br/>

Further, from $\dot { \overline { \sigma } } _i ^{ \varpi } = ( \Lambda _i ) ^ { -1 } ( d _i ^{ \varpi } - h _i ^{ \varpi, 3 } \overline { \sigma } _i ^{ \varpi } )$ we can deduce that <br/>

$$
\begin{aligned}
d _i ^{ \varpi } = \Lambda _i \dot { \overline { \sigma } } _i ^{ \varpi } + h _i ^{ \varpi, 3 } \overline { \sigma } _i ^{ \varpi } - h _i ^{ \varpi, 3 } \tilde { \overline { \sigma } } _i ^{ \varpi } = \Lambda _i \dot { \overline { \sigma } } _i ^{ \varpi } + h _i ^{ \varpi, 3 } \hat { \overline { \sigma } } _i ^{ \varpi }
\end{aligned}
\quad\quad(2.6)
$$<br/>

Substitute **Eq.(2.6)** into **Eq.(2.5)** and yield <br/>

$$
\begin{aligned}
\hat d _i ^{ \varpi } = \Lambda _i \dot { \overline { \sigma } } _i ^{ \varpi } + h _i ^{ \varpi, 3 } \overline { \sigma } _i ^{ \varpi }
\end{aligned}
$$<br/>

Take the derivative of Lyapunov function $V _i ^{ d, \varpi }$ and substitute **Eq.(2.4)** into $\dot V _i ^{ d, \varpi }$. Utilize the inequalities in **Eq.(2.2)** and further yield <br/>

$$
\begin{aligned}
\dot V _i ^{ d, \varpi } & = ( \tilde {\overline { \sigma } } _i ^{ \varpi } )^T \dot { \tilde {\overline { \sigma } } } _i ^{ \varpi } \\
& = - h _i ^{ \varpi, 1 } ( \tilde {\overline { \sigma } } _i ^{ \varpi } )^T \vartheta ( \tilde {\overline { \sigma } } _i ^{ \varpi }, \alpha _1 ^{ \varpi }, \mu _d ^{ \varpi } ) - h _i ^{ \varpi, 2 } ( \tilde {\overline { \sigma } } _i ^{ \varpi } )^T \vartheta ( \tilde {\overline { \sigma } } _i ^{ \varpi }, \alpha _2 ^{ \varpi }, \mu _d ^{ \varpi } ) \\
& \le - 2 ^{ \frac { \alpha _1 ^{ \varpi } + 1 } { 2 } } 3 ^{ \frac { 1 - \alpha _1 ^{ \varpi } } { 2 } } K _{ \alpha } ^{ d, \varpi } h _i ^{ \varpi, 1 } { ( V _i ^{ d, \varpi } ) } ^{ \frac { \alpha _1 ^{ \varpi } + 1 } { 2 } } - 2 ^{ \frac { \alpha _2 ^{ \varpi } + 1 } { 2 } } K _{ \alpha } ^{ d, \varpi } h _i ^{ \varpi, 2 } { ( V _i ^{ d, \varpi } ) } ^{ \frac { \alpha _2 ^{ \varpi } + 1 } { 2 } } \\
& \le - a _1 { ( V _i ^{ d, \varpi } ) } ^{ b _1 } - a _2 { ( V _i ^{ d, \varpi } ) } ^{ b _2 }
\end{aligned}
\quad\quad(2.7)
$$<br/>

According to **Lemma 1**, **Eq.(2.7)** and **Eq.(2.3)**, $\tilde {\overline { \sigma } } _i ^{ \varpi } = 0$ can be achieved within fixed time only if **Eq.(2.7)** holds and the parameter settings in **Eq.(2.3)** satisfy $a _1 > 0$, $a _2 > 0$, $b _1 > 1$, $0 < b _2 < 1$. The parameters should be set as $h _i ^{ \varpi, 1 } > 0$, $h _i ^{ \varpi, 2 } > 0$, $h _i ^{ \varpi, 3 } > 0$, $\mu _d ^{ \varpi } > 0$, $\alpha _1 ^{ \varpi } > 1$, $0 < \alpha _2 ^{ \varpi } < 1$. <br/>

## 2.2 Observer implementation procedure <br/>

Update virtual angular velocity tracking vector<br/>

$$
\begin{aligned}
\dot \sigma _i ^\varpi = ( \Lambda _i ) ^{-1} ( - ( \varpi _i ) _\times \Lambda _i \varpi _i + \tau _i + h _i ^{\varpi,3} \overline \sigma _i ^\varpi )
\end{aligned}
\quad\quad(2.8)$$<br/>

Virtual angular velocity tracking error

$$
\begin{aligned}
\overline \sigma _i ^\varpi = \varpi _i - \sigma _i ^\varpi
\end{aligned}
\quad\quad(2.9)$$<br/>

In order to achieve $\hat \sigma ^i _2 \to \ddot {\overline \sigma} _i ^\varpi$, $\hat \sigma ^i _1 \to \dot {\overline \sigma} _i ^\varpi$, $\hat \sigma ^i _0 \to {\overline \sigma} _i ^\varpi$, a high-order nonlinear differentiator is given as

$$
\begin{aligned}
\begin{cases}
&\tilde \sigma ^i _0 = \overline \sigma _i ^\varpi - \hat \sigma ^i _0 \\
&\dot {\hat \sigma} ^i _2 = - c _1 ^i \mathrm {sgn} ( \tilde \sigma ^i _0 ) - c _1 ^i ( 1 - \hbar ) \mathrm {si} \mathrm g ^{1+\varsigma} ( \tilde \sigma ^i _0 ) \\
&\dot {\hat \sigma} ^i _1 = \hat \sigma _2 ^i - c _2 ^i \hbar \mathrm {si} \mathrm g ^{\frac 1 3} ( \tilde \sigma ^i _0 ) - c _2 ^i ( 1 - \hbar ) \mathrm {si} \mathrm g ^{1+ {\frac 2 3} \varsigma} ( \tilde \sigma ^i _0 ) \\
&\dot {\hat \sigma} ^i _0 = \hat \sigma ^i _1 - c _3 ^i \hbar \mathrm {si} \mathrm g ^{\frac 2 3} ( \tilde \sigma ^i _0 ) - c _3 ^i ( 1 - \hbar ) \mathrm {si} \mathrm g ^{1+ {\frac 1 3} \varsigma} ( \tilde \sigma ^i _0 )
\end{cases}
\end{aligned}
\quad\quad(2.10)$$<br/>


and then $\dot {\overline \sigma} _i ^\varpi$ in Eq.(2.8) approximately equals to the output value $\hat \sigma ^i _1$<br/>


an adaptive updating law<br/>

$$
\begin{aligned}
\dot {\hat {\overline \sigma}} _i ^\varpi = \dot {\overline \sigma} _i ^\varpi + h _i ^{\varpi,1} \vartheta ( \tilde {\overline \sigma} _i ^\varpi, \alpha _1 ^\varpi, \mu _d ^\varpi ) + h _i ^{\varpi,2} \vartheta ( \tilde {\overline \sigma} _i ^\varpi, \alpha _2 ^\varpi, \mu _d ^\varpi )
\end{aligned}
\quad\quad(2.11)$$<br/>

rotational disturbance observer<br/>

$$
\begin{aligned}
\hat d _i ^\varpi = \Lambda _i \dot {\overline \sigma} _i ^\varpi + h _i ^{\varpi,3} \hat {\overline \sigma} _i ^\varpi
\end{aligned}
\quad\quad(2.12)$$<br/>

## 2.3 Parameter settings and validation <br/>

According to **Lemma 1**, **Eq.(2.7)** and **Eq.(2.3)**, parameter settings should guarantee $h _i ^{ \varpi, 1 } > 0$, $h _i ^{ \varpi, 2 } > 0$, $h _i ^{ \varpi, 3 } > 0$, $\mu _d ^{ \varpi } > 0$, $\alpha _1 ^{ \varpi } > 1$, $0 < \alpha _2 ^{ \varpi } < 1$. <br/>



## 2.4 Validation for Comparison <br/>




# 3. Nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC)<br/>

According to the rotational dynamics in Eq.(1.6), define an exponential coordination $\psi _i ^e = [ \Psi ( R ( Q _i ^e) ) ] _{\vee}$ which is derived through inverse logarithm mapping. We establish the rotational system error dynamics as

$$
\begin{aligned}
\begin{cases}
&\dot \psi _i ^e = \varpi _i ^e \\
&\dot \varpi _i ^e = \Lambda _i ^{-1} ( -( \varpi _i ) _\times \Lambda _i \varpi _i + \tau _i ) + ( \varpi _i ^e ) _\times ( R ( Q _i ^e ) ) ^T \varpi _i ^e - ( R ( Q _i ^e ) ) ^T \dot \varpi _i ^c
\end{cases}
\end{aligned}
\quad\quad(3.1)$$<br/>

## 3.1 Theories and design principles <br/>

In order to avoid the ambiguity of the logarithmic mapping on boundary of singular spherical neighborhood $\partial \Omega _{\pi} = \lbrace \psi _i ^e \in \mathbb{R} ^{3} \mid \lVert \psi _i ^e \rVert = \pi \rbrace$



## 3.2 Controller implementation procedure <br/>

For any $i \in {x,y,z}$, define auxiliary sliding mode surface

$$
\begin{aligned}
\overline S _{i,k} = \varpi ^e _{i,k} + c _i ^S {\frac {3 ^{p+1} \pi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^{p+1}} {2 \tanh ( \gamma {\frac {\psi ^e _{i,k}} \pi} / 2 )} }
\end{aligned}
\quad\quad(14)$$<br/>

For any $i \in {x,y,z}$, define piecewise auxiliary rotational error

$$
\begin{aligned}
\Phi _k ( \psi ^e _{i,k}) = 
\begin{cases}
&\frac {3 ^{p+1} \pi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^{p+1}} {2 \tanh ( \gamma {\frac {\psi ^e _{i,k}} \pi} / 2 )} \\
& \quad\quad \text{  if } \overline S _{i,k} = 0, \text{  or  } \overline S _{i,k} \ne 0,\text{  } \lvert \psi ^e _{i,k} \rvert > \overline \varepsilon ^{S} _{i,k} \\
&\psi ^e _{i,k} ( \beta ^1 _\Phi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^2 + \beta ^2 _\Phi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^{2p} ) \\
& \quad\quad \text{  if } \overline S _{i,k} \ne 0,\text{ } \lvert \psi ^e _{i,k} \rvert \le \overline \varepsilon ^{S} _{i,k}
\end{cases}
\end{aligned}
\quad\quad(15)$$<br/>

Sliding mode surface

$$
\begin{aligned}
S _{i,k} = \varpi ^e _{i,k} + c _i ^S \Phi _k ( \psi ^e _{i,k})
\end{aligned}
\quad\quad(16)$$<br/>

The derivative of piecewise auxiliary rotational error

$$
\begin{aligned}
\overline \Phi _k ( \psi ^e _{i,k}) = 
\begin{cases}
&\frac { 3 ^{p+1} (p+1) \frac {\psi ^e _{i,k}} \pi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^{p-1} \varpi ^e _{i,k} } {2 \tanh ( \gamma {\frac {\psi ^e _{i,k}} \pi} / 2 )} \\
&+ \frac {3 ^{p+1}} 4 \gamma \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^{p+1} (\frac { \mathrm{sech} ( \gamma \psi ^e _{i,k} / 2 \pi )} {\tanh ( \gamma \psi ^e _{i,k} / 2 \pi )} ) ^2 \varpi ^e _{i,k}, \\
& \quad\quad \text{             if } \overline S _{i,k} = 0, \text{  or  } \overline S _{i,k} \ne 0,\text{  } \lvert \psi ^e _{i,k} \rvert > \overline \varepsilon ^{S} _{i,k} \\
&( 2 \beta ^1 _\Phi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^2 + ( 2p + 1 ) \beta ^2 _\Phi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^{2p} ) \varpi ^e _{i,k}, \\
& \quad\quad \text{             if } \overline S _{i,k} \ne 0,\text{  } \lvert \psi ^e _{i,k} \rvert \le \overline \varepsilon ^{S} _{i,k} \\
\end{cases}
\end{aligned}
\quad\quad(17)$$<br/>


The rotation compensation term

$$
\begin{aligned}
F _i ^S = &- ( \varpi _i ) _\times \Lambda _i \varpi _i + \Lambda _i ( \varpi _i ^e ) _\times ( R ( Q _i ^e ) ) ^T \varpi _i ^c \\
&- \Lambda _i ( R ( Q _i ^e ) ) ^T \dot \varpi _i ^c + c _i ^S \Lambda _i \overline \Phi _k ( \psi ^e _{i,k})
\end{aligned}
\quad\quad(18)$$<br/>

The applied torque control input

$$
\begin{aligned}
\tau _i = &- c _i ^{\varpi,1} \vartheta ( S_i, \beta ^1 _\varpi, \mu _c ^\Theta ) - c _i ^{\varpi,2} \vartheta ( S_i, \beta ^2 _\varpi, \mu _c ^\Theta ) \\
&- c _i ^{\varpi,3} S_i - \hat d _i ^\varpi - \frac 1 2 ( \overline \eta _i ^S ) ^2 \lVert F _i ^S \rVert ^2 S_i
\end{aligned}
\quad\quad(17)$$<br/>

where $\hat d _i ^\varpi$ is obtained by FxTDO in rotational system<br/>

Update rotational dynamics, including angular velocity, rotational matrix and their tracking errors

$$
\begin{aligned}
\begin{cases}
&\dot \varpi _i = ( \Lambda _i ) ^{-1} ( - ( \varpi _i ) _\times \Lambda _i \varpi _i + \tau _i + d _i ^\varpi ) \\
&\dot R ( Q _i ) = R ( Q _i ) ( \varpi _i ) _\times \\
&\varpi _i ^e = \varpi _i - ( R ( Q _i ^e ) ) ^T \varpi _i ^c \\
&\dot R ( Q _i ^e ) = R ( Q _i ^e ) ( \varpi _i ^e ) _\times \\
&\dot \psi ^e _i = \varpi _i ^e
\end{cases}
\end{aligned}
\quad\quad(18)$$<br/>

## 3.3 Parameter settings and validation <br/>




## 3.4 Validation for comparison <br/>





# 4. Practical fixed-time distributed state observer (PFxTDSO) <br/>


## 4.1 Theories and design principles <br/>



## 4.2 Observer implementation procedure <br/>

Observation error for follower UAV node $i$<br/>

$$
\begin{aligned}
e _{i,v} ^d = \hat v _i ^d - v_0 - \dot \delta _i
\end{aligned}
\quad\quad(1)$$<br/>

$$
\begin{aligned}
e _{i,p} ^d = \hat p _i ^d - p_0 - \delta _i
\end{aligned}
\quad\quad(2)$$<br/>

Lumped formation observation error for follower UAV node $i$<br/>

$$
\begin{aligned}
\tilde e _{i,v} ^d &= b _{i0} e ^d _{i,v} + \sum ^N _{j=1} w _{ij} ( e ^d _{i,v} - e ^d _{j,v} ) \\
&= b _{i0} ( \hat v _i ^d - v_0 - \dot \delta _i ) + \sum ^N _{j=1} w _{ij} ( ( \hat v _i ^d - \dot \delta _i ) - ( \hat v _j ^d - \dot \delta _j ) )
\end{aligned}
\quad\quad(3)$$<br/>

$$
\begin{aligned}
\tilde e _{i,p} ^d &= b _{i0} e ^d _{i,p} + \sum ^N _{j=1} w _{ij} ( e ^d _{i,p} - e ^d _{j,p} ) \\
&= b _{i0} ( \hat p _i ^d - p_0 - \delta _i ) + \sum ^N _{j=1} w _{ij} ( ( \hat p _i ^d - \delta _i ) - ( \hat p _j ^d - \delta _j ) )
\end{aligned}
\quad\quad(4)$$<br/>

Update desired state observation for follower UAV node $i$<br/>

$$
\begin{aligned}
\dot {\hat v} _i ^d = - \ell _1 ^v \vartheta ( \tilde e ^d _{i,v}, \gamma _1, \mu _o ^p ) - \ell _2 ^v \vartheta ( \tilde e ^d _{i,v}, \gamma _2, \mu _o ^p ) )
\end{aligned}
\quad\quad(5)$$<br/>

$$
\begin{aligned}
\dot {\hat p} _i ^d = \hat v _i ^d - \ell _1 ^p \vartheta ( \tilde e ^d _{i,p}, \gamma _1, \mu _o ^p ) - \ell _2 ^p \vartheta ( \tilde e ^d _{i,p}, \gamma _2, \mu _o ^p ) )
\end{aligned}
\quad\quad(6)$$<br/>


## 4.3 Parameter settings and validation <br/>




## 4.4 Validation for comparison <br/>




# 5. Fixed-time translational disturbance observer (FxTDO) <br/>

## 5.1 Theories and design principles

The virtual tracking 

$$
\begin{aligned}
\dot \sigma _i ^v = - g \overline e _3 + \frac { T _i } { m _i } R ( Q _i ) \overline e _3 + c _i ^{ v, 3 } \overline sigma _i ^v
\end{aligned}
\quad\quad(8)
$$<br/>

## 5.2 Observer implementation framework

Update virtual linear velocity tracking vector<br/>

$$
\begin{aligned}
\dot \sigma _i ^v = -g \overline e _3 + T_i R(Q_i) \overline e _3/ m _i + c _i ^{v,3} \overline \sigma _i ^v
\end{aligned}
\quad\quad(19)$$<br/>

Virtual linear velocity tracking error

$$
\begin{aligned}
\overline \sigma _i ^v = v _i - \sigma _i ^v
\end{aligned}
\quad\quad(20)$$<br/>

In order to achieve $\hat \sigma ^i _2 \to \ddot {\overline \sigma} _i ^v$, $\hat \sigma ^i _1 \to \dot {\overline \sigma} _i ^v$, $\hat \sigma ^i _0 \to {\overline \sigma} _i ^v$, a high-order nonlinear differentiator is given as

$$
\begin{aligned}
\begin{cases}
&\tilde \sigma ^i _0 = \overline \sigma _i ^v - \hat \sigma ^i _0 \\
&\dot {\hat \sigma} ^i _2 = - c _1 ^i \mathrm {sgn} ( \tilde \sigma ^i _0 ) - c _1 ^i ( 1 - \hbar ) \mathrm {si} \mathrm g ^{1+\varsigma} ( \tilde \sigma ^i _0 ) \\
&\dot {\hat \sigma} ^i _1 = \hat \sigma _2 ^i - c _2 ^i \hbar \mathrm {si} \mathrm g ^{\frac 1 3} ( \tilde \sigma ^i _0 ) - c _2 ^i ( 1 - \hbar ) \mathrm {si} \mathrm g ^{1+ {\frac 2 3} \varsigma} ( \tilde \sigma ^i _0 ) \\
&\dot {\hat \sigma} ^i _0 = \hat \sigma ^i _1 - c _3 ^i \hbar \mathrm {si} \mathrm g ^{\frac 2 3} ( \tilde \sigma ^i _0 ) - c _3 ^i ( 1 - \hbar ) \mathrm {si} \mathrm g ^{1+ {\frac 1 3} \varsigma} ( \tilde \sigma ^i _0 )
\end{cases}
\end{aligned}
\quad\quad(21)$$<br/>


and then $\dot {\overline \sigma} _i ^v$ in Eq.(8) approximately equals to the output value $\hat \sigma ^i _1$<br/>


an adaptive updating law<br/>

$$
\begin{aligned}
\dot {\hat {\overline \sigma}} _i ^v = \dot {\overline \sigma} _i ^v + c _i ^{v,1} \vartheta ( \tilde {\overline \sigma} _i ^v, \alpha _1 ^v, \mu _d ^v ) + c _i ^{v,2} \vartheta ( \tilde {\overline \sigma} _i ^v, \alpha _2 ^v, \mu _d ^v )
\end{aligned}
\quad\quad(22)$$<br/>

translational disturbance observer<br/>

$$
\begin{aligned}
\hat d _i ^v = \dot {\overline \sigma} _i ^v + c _i ^{v,3} \hat {\overline \sigma} _i ^v
\end{aligned}
\quad\quad(23)$$<br/>


# 6. Practical fixed-time decentralized formation controller (PFxTDFC) <br/>

According to the translational dynamics in Eq.(7), <br/>


## 6.1 PFxTDFC for translational control simulation <br/>
Position and linear velocity tracking error

$$
\begin{aligned}
e _i ^v = v _i - \hat v _i ^d
\end{aligned}
\quad\quad(24)$$<br/>

$$
\begin{aligned}
e _i ^p = p _i - \hat p _i ^d
\end{aligned}
\quad\quad(25)$$<br/>

virtual velocity error tracking vector

$$
\begin{aligned}
\chi _i = - \kappa _1 ^\chi \vartheta ( e _i ^p, \beta _1, \mu _c ^p ) - \kappa _2 ^\chi \vartheta ( e _i ^p, \beta _2, \mu _c ^p )
\end{aligned}
\quad\quad(26)$$<br/>

virtual velocity error tracking error

$$
\begin{aligned}
\phi _i = e _i ^v - \chi _i
\end{aligned}
\quad\quad(27)$$<br/>

employ Eq.(23) to design the translational control input

$$
\begin{aligned}
u _i = g \overline e _3 + \dot \chi _i - \kappa _1 ^u \vartheta ( \phi _i, \beta _1, \mu _c ^p ) - \kappa _2 ^u \vartheta ( \phi _i, \beta _2, \mu _c ^p ) - \hat d _i ^v
\end{aligned}
\quad\quad(28)$$<br/>

update the translational dynamics

$$
\begin{aligned}
\begin{cases}
&\dot v _i = -g \overline e _3 + u _i + T _i R( Q _i ^c ) ( R( Q _i ^e ) - I _3 ) \overline e _3 / m _i + d _i ^v \\
&\dot p _i = v _i
\end{cases}
\end{aligned}
\quad\quad(29)$$<br/>


## 6.2 PFxTDFC for experimental validation <br/>

The translational dynamics Eq.(29) is simplified as a second-order agent with external disturbacne <br/>

$$
\begin{aligned}
\begin{cases}
&\dot v _i = U _i + d _i ^v \\
&\dot p _i = v _i
\end{cases}
\end{aligned}
\quad\quad(30)$$<br/>

employ Eq.(23) and design $U _i$ to reformulate the PFxTDFC Eq.(28) as <br/>

$$
\begin{aligned}
U _i = \dot \chi _i - \kappa _1 ^u \vartheta ( \phi _i, \beta _1, \mu _c ^p ) - \kappa _2 ^u \vartheta ( \phi _i, \beta _2, \mu _c ^p ) - \hat d _i ^v
\end{aligned}
\quad\quad(31)$$<br/>


# Appendix : Symbol definition list <br/>

- $g$ : gravitational acceleration
- $\overline e _3$ : unit vector $[0,0,1]^{T}$
- $m_{i}$ : mass
- $\Lambda_{i}$ : inertia matrix 
- $p_{0}$, $p_{i}$ : position vector of leader UAV, follower UAV node $i$
- $v_{0}$, $v_{i}$ : linear velocity vector of leader UAV, follower UAV node $i$
- $u_{0}$, $u_{i}$ : translational control input of leader UAV, follower UAV node $i$
- $\hat p_i ^d, \hat v_i ^d$ : desired position, linear velocity observation for follower UAV node $i$
- $e_{i,p}^{d}$, $e_{i,v}^{d}$ : observation error of desired position, linear velocity for follower UAV node $i$
- $\tilde e _{i,p} ^d$, $\tilde e _{i,v} ^d$ : lumped formation observation error of desired position, linear velocity for follower UAV node $i$
- $e_{i}^{p}$, $e_{i}^{v}$ : position, linear velocity tracking error of follower UAV node $i$
- $\chi_i$, $\phi_i$ : virtual linear velocity tracking vector, tracking error
- $d _i ^v$, $\hat d _i ^v$ : translational disturbance, disturbance observation
- $\sigma _i ^v$, $\overline \sigma _i ^v$, $\hat {\overline \sigma} _i ^v$ : virtual linear velocity tracking vector, tracking error, tracking error observation
- $Q_{i} = [\rho_{i}, q_{i}^{T}]^{T} = [\rho_{i}, q_{i}^{1}, q_{i}^{2}, q_{i}^{3}]^T$ : quaternion
- $Q_{i}^{c}$, $Q_{i}^{e}$ : quaternion command, error
- $R(Q_{i})$, $R(Q_{i}^{c})$, $R(Q_{i}^{e})$ : rotation matrix, command, error
- $\varpi_{i}$, $\varpi_{i}^{c}$, $\varpi_{i}^{e}$ : angular velocity, command, error
- $\psi_i^e = [\Psi(R(Q_i^e))]_\vee$ : rotational error in Lie Algebra
- $\overline S_i$ = $[ \overline S_{i,x}, \overline S_{i,y}, \overline S_{i,z} ]^T$ : auxiliary sliding mode surface
- $\Phi(\psi_i^e) = [ \Phi_x(\psi_{i,x} ^e), \Phi_y(\psi_{i,y} ^e), \Phi_z(\psi_{i,z} ^e)]^T$ : piecewise auxiliary rotational error
- $\overline \Phi (\psi_i^e) = [ \overline \Phi _x(\psi ^e _{i,x}), \overline \Phi _y(\psi ^e _{i,y}), \overline \Phi _z(\psi ^e _{i,z})]^T$ : first derivative of piecewise auxiliary rotational error
- $F_i^S$ : rotation compensation term
- $\tau_i$ : applied torque rotational control input


