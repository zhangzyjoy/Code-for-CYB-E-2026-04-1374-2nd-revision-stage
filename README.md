We upload **Matlab/Simulink** source code for **simulation** and **Python/ROS** code package for **experimental validation** in this webpage link to enhance the instructions on the relationship between theoretical contribution and further verification according to the reviewers' suggestions for our revised version with a submission ID **CYB-E-2026-04-1374** in **IEEE Transactions on Cybernetics**.<br/><br/> 

This link demonstrates the proposed control approach for the formation control issue of uncrewed aerial vehicle (UAV) teams. <br/><br/>

To address the reviewer's comments, the logical connection between theory and validation has been strengthened. The narrative follows a systematic progression. <br/> 
First, the theoretical basis for controller design is introduced using fundamental lemmas. <br/> 
Next, the derived parameter bounds for system stability are presented. <br/> 
Then, the exact parameter configurations for simulation or experimental validation are specified. <br/> 
Finally, the validation results are presented and discussed. <br/> 



**Note : Source code and instructions for the first revision under Revised & Resubmit decision is deposited at https://github.com/zhangzyjoy/Code-for-manuscript-revision-stage.git.** <br/>

# Significant Theories and Lemmas <br/>



# Control Scheme Instruction <br/>

Each quadcopter UAV node is decoupled into rotational and translational subsystems, respectively. <br/><br/> 

The proposed control scheme includes five main modules: <br/>
1. practical fixed-time distributed state observer (PFxTDSO) <br/>
2. fixed-time disturbance observer in rotational subsystem (FxTDO) <br/>
3. nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC) <br/>
4. fixed-time disturbance observer in translational subsystem (FxTDO) <br/>
5. practical fixed-time decentralized formation controller (PFxTDFC) <br/><br/>

Details are illustrated as follows. <br/>
<1> For the translational subsystem, a distributed practical fixed-time formation controller (PFxTDFC) is proposed to achieve formation consensus. <br/> 
<2> Practical fixed-time distributed state observers (PFxTDSO) are developed to estimate the desired velocity and position for each follower UAV and to maintain fully decentralized realization. <br/> 
<3> Leveraging the logarithmic mapping of rotational errors in Lie algebra space, a nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC) is developed to attain practical fixed-time singularity-free anti-disturbance attitude tracking in rotational subsystem. <br/> 
<4> Fixed-time disturbance observers (FxTDO) is promoted to compensate external disturbances in both rotational and translational subsystems. <br/><br/> 

## Nonlinear smooth sigmoid function <br/>

A newly introduced nonlinear smooth sigmoid vector is defined as <br/>

$$
\begin{aligned}
\vartheta ( \overline x, \alpha, \gamma ) = [ \vartheta _k ( \overline x _1, \alpha, \gamma ), ..., \vartheta _k ( \overline x _n, \alpha, \gamma ) ] ^T
\end{aligned}
$$<br/>

with each entry denoted as <br/>

$$
\begin{aligned}
\vartheta _k ( \overline x _k, \alpha, \gamma ) = | x _k | ^ \alpha \lambda _k ( x _k, \gamma )
\end{aligned}
$$<br/>

where the sigmoid function is <br/>

$$
\begin{aligned}
\lambda _k ( x _k, \gamma ) = -1 + \frac 1 {1 + \exp ( - \gamma x _k )}
\end{aligned}
$$<br/>

## Fixed-time rotational disturbance observer (FxTDO) <br/>

Update virtual angular velocity tracking vector<br/>

$$
\begin{aligned}
\dot \sigma _i ^\varpi = ( \Lambda _i ) ^{-1} ( - ( \varpi _i ) _\times \Lambda _i \varpi _i + \tau _i + h _i ^{\varpi,3} \overline \sigma _i ^\varpi )
\end{aligned}
\quad\quad(7)$$<br/>

Virtual angular velocity tracking error

$$
\begin{aligned}
\overline \sigma _i ^\varpi = \varpi _i - \sigma _i ^\varpi
\end{aligned}
\quad\quad(8)$$<br/>

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
\quad\quad(9)$$<br/>


and then $\dot {\overline \sigma} _i ^\varpi$ in Eq.(8) approximately equals to the output value $\hat \sigma ^i _1$<br/>


an adaptive updating law<br/>

$$
\begin{aligned}
\dot {\hat {\overline \sigma}} _i ^\varpi = \dot {\overline \sigma} _i ^\varpi + h _i ^{\varpi,1} \vartheta ( \tilde {\overline \sigma} _i ^\varpi, \alpha _1 ^\varpi, \mu _d ^\varpi ) + h _i ^{\varpi,2} \vartheta ( \tilde {\overline \sigma} _i ^\varpi, \alpha _2 ^\varpi, \mu _d ^\varpi )
\end{aligned}
\quad\quad(10)$$<br/>

rotational disturbance observer<br/>

$$
\begin{aligned}
\hat d _i ^\varpi = \Lambda _i \dot {\overline \sigma} _i ^\varpi + h _i ^{\varpi,3} \hat {\overline \sigma} _i ^\varpi
\end{aligned}
\quad\quad(11)$$<br/>


## Nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC)<br/>

For any $i \in {x,y,z}$, define auxiliary sliding mode surface

$$
\begin{aligned}
\overline S _{i,k} = \varpi ^e _{i,k} + c _i ^S {\frac {3 ^{p+1} \pi | \frac {\psi ^e _{i,k}} \pi | ^{p+1}} {2 \tanh ( \gamma {\frac {\psi ^e _{i,k}} \pi} / 2 )} }
\end{aligned}
\quad\quad(12)$$<br/>

For any $i \in {x,y,z}$, define piecewise auxiliary rotational error

$$
\begin{aligned}
\Phi _k ( \psi ^e _{i,k}) = 
\begin{cases}
&\frac {3 ^{p+1} \pi | \frac {\psi ^e _{i,k}} \pi | ^{p+1}} {2 \tanh ( \gamma {\frac {\psi ^e _{i,k}} \pi} / 2 )} \\
& \quad\quad \text{  if } \overline S _{i,k} = 0, \text{  or  } \overline S _{i,k} \ne 0,\text{  }|\psi ^e _{i,k}| > \overline \varepsilon ^{S} _{i,k} \\
&\psi ^e _{i,k} ( \beta ^1 _\Phi |\frac {\psi ^e _{i,k}} \pi|^2 + \beta ^2 _\Phi |\frac {\psi ^e _{i,k}} \pi| ^{2p} ) \\
& \quad\quad \text{  if } \overline S _{i,k} \ne 0,\text{ }|\psi ^e _{i,k}| \le \overline \varepsilon ^{S} _{i,k}
\end{cases}
\end{aligned}
\quad\quad(13)$$<br/>

Sliding mode surface

$$
\begin{aligned}
S _{i,k} = \varpi ^e _{i,k} + c _i ^S \Phi _k ( \psi ^e _{i,k})
\end{aligned}
\quad\quad(14)$$<br/>

The derivative of piecewise auxiliary rotational error

$$
\begin{aligned}
\overline \Phi _k ( \psi ^e _{i,k}) = 
\begin{cases}
&\frac { 3 ^{p+1} (p+1) \frac {\psi ^e _{i,k}} \pi | \frac {\psi ^e _{i,k}} \pi | ^{p-1} \varpi ^e _{i,k} } {2 \tanh ( \gamma {\frac {\psi ^e _{i,k}} \pi} / 2 )} \\
&+ \frac {3 ^{p+1}} 4 \gamma |\frac {\psi ^e _{i,k}} \pi| ^{p+1} (\frac { \mathrm{sech} ( \gamma \psi ^e _{i,k} / 2 \pi )} {\tanh ( \gamma \psi ^e _{i,k} / 2 \pi )} ) ^2 \varpi ^e _{i,k}, \\
& \quad\quad \text{             if } \overline S _{i,k} = 0, \text{  or  } \overline S _{i,k} \ne 0,\text{  }|\psi ^e _{i,k}| > \overline \varepsilon ^{S} _{i,k} \\
&( 2 \beta ^1 _\Phi | \frac {\psi ^e _{i,k}} \pi | ^2 + ( 2p + 1 ) \beta ^2 _\Phi | \frac {\psi ^e _{i,k}} \pi | ^{2p} ) \varpi ^e _{i,k}, \\
& \quad\quad \text{             if } \overline S _{i,k} \ne 0,\text{  }|\psi ^e _{i,k}| \le \overline \varepsilon ^{S} _{i,k} \\
\end{cases}
\end{aligned}
\quad\quad(15)$$<br/>


The rotation compensation term

$$
\begin{aligned}
F _i ^S = &- ( \varpi _i ) _\times \Lambda _i \varpi _i + \Lambda _i ( \varpi _i ^e ) _\times ( R ( Q _i ^e ) ) ^T \varpi _i ^c \\
&- \Lambda _i ( R ( Q _i ^e ) ) ^T \dot \varpi _i ^c + c _i ^S \Lambda _i \overline \Phi _k ( \psi ^e _{i,k})
\end{aligned}
\quad\quad(16)$$<br/>

The applied torque control input

$$
\begin{aligned}
\tau _i = &- c _i ^{\varpi,1} \vartheta ( S_i, \beta ^1 _\varpi, \mu _c ^\Theta ) - c _i ^{\varpi,2} \vartheta ( S_i, \beta ^2 _\varpi, \mu _c ^\Theta ) \\
&- c _i ^{\varpi,3} S_i - \hat d _i ^\varpi - \frac 1 2 ( \overline \eta _i ^S ) ^2 \|\| F _i ^S \|\| ^2 S_i
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


## Practical fixed-time distributed state observer (PFxTDSO) <br/>

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


## Fixed-time translational disturbance observer (FxTDO) <br/>

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


## Practical fixed-time decentralized formation controller (PFxTDFC) <br/>

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


## Practical fixed-time decentralized formation controller (PFxTDFC) in experiment <br/>

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


## Symbol definition list <br/>

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


# Simulation on Proposed Controller <br/>



# Simulation for Comparison <br/>



# Experimental Validation <br/>


