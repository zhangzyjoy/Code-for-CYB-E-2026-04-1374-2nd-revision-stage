# 5. Fixed-time translational disturbance observer (FxTDO) <br/>

## 5.1 Theories and design principles




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
\quad\quad(22)
$$<br/>

translational disturbance observer<br/>

$$
\begin{aligned}
\hat d _i ^v = \dot {\overline \sigma} _i ^v + c _i ^{v,3} \hat {\overline \sigma} _i ^v
\end{aligned}
\quad\quad(23)
$$<br/>


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

