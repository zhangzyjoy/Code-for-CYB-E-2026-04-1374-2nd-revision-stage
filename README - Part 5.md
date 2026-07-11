# 6. Practical fixed-time decentralized formation controller (PFxTDFC) <br/>



## 6.1 PFxTDFC for translational control simulation <br/>

Position and linear velocity tracking error

$$
\begin{aligned}
{ e _i ^v } = { v _i - \hat { v } _i ^d }
\end{aligned}
\quad\quad(24)
$$<br/>

$$
\begin{aligned}
{ e _i ^p } = { p _i - \hat { p } _i ^d }
\end{aligned}
\quad\quad(25)
$$<br/>

The auxiliary velocity control variable

$$
\begin{aligned}
{ \chi _i } = { - \kappa _1 ^{ \chi } { \vartheta ( e _i ^p, \beta _1, \mu _c ^p ) } - \kappa _2 ^{ \chi } { \vartheta ( e _i ^p, \beta _2, \mu _c ^p ) } }
\end{aligned}
\quad\quad(26)
$$<br/>

The virtual velocity tracking error

$$
\begin{aligned}
{ \phi _i } = { e _i ^v - \chi _i }
\end{aligned}
\quad\quad(27)
$$<br/>

Employ Eq.(23) to design the translational control input

$$
\begin{aligned}
{ u _i } = { g \overline e _3 + \dot \chi _i - \kappa _1 ^u \vartheta ( \phi _i, \beta _1, \mu _c ^p ) - \kappa _2 ^u \vartheta ( \phi _i, \beta _2, \mu _c ^p ) - \hat d _i ^v }
\end{aligned}
\quad\quad(28)
$$<br/>

Update the translational dynamics

$$
\begin{aligned}
\begin{cases}
&{ \dot v _i } = { -g \overline e _3 + u _i + T _i R( Q _i ^c ) ( R( Q _i ^e ) - I _3 ) \overline e _3 / m _i + d _i ^v } \\
&{ \dot p _i } = { v _i }
\end{cases}
\end{aligned}
\quad\quad(29)
$$<br/>


## 6.2 PFxTDFC for experimental validation <br/>

The translational dynamics Eq.(29) is simplified as a second-order agent with external disturbacne <br/>

$$
\begin{aligned}
\begin{cases}
&{ \dot v _i } = { U _i + d _i ^v } \\
&{ \dot p _i } = { v _i }
\end{cases}
\end{aligned}
\quad\quad(30)
$$<br/>

Employ Eq.(23) and design $U _i$ to reformulate the PFxTDFC Eq.(28) as <br/>

$$
\begin{aligned}
{ U _i } = { \dot \chi _i - \kappa _1 ^u \vartheta ( \phi _i, \beta _1, \mu _c ^p ) - \kappa _2 ^u \vartheta ( \phi _i, \beta _2, \mu _c ^p ) - \hat d _i ^v }
\end{aligned}
\quad\quad(31)
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

