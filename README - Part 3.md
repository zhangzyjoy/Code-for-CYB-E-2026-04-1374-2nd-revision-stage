# 4. Practical fixed-time distributed state observer (PFxTDSO) <br/>


## 4.1 Theories and design principles <br/>



## 4.2 Observer implementation procedure <br/>

Observation error for follower UAV node $i$<br/>

$$
\begin{aligned}
e _{i,v} ^d = \hat v _i ^d - v_0 - \dot \delta _i
\end{aligned}
\quad\quad(1)
$$<br/>

$$
\begin{aligned}
e _{i,p} ^d = \hat p _i ^d - p_0 - \delta _i
\end{aligned}
\quad\quad(2)
$$<br/>

Lumped formation observation error for follower UAV node $i$<br/>

$$
\begin{aligned}
\tilde e _{i,v} ^d &= b _{i0} e ^d _{i,v} + \sum ^N _{j=1} w _{ij} ( e ^d _{i,v} - e ^d _{j,v} ) \\
&= b _{i0} ( \hat v _i ^d - v_0 - \dot \delta _i ) + \sum ^N _{j=1} w _{ij} ( ( \hat v _i ^d - \dot \delta _i ) - ( \hat v _j ^d - \dot \delta _j ) )
\end{aligned}
\quad\quad(3)
$$<br/>

$$
\begin{aligned}
\tilde e _{i,p} ^d &= b _{i0} e ^d _{i,p} + \sum ^N _{j=1} w _{ij} ( e ^d _{i,p} - e ^d _{j,p} ) \\
&= b _{i0} ( \hat p _i ^d - p_0 - \delta _i ) + \sum ^N _{j=1} w _{ij} ( ( \hat p _i ^d - \delta _i ) - ( \hat p _j ^d - \delta _j ) )
\end{aligned}
\quad\quad(4)
$$<br/>

Update desired state observation for follower UAV node $i$<br/>

$$
\begin{aligned}
\dot {\hat v} _i ^d = - \ell _1 ^v \vartheta ( \tilde e ^d _{i,v}, \gamma _1, \mu _o ^p ) - \ell _2 ^v \vartheta ( \tilde e ^d _{i,v}, \gamma _2, \mu _o ^p )
\end{aligned}
\quad\quad(5)
$$<br/>

$$
\begin{aligned}
\dot {\hat p} _i ^d = \hat v _i ^d - \ell _1 ^p \vartheta ( \tilde e ^d _{i,p}, \gamma _1, \mu _o ^p ) - \ell _2 ^p \vartheta ( \tilde e ^d _{i,p}, \gamma _2, \mu _o ^p )
\end{aligned}
\quad\quad(6)
$$<br/>


## 4.3 Parameter settings and validation <br/>




## 4.4 Validation for comparison <br/>


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


