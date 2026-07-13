# 3. $\text{  }$ Nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC)<br/>

**Note : Please refer to <README - Part 2 - 1.md> for the former part of Section 3.** <br/>


## 3.2 $\text{  }$ Controller implementation procedure <br/>

For any $k \in \lbrace { x,y,z } \rbrace$, define an auxiliary sliding mode surface

$$
\begin{aligned}
\overline S _{i,k} = \varpi ^e _{i,k} + c _i ^S {\frac {3 ^{p+1} \pi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^{p+1}} {2 \tanh ( \gamma {\frac {\psi ^e _{i,k}} \pi} / 2 )} }
\end{aligned}
\quad\quad(2.1)$$<br/>

For any $k \in \lbrace x,y,z \rbrace$, define piecewise auxiliary rotational error

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
\quad\quad(2.2)$$<br/>

Sliding mode surface

$$
\begin{aligned}
S _{i,k} = \varpi ^e _{i,k} + c _i ^S \Phi _k ( \psi ^e _{i,k})
\end{aligned}
\quad\quad(2.3)
$$<br/>

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
\quad\quad(2.4)
$$<br/>


The rotation compensation term

$$
\begin{aligned}
F _i ^S = &- ( \varpi _i ) _\times \Lambda _i \varpi _i + \Lambda _i ( \varpi _i ^e ) _\times ( R ( Q _i ^e ) ) ^T \varpi _i ^c \\
&- \Lambda _i ( R ( Q _i ^e ) ) ^T \dot \varpi _i ^c + c _i ^S \Lambda _i \overline \Phi _k ( \psi ^e _{i,k})
\end{aligned}
\quad\quad(2.5)$$<br/>

The applied torque control input

$$
\begin{aligned}
\tau _i = &- c _i ^{\varpi,1} \vartheta ( S_i, \beta ^1 _\varpi, \mu _c ^\Theta ) - c _i ^{\varpi,2} \vartheta ( S_i, \beta ^2 _\varpi, \mu _c ^\Theta ) \\
&- c _i ^{\varpi,3} S_i - \hat d _i ^\varpi - \frac 1 2 ( \overline \eta _i ^S ) ^2 \lVert F _i ^S \rVert ^2 S_i
\end{aligned}
\quad\quad(2.6)$$<br/>

where $\hat d _i ^\varpi$ is obtained by FxTDO in rotational system. <br/>

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
\quad\quad(2.7)$$<br/>

## 3.3 $\text{  }$ Parameter settings and validation <br/>

According to **Lemma 2** and **Eq.(1.9)**, the parameters of the rotational torque control input are chosen as $\tau _i$ as $c _i ^{ \varpi, 1 } > 0$, $c _i ^{ \varpi, 2 } > 0$, $\beta _{ \varpi } ^1 > 1$, $0 < \beta _{ \varpi } ^2 < 1$. <br/>

According to **Lemma 2**, the settling time for the practical fixed-time convergence of **PCNSMS** $S _i$ is indicated as

$$
\begin{aligned}
T _r ^{ \varpi } \le \overline { T } _r ^{ \varpi } = \frac { { ( \lambda _{max} ( \Lambda _i ) ) } ^ { \frac { \beta _{ \varpi } ^1 + 1 } { 2 } } } { 2 ^{ \frac { \beta _{ \varpi } ^1 - 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ \varpi } ^1 } { 2 } } c _i ^{ \varpi, 1 } K _i ^{ \varpi } \eta _c ^{ \varpi } ( \beta _{ \varpi } ^1 - 1 ) } + \frac { { ( \lambda _{max} ( \Lambda _i ) ) } ^ { \frac { \beta _{ \varpi } ^2 + 1 } { 2 } } } { 2 ^{ \frac { \beta _{ \varpi } ^2 - 1 } { 2 } } c _i ^{ \varpi, 2 } K _i ^{ \varpi } \eta _c ^{ \varpi } ( 1 - \beta _{ \varpi } ^2 ) }
\end{aligned}
$$<br/>

where $K _i ^{ \varpi } > 0$ and $\eta _c ^{ \varpi } > 0$ hold. <br/>




## 3.4 $\text{  }$ Validation for comparison <br/>


# Appendix 1 : $\text{  }$ Significant theories <br/>

## A.1 $\text{  }$ Fixed-time / Practical fixed-time stability lemmas <br/>

Consider a nonlinear system as <br/>

$$
\begin{aligned}
\dot x ( t ) = f ( x ( t ) ) , x ( t _0 ) = x _0
\end{aligned}
\quad\quad(A.1)
$$<br/>

where $t _0$ is the initial time and $x ( t _0 )$ is the state vector, and $f ( x ( t ) )$ is a nonlinear function of system dynamics. <br/>

**Lemma 1 (Fixed-Time Stability)** : If a nonlinear function $V ( t , x ) : \mathbb R \times \mathbb R ^n \to \mathbb R ^+ \cup \lbrace 0 \rbrace$ is defined as positive definite and radially unbounded, and $\dot V ( t , x ) \le - a _1 ( V ( t, x ) ^ { b _1 } ) - a _2 ( V ( t, x ) ^ { b _2 } )$ holds such that $a _1 >0$, $a _2 >0$, $b _1 > 1$, $0 < b _2 < 1$, respectively, then the origin of (1) is fixed-time stable and the settling time is bounded by <br/>

$$
\begin{aligned}
T _s \le \overline T _s = \frac { 1 } { a _1 ( b _1 - 1 )} + \frac { 1 } { a _2 ( 1 - b _2 ) }
\end{aligned}
$$<br/>

**Lemma 2 (Practical Fixed-Time Stability)** : If a radially unbounded positive definite function satisfies $\dot V ( t , x ) \le - a _1 ( V ( t, x ) ^ { b _1 } ) - a _2 ( V ( t, x ) ^ { b _2 } ) + c _0$ such that $a _1 >0$, $a _2 >0$, $b _1 > 1$, $0 < b _2 < 1$, and $c _0 > 0$, the origin of (1) is practical fixed-time stable. There exists a constant $0 < \eta _0 < 1$ such that the settling time fulfills

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

**Lemma 4** : Consider a sequence with positive scalars $q _1, \text{ } q _2, \text{ } ..., \text{ } q_N$ where $q _k \text{ } \ge \text{ } 0$ for any $k \in \lbrace 1,...,N \rbrace$. Given that $0 < \alpha _1 \le 1$, $\alpha _2 > 1$, it can be yielded that

$$
\begin{aligned}
\sum _{ k = 1 } ^{ N } { ( q _k ) ^{ \alpha _1 } } \ge { ( \sum _{ k = 1 } ^{ N } { q _k } ) } ^{ \alpha _1 }, \quad \quad \sum _{ k = 1 } ^{ N } { ( q _k ) ^{ \alpha _2 } } \ge N ^{ 1 - \alpha _2 } { ( \sum _{ k = 1 } ^{ N } { q _k } ) } ^{ \alpha _2 }
\end{aligned}
$$<br/>


## A.2 $\text{  }$ Design of nonlinear smooth sigmoid function <br/>

A newly introduced **nonlinear smooth sigmoid vector** is defined as <br/>

$$
\begin{aligned}
\vartheta ( \overline x, \alpha, \gamma ) = [ \vartheta _k ( \overline x _1, \alpha, \gamma ), ..., \vartheta _k ( \overline x _n, \alpha, \gamma ) ] ^T
\end{aligned}
\quad\quad(A.2)
$$<br/>

with each entry denoted as <br/>

$$
\begin{aligned}
\vartheta _k ( \overline x _k, \alpha, \gamma ) = \lvert x _k \rvert ^ \alpha \lambda _k ( x _k, \gamma )
\end{aligned}
\quad\quad(A.3)
$$<br/>

$\lambda _k ( x _k, \gamma )$ is a sigmoid-like bounded function defined as <br/>

$$
\begin{aligned}
\lambda _k ( x _k, \gamma ) = -1 + \frac 1 {1 + \exp ( - \gamma x _k )}
\end{aligned}
\quad\quad(A.4)
$$<br/>

where $\gamma > 0$ controls the growth rate around the zero crossing. <br/>

**Lemma 5** : For a vector $\overline x = [ x _1, ..., x _n ] ^T \in \mathbb R ^n$ , $\alpha > 0$, and $\gamma > 0$, define $\vartheta ( \overline x , \alpha, \gamma ) = [ \vartheta _1 ( x _1, \alpha, \gamma ), ..., \vartheta _n ( x _n, \alpha, \gamma ) ]^T$ with each entries $\vartheta _k ( x _k, \alpha, \gamma )$ as the form of Eq.(3), the inequalities are yielded as

$$
\begin{aligned}
\begin{cases}
&- ( \overline x ) ^T \vartheta ( \overline x, \alpha, \gamma ) \le - 2 ^{ \frac { \alpha + 1 } { 2 } } K _{ \alpha } ( \frac { ( \overline x ) ^T \overline x } { 2 } ) ^{ \frac { \alpha + 1 } { 2 } } ), \quad\quad ( 0 < \alpha \le 1 ) \\
&- ( \overline x ) ^T \vartheta ( \overline x, \alpha, \gamma ) \le - 2 ^{ \frac { \alpha + 1 } { 2 } } n ^{ \frac { 1 - \alpha } { 2 } } K _{ \alpha } ( \frac { ( \overline x ) ^T \overline x } { 2 } ) ^{ \frac { \alpha + 1 } { 2 } } ), \quad\quad ( \alpha > 1 )
\end{cases}
\end{aligned}
\quad\quad(A.5)
$$<br/>

where $K _{ \alpha } = min \lbrace \frac { 1 } { \overline \epsilon } , \underline \epsilon \rbrace$, $\overline \epsilon$ and $\underline \epsilon$ are positive constants. <br/>

## A.3 $\text{  }$ System dynamics of a quadrotor UAV <br/>

Each agent in the UAV team is considered as a quadrotor, which can be decoupled into rotational and translational subsystems as

$$
\begin{aligned}
\begin{cases}
&\dot R ( Q _i ) = R ( Q _i ) ( \varpi _i ) _{\times}  \\
&\dot \varpi _i = ( \Lambda _i ) ^{-1} ( -( \varpi _i ) _{\times} \Lambda _i \varpi _i + \tau _i + d _i ^{\varpi} )
\end{cases}
\end{aligned}
\quad\quad(A.6)$$<br/>

$$
\begin{aligned}
\begin{cases}
&\dot p _i = v _i  \\
&\dot v _i = - g \overline e _3 + \frac { T _i } { m _i } R ( Q _i ) \overline e _3 + d _i ^v
\end{cases}
\end{aligned}
\quad\quad(A.7)$$<br/>

Each quadcopter UAV node is decoupled into rotational and translational subsystems. <br/> <br/> 


# Appendix 2 : $\text{  }$ Symbol definition list <br/>

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



