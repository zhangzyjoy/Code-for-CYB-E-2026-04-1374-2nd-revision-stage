# 3. $\text{  }$ Practical fixed-time distributed state observer (PFxTDSO) <br/>


## 3.1 $\text{  }$ Theories and design principles <br/>

We use $\hat { p } _i ^d$ and $\hat { v } _i ^d$ to denote the distributed observation of the desired position and velocity for follower vertex, respectively. The observation objective $\hat v _i ^d \to v _{ 0 } + \dot { \delta } _{ i }$ and $\hat p _i ^d \to p _{ 0 } + { \delta } _{ i }$ are expected to achieve for each follower vertex. The preset formation configuration $\delta _i$ and $\dot { \delta } _i$ and the state feedback $p _0$ and $v _0$ of the leader vertex are transmitted to only partial followers in the swarm. <br/>

The observation errors of desired velocity and position for the $i$-th follower vertex are individually defined as

$$
\begin{aligned}
\begin{cases}
{ e _{i,v} ^d } &= { \hat { v } _i ^{ d } - v _0 - \dot { \delta } _i } \\
{ e _{i,p} ^d } &= { \hat { p } _i ^{ d } - p _0 - \delta _i }
\end{cases}
\end{aligned}
\quad\quad(1.1)
$$<br/>

Then the stacked observation vector is defined as $E _v ^d = [ { ( e _{ 1,v } ^d ) } ^T, ..., { ( e _{ n,v } ^d ) } ^T ] ^T$ and $E _p ^d = [ { ( e _{ 1,p } ^d ) } ^T, ..., { ( e _{ n,p } ^d ) } ^T ] ^T$. Introduce the Laplacian matrix $L = [ l _{ ij } ] _{ N × N }$ with each entry as $l _{ ij } = \sum _{ j = 1, j \neq i } ^{ N } { w _{ ij } }$ for $i = j$ and $l _{ ij } = - w _{ ij }$ for $i \neq j$ and a diagonal matrix $B = \text{diag} { [ b _{ 10 }, ..., b _{ n0 } ] }$. An auxiliary Laplician matrix is further defined as $\overline { L } = L + B$. Invoking the auxiliary Laplician $\overline { L }$ and considering the overall observation result, the stacked lumped observation error is indicated as <br/>

$$
\begin{aligned}
{ \tilde { E } _{ i,v } ^d } &= { { ( \overline { L } \otimes I _{ 3 } ) } E _{ i,v } ^d }, \quad \quad { \tilde { E } _{ i,p } ^d } &= { ( \overline { L } \otimes I _{ 3 } ) E _{ i,p } ^d }
\end{aligned}
\quad\quad(1.2)
$$<br/>

Invoking **Lemma 4** on the theoretical design for PFxTDSO, and given parameters that $\gamma _{ 1 } > 1$ and $0 < \gamma _{ 2 } < 1$ holds, it can be deduced that

$$
\begin{aligned}
\begin{cases}
{ - { ( \tilde { E } _v ^{ d } ) } ^{ T } \vartheta ( \tilde { E } _v ^{ d } , \gamma _{ 1 }, \mu _o ^{ p } ) } & \le { - 2 ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \gamma _{ 1 } } { 2 } } K _{ \gamma } ^{ 1 } { ( { ( \tilde { E } _v ^{ d } ) } ^{ T } \tilde { E } _v ^{ d } / 2 ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } } \\
{ - { ( \tilde { E } _v ^{ d } ) } ^{ T } \vartheta ( \tilde { E } _v ^{ d } , \gamma _{ 2 }, \mu _o ^{ p } ) } & \le { - 2 ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } K _{ \gamma } ^{ 2 } { ( { ( \tilde { E } _v ^{ d } ) } ^{ T } \tilde { E } _v ^{ d } / 2 ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } }
\end{cases}
\end{aligned}
\quad\quad(1.3)
$$<br/>

For the theoretical proof for the fixed-time stability of the distributed velocity observation, the Lyapunov candidate is chosen as $V _o ^{ v } = { ( E _v ^{ d } ) } ^{ T } ( \overline { L } ^T \otimes I _{ 3 } ) E _v ^{ d } / 2$. Substitute Eq.(1.2) into ${ ( \tilde { E } _v ^{ d } ) } ^{ T } \tilde { E } _v ^{ d } / 2$ and then obtain <br/>

$$
\begin{aligned}
{ \frac { 1 } { 2 } { ( \tilde { E } _v ^{ d } ) } ^{ T } \tilde { E } _v ^{ d } } = { \frac { 1 } { 2 } { ( E _v ^{ d } ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) ( \overline { L } \otimes I _{ 3 } ) E _v ^{ d } } \ge { \lambda _{ \min } ( \overline { L } ) ( \frac { 1 } { 2 } { ( E _v ^{ d } ) } ^{ T } ( \overline { L } \otimes I _{ 3 } ) E _v ^{ d } ) } = \lambda _{ \min } ( \overline { L } ) { V _o ^v }
\end{aligned}
\quad\quad(1.4)
$$<br/>

Substitute Eq.(1.4) into Eq.(1.3) and yield

$$
\begin{aligned}
\begin{cases}
{ - { ( \tilde { E } _v ^{ d } ) } ^{ T } \vartheta ( \tilde { E } _v ^{ d } , \gamma _{ 1 }, \mu _o ^{ p } ) } & \le { - 2 ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \gamma _{ 1 } } { 2 } } K _{ \gamma } ^{ 1 } { \lambda _{ \min } ( \overline { L } ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } { V _o ^{ v } } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } } \\
{ - { ( \tilde { E } _v ^{ d } ) } ^{ T } \vartheta ( \tilde { E } _v ^{ d } , \gamma _{ 2 }, \mu _o ^{ p } ) } & \le { - 2 ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } K _{ \gamma } ^{ 2 } { \lambda _{ \min } ( \overline { L } ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } { V _o ^{ v } } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } }
\end{cases}
\end{aligned}
\quad\quad(1.5)
$$<br/>

Invoking $E _v ^d = \hat { v } ^d - 1 _{ N } \otimes v _0$, we obtain the derivative to be $\dot { E } _v ^d = \dot { \hat { v } } ^d - 1 _{ N } \otimes u _0$. Subsequently, it can be derived that

$$
\begin{aligned}
{ \dot { V } _o ^{ v } } = { { ( E _v ^d ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) \dot { E } _v ^d } = { { ( E _v ^d ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) \dot { \hat { v } } ^d - { ( E _v ^d ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) ( 1 _{ N } \otimes u _0 ) }
\end{aligned}
\quad\quad(1.6)
$$<br/>

Leveraging Young's inequality and yield

$$
\begin{aligned}
{ - { ( E _v ^d ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) ( 1 _{ N } \otimes u _0 ) } & \le { \frac { 1 } { 2 } \cdot \frac { 1 } { 2 } ( { ( E _v ^d ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) ) { ( { ( E _v ^d ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) ) } ^{ T } + \frac { 1 } { 2 } 2 \cdot N \cdot { \overline { u } } ^{ 2 } } \\
& \le { \frac { 1 } { 4 } \lambda _{ max } ( \overline { L } ) ( { ( E _v ^d ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) E _v ^d ) + N { \overline u _{ 0 } } ^{ 2 } } \\
& \le { \frac { 1 } { 4 } \lambda _{ max } ( \overline { L } ) { ( V _o ^{ v } ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } + \frac { 1 } { 4 } \lambda _{ max } ( \overline { L } ) { ( V _o ^{ v } ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } + N { \overline u _{ 0 } } ^{ 2 } }
\end{aligned}
\quad\quad(1.7)
$$<br/>

where the control input $u _0$ for the leader vertex has an upper bound $\lVert u _{ 0 } \rVert \le \overline { u } _{ 0 }$. Substitute Eq.(1.7) into Eq.(1.6) and further yield

$$
\begin{aligned}
{ \dot { V } _o ^{ v } } \le { { ( E _v ^{ d } ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) \dot { \hat { v } } ^{ d } + \frac { 1 } { 4 } \lambda _{ max } ( \overline { L } ) { ( V _o ^{ v } ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } + \frac { 1 } { 4 } \lambda _{ max } ( \overline { L } ) { ( V _o ^{ v } ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } + N { \overline u _{ 0 } } ^{ 2 } }
\end{aligned}
\quad\quad(1.8)
$$<br/>

To maintain the practical fixed-time stability of the distributed velocity tracking error $\tilde { v } _i ^d$, it is equivalent that $E _v ^d = \hat { v } ^d - 1 _{ N } \otimes v _0$ can converge to zero. According to **Lemma 2**, the terms ${ ( V _o ^{ v } ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } }$ and ${ ( V _o ^{ v } ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } }$ are required to have negative coefficients to maintain the practical fixed-time stability. <br/>

Considering the existing terms in Eq.(1.8) such that $\frac { 1 } { 4 } \lambda _{ max } ( \overline { L } ) { ( V _o ^{ v } ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } }$ and $\frac { 1 } { 4 } \lambda _{ max } ( \overline { L } ) { ( V _o ^{ v } ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } }$, two corresponding terms with negative coefficients should exist in the terms derived from bounding the term ${ ( E _v ^{ d } ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) \dot { \hat { v } } ^{ d }$. Therefore, ${ ( E _v ^{ d } ) } ^{ T } \dot { \hat { v } } ^{ d } \le - \overline { l } _1 ^ {V} { ( V _o ^{ v } ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } - \overline { l } _2 ^ {V} { ( V _o ^{ v } ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } }$ can fulfill the practical fixed-time stability according to Eq.(1.8). Invoke Eq.(1.5) to design the term above, and suppose the feasible design as

$$
\begin{aligned}
{ { ( \tilde { E } _v ^{ d } ) } ^{ T } \dot { \hat { v } } ^{ d } } = { - \ell _1 ^{ v } { ( \tilde { E } _v ^{ d } ) } ^{ T } \vartheta ( \tilde { E } _v ^{ d } , \gamma _{ 1 }, \mu _o ^{ p } ) - \ell _2 ^{ v } { ( \tilde { E } _v ^{ d } ) } ^{ T } \vartheta ( \tilde { E } _v ^{ d } , \gamma _{ 2 }, \mu _o ^{ p } ) }
\end{aligned}
\quad\quad(1.9)
$$<br/>

Substitute Eq.(1.5) into Eq.(1.9), then the term ${ ( E _v ^{ d } ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) \dot { \hat { v } } ^{ d }$ is then derived as <br/>

$$
\begin{aligned}
{ { ( E _v ^{ d } ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) \dot { \hat { v } } ^{ d } } & = - \ell _1 ^{ v } { ( \tilde { E } _v ^{ d } ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) \vartheta ( \tilde { E } _v ^{ d }, \gamma _{ 1 }, \mu _o ^{ p } ) \\
& \quad \quad - \ell _2 ^{ v } { ( \tilde { E } _v ^{ d } ) } ^{ T } ( \overline { L } ^{ T } \otimes I _{ 3 } ) \vartheta ( \tilde { E } _v ^{ d }, \gamma _{ 2 }, \mu _o ^{ p } ) \\
& \le - 2 ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \gamma _{ 1 } } { 2 } } \ell _1 ^{ v } K _{ \gamma } ^{ 1 } { \lambda _{ \min } ( \overline { L } ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } { V _o ^{ v } } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } \\
& \quad \quad - 2 ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } \ell _2 ^{ v } K _{ \gamma } ^{ 2 } { \lambda _{ \min } ( \overline { L } ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } { V _o ^{ v } } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } }
\end{aligned}
\quad\quad(1.10)
$$<br/>

Combine Eq.(1.8) with Eq.(1.10) and we can further derive that

$$
\begin{aligned}
{ \dot { V } _o ^{ v } } & \le - ( 2 ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \gamma _{ 1 } } { 2 } } \ell _1 ^{ v } K _{ \gamma } ^{ 1 } { \lambda _{ \min } ( \overline { L } ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } - \lambda _{ \max } ( \overline { L } ) / 4 ) { V _o ^{ v } } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } \\
& \quad \quad - ( 2 ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } \ell _2 ^{ v } K _{ \gamma } ^{ 2 } { \lambda _{ \min } ( \overline { L } ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } - \lambda _{ \max } ( \overline { L } ) / 4 ) { V _o ^{ v } } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } + N { ( \overline { u } _{ 0 } ) } ^{ 2 }
\end{aligned}
\quad\quad(1.11)
$$<br/>



## 3.2 $\text{  }$ Observer implementation procedure <br/>

Observation error for follower UAV node $i$<br/>

$$
\begin{aligned}
e _{i,v} ^d = \hat v _i ^d - v_0 - \dot \delta _i
\end{aligned}
\quad\quad(2.1)
$$<br/>

$$
\begin{aligned}
e _{i,p} ^d = \hat p _i ^d - p_0 - \delta _i
\end{aligned}
\quad\quad(2.2)
$$<br/>

Lumped formation observation error for follower UAV node $i$<br/>

$$
\begin{aligned}
\tilde e _{i,v} ^d &= b _{i0} e ^d _{i,v} + \sum ^N _{j=1} w _{ij} ( e ^d _{i,v} - e ^d _{j,v} ) \\
&= b _{i0} ( \hat v _i ^d - v_0 - \dot \delta _i ) + \sum ^N _{j=1} w _{ij} ( ( \hat v _i ^d - \dot \delta _i ) - ( \hat v _j ^d - \dot \delta _j ) )
\end{aligned}
\quad\quad(2.3)
$$<br/>

$$
\begin{aligned}
\tilde e _{i,p} ^d &= b _{i0} e ^d _{i,p} + \sum ^N _{j=1} w _{ij} ( e ^d _{i,p} - e ^d _{j,p} ) \\
&= b _{i0} ( \hat p _i ^d - p_0 - \delta _i ) + \sum ^N _{j=1} w _{ij} ( ( \hat p _i ^d - \delta _i ) - ( \hat p _j ^d - \delta _j ) )
\end{aligned}
\quad\quad(2.4)
$$<br/>

Update desired state observation for follower UAV node $i$<br/>

$$
\begin{aligned}
\dot {\hat v} _i ^d = - \ell _1 ^v \vartheta ( \tilde e ^d _{i,v}, \gamma _1, \mu _o ^p ) - \ell _2 ^v \vartheta ( \tilde e ^d _{i,v}, \gamma _2, \mu _o ^p )
\end{aligned}
\quad\quad(2.5)
$$<br/>

$$
\begin{aligned}
\dot {\hat p} _i ^d = \hat v _i ^d - \ell _1 ^p \vartheta ( \tilde e ^d _{i,p}, \gamma _1, \mu _o ^p ) - \ell _2 ^p \vartheta ( \tilde e ^d _{i,p}, \gamma _2, \mu _o ^p )
\end{aligned}
\quad\quad(2.6)
$$<br/>


## 3.3 $\text{  }$ Parameter settings and validation <br/>




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


## A.2 $\text{  }$ Design of nonlinear smooth sigmoid function <br/>

A newly introduced **nonlinear smooth sigmoid vector** is defined as <br/>

$$
\begin{aligned}
\vartheta ( \overline x, \alpha, \gamma ) = [ \vartheta _1 ( x _1, \alpha, \gamma ), ..., \vartheta _n ( x _n, \alpha, \gamma ) ] ^T
\end{aligned}
\quad\quad(A.2)
$$<br/>

with each entry denoted as <br/>

$$
\begin{aligned}
\vartheta _k ( x _k, \alpha, \gamma ) = \lvert x _k \rvert ^ \alpha \lambda _k ( x _k, \gamma )
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

**Lemma 4** : For a vector $\overline x = [ x _1, ..., x _n ] ^T \in \mathbb R ^n$ , $\alpha > 0$, and $\gamma > 0$, define $\vartheta ( \overline x , \alpha, \gamma ) = [ \vartheta _1 ( x _1, \alpha, \gamma ), ..., \vartheta _n ( x _n, \alpha, \gamma ) ]^T$ with each entries $\vartheta _k ( x _k, \alpha, \gamma )$ as the form of Eq.(3), the inequalities are yielded as

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


