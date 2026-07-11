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


