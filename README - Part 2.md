
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
