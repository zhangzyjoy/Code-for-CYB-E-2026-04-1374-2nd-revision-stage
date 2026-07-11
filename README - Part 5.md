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


