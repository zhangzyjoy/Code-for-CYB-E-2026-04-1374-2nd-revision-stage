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

