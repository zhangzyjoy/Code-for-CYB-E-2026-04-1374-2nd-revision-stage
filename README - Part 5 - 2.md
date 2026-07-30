# 5. $\text{  }$ Practical fixed-time decentralized formation controller (PFxTDFC) <br/>

This markdown file **< README - Part 5 - 2.md >** includes the implementation procedure of the proposed distributed control scheme in **Chapter 5.2**, and also includes the parameter settings and validation of the proposed distributed control scheme in **Chapter 5.3**. <br/> 

Please refer to **< README - Part 5 - 1.md >** for the Lyapunov-based theoretical instruction for the distributed controller design according to the stability proof in **Chapter 5.1**. <br/>

Refer to **< README - Part 5 - 3.md >** for comparative simulation studies in **Chapter 5.4** against some representitive distributed controllers published recently in IEEE Transactions. <br/> <br/>


## 5.2 $\text{  }$ Controller implementation procedure <br/>


### 5.2.1 $\text{  }$ Implementation procedure for numerical simulation <br/>


Position and linear velocity tracking error

$$
\begin{aligned}
{ e _i ^v } = { v _i - \hat { v } _i ^{ d } }
\end{aligned}
\quad\quad(2.1)
$$<br/>

$$
\begin{aligned}
{ e _i ^p } = { p _i - \hat { p } _i ^{ d } }
\end{aligned}
\quad\quad(2.2)
$$<br/>

The auxiliary velocity control variable

$$
\begin{aligned}
{ \chi _i } = { - \kappa _1 ^{ \chi } { \vartheta ( e _i ^p, \beta _1, \mu _c ^p ) } - \kappa _2 ^{ \chi } { \vartheta ( e _i ^p, \beta _2, \mu _c ^p ) } }
\end{aligned}
\quad\quad(2.3)
$$<br/>


The virtual velocity tracking error

$$
\begin{aligned}
{ \phi _i } = { e _i ^v - \chi _i }
\end{aligned}
\quad\quad(2.4)
$$<br/>

Design the translational control input

$$
\begin{aligned}
{ u _i } = { g \overline e _3 + \dot \chi _i - \kappa _1 ^u \vartheta ( \phi _i, \beta _1, \mu _c ^p ) - \kappa _2 ^u \vartheta ( \phi _i, \beta _2, \mu _c ^p ) - \hat d _i ^v }
\end{aligned}
\quad\quad(2.5)
$$<br/>

Update the translational dynamics

$$
\begin{aligned}
\begin{cases}
&{ \dot v _i } = { -g \overline e _3 + u _i + T _i R( Q _i ^c ) ( R( Q _i ^e ) - I _3 ) \overline e _3 / m _i + d _i ^v } \\
&{ \dot p _i } = { v _i }
\end{cases}
\end{aligned}
\quad\quad(2.6)
$$<br/>


### 5.2.2 $\text{  }$ Implementation procedure for experimental validation <br/>

The translational dynamics Eq.(A.7) of each uncrewed aerial vehicle is simplified as a second-order agent with external disturbance <br/>

$$
\begin{aligned}
\begin{cases}
&{ \dot v _i } = { U _i + d _i ^v } \\
&{ \dot p _i } = { v _i }
\end{cases}
\end{aligned}
\quad\quad(2.7)
$$<br/>

Design $U _i$ to reformulate the PFxTDFC Eq.(2.5) to fit for employment in a second-order double-integrator system depicted by Eq.(2.7) as <br/>

$$
\begin{aligned}
{ U _i } = { \dot \chi _i - \kappa _1 ^u \vartheta ( \phi _i, \beta _1, \mu _c ^p ) - \kappa _2 ^u \vartheta ( \phi _i, \beta _2, \mu _c ^p ) - \hat d _i ^v }
\end{aligned}
\quad\quad(2.8)
$$<br/>


## 5.3 $\text{  }$ Parameter settings and validation


