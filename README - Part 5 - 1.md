# 5. $\text{  }$ Practical fixed-time decentralized formation controller (PFxTDFC) <br/>

This markdown file **< README - Part 5 - 1.md >** includes the Lyapunov-based theoretical instruction for the distributed controller design according to the stability proof in **Chapter 5.1**, and also includes the implementation procedure of the proposed distributed control scheme in **Chapter 5.2**. <br/> 

Please refer to **< README - Part 5 - 2.md >** for the parameter settings and validation of the proposed distributed control scheme in **Chapter 5.3**. <br/>

Refer to **< README - Part 5 - 3.md >** for comparative simulation studies in **Chapter 5.4** against some representitive distributed controllers published recently in IEEE Transactions. <br/> <br/>

## 5.1 $\text{  }$ Theories and design principles <br/>

For each vertex $i \in \lbrace 1, ..., N \rbrace$, the formation tracking error is originally defined as

$$
\begin{aligned}
\begin{cases}
& { e _i ^p } = { p _{ i } - p _{ 0 } - \delta _i } \\
& { e _i ^v } = { v _{ i } - v _{ 0 } - \dot { \delta } _i }
\end{cases}
\end{aligned}
$$<br/>

As denoted in the markdown file **<README - Part 3.md>**, the desired position and velocity of the $i$-th follower vertex is given by $\hat { p } _i ^d$ and $\hat { v } _i ^d$, respectively. Furthermore, it can be proved that the observation errors for position and velocity, denoted as $\tilde { e } _{ i, p } ^d = \hat { p } _i ^{ d } - p _{ 0 } - \delta _i$ and $\tilde { e } _{ i, v } ^d = \hat { v } _i ^{ d } - v _{ 0 } - \dot { \delta } _i$ can respectively converge into the origin and reach practical fixed-time stability. In other words, $\hat { p } _i ^{ d } \to p _{ 0 } + \delta _i$ and $\hat { v } _i ^{ d } \to v _{ 0 } + \dot { \delta } _i$ can reach the convergence in practical fixed time. <br/>

Substitute the observation value $\hat { p } _i ^{ d }$ and $\hat { v } _i ^{ d }$ into the desired follower position $p _{ 0 } + \delta _i$ and desired follower velocity $v _{ 0 } + \dot { \delta } _i$, respectively. Then we obtain the novel formation tracking error terms based on the observation, which is suitable to be employed in the distributed control system as <br/>

$$
\begin{aligned}
\begin{cases}
& { e _i ^p } = { p _{ i } - \hat { p } _i ^{ d } } \\
& { e _i ^v } = { v _{ i } - \hat { v } _i ^{ d } }
\end{cases}
\end{aligned}
\quad\quad(1.1)
$$<br/>

Substitute $u _{ i } = T _{ i } R ( Q _i ^{ c } ) \overline { e } _{ 3 } / m _{ i }$ into the linear velocity dynamics, we can yield that

$$
\begin{aligned}
{ \dot v _i } = { -g \overline e _3 + u _i + T _{ i } R( Q _i ^c ) ( R ( Q _i ^e ) - I _3 ) \overline e _3 / m _i + d _i ^v }
\end{aligned}
$$<br/>

Employing Eq.(1.1) and Eq.(A.7), we can derive the following equalities by substituting Eq.(A.7) into the derivative of Eq.(1.1) such that

$$
\begin{aligned}
\begin{cases}
{ \dot { e } _i ^{ p } } & = { \dot { p } _{ i } - \dot { \hat { p } } _i ^{ d } = v _{ i } - \hat { v } _i ^{ d } - \ell _1 ^{ p } \vartheta ( \tilde { e } _{ i,p } ^{ d }, \gamma _1, \mu _o ^{ p } ) - \ell _2 ^{ p } \vartheta ( \tilde e _{ i,p } ^{ d }, \gamma _2, \mu _o ^{ p } ) } \\
{ \dot { e } _i ^{ v } } & = { \dot { v } _{ i } - \dot { \hat { v } } _i ^{ d } = - g \overline { e } _{ 3 } + u _{ i } + T _{ i } R( Q _i ^{ c } ) ( R( Q _i ^{ e } ) - I _3 ) \overline e _3 / m _i + d _i ^{ v } - \ell _1 ^{ v } \vartheta ( \tilde { e } _{ i,v } ^{ d }, \gamma _1, \mu _o ^{ p } ) - \ell _2 ^{ v } \vartheta ( \tilde { e } _{ i,v } ^{ d }, \gamma _2, \mu _o ^{ p } ) }
\end{cases}
\end{aligned}
\quad\quad(1.3)
$$<br/>

Invoking $e _i ^{ v } = v _{ i } - \hat { v } _i ^{ d }$, the formation tracking error derivatives of each uncrewed aerial vehicle is further derived by

$$
\begin{aligned}
\begin{cases}
& { F _i ^{ \hat { p } } ( p _{ 0 }, \hat { p } _i ^{ d }, \hat { p } _j ^{ d } ) } = { \ell _1 ^{ p } \vartheta ( \tilde { e } _{ i,p } ^{ d }, \gamma _1, \mu _o ^{ p } ) + \ell _2 ^{ p } \vartheta ( \tilde e _{ i,p } ^{ d }, \gamma _2, \mu _o ^{ p } ) } \\
& { F _i ^{ \hat { v } } ( v _{ 0 }, \hat { v } _i ^{ d }, \hat { v } _j ^{ d } ) } = { \ell _1 ^{ v } \vartheta ( \tilde { e } _{ i,v } ^{ d }, \gamma _1, \mu _o ^{ p } ) + \ell _2 ^{ v } \vartheta ( \tilde { e } _{ i,v } ^{ d }, \gamma _2, \mu _o ^{ p } ) } \\
& { \dot { e } _i ^{ p } } = { e _i ^{ v } - F _i ^{ \hat { p } } ( p _{ 0 }, \hat { p } _i ^{ d }, \hat { p } _j ^{ d } ) } \\
& { \dot { e } _i ^{ v } } = { - g \overline { e } _{ 3 } + u _{ i } + T _{ i } R( Q _i ^{ c } ) ( R( Q _i ^{ e } ) - I _3 ) \overline e _3 / m _i + d _i ^{ v } - F _i ^{ \hat { v } } ( v _{ 0 }, \hat { v } _i ^{ d }, \hat { v } _j ^{ d } ) }
\end{cases}
\end{aligned}
\quad\quad(1.3)
$$<br/>

In Eq.(1.3), $R ( Q _i ^{ e } ) \to I _3$ holds when the rotational error terms in the rotational subsystem converges into the origin. Therefore, the upper bound is determined by $\lVert T _{ i } R ( Q _i ^{ c } ) ( R ( Q _i ^{ e } ) - I _{ 3 } ) \overline { e } _{ 3 } / m _{ i } \rVert \le \overline { \Gamma } _{ u }$. <br/>

If $\hat { p } _i ^{ d } \to p _{ 0 } + \delta _i$ and $\hat { v } _i ^{ d } \to v _{ 0 } + \dot { \delta } _i$ are reached within the practical fixed time, it can be concluded easily that $\tilde { e } _{ i, p } ^{ d } \to 0$ holds after considering $\tilde { E } _{ p } ^{ d } = ( \overline { L } \otimes I _{ 3 } ) E _{ p } ^{ d }$, and the same deduction can be obtained as $\tilde { e } _{ i, v } ^{ d } \to 0$. Since $\vartheta ( \tilde e _{ i,p } ^{ d }, \gamma _k, \mu _o ^{ p } )$ and $\vartheta ( \tilde e _{ i,v } ^{ d }, \gamma _k, \mu _o ^{ p } )$ shares the same convergence property with $\tilde e _{ i,p } ^{ d }$ and $\tilde e _{ i,v } ^{ d }$, respectively, then after reaching the practical fixed time, tracking errors $\tilde e _{ i,p } ^{ d }$ and $\tilde e _{ i,v } ^{ d }$ can converge to small neighborhood region near zero and such that the following bounding properties are yielded as

$$
\begin{aligned}
\lVert F _i ^{ \hat { v } } ( v _{ 0 }, \hat { v } _i ^{ d }, \hat { v } _j ^{ d } ) \rVert \le \overline { F } _o ^{ V }, \quad \quad \lVert F _i ^{ \hat { p } } ( p _{ 0 }, \hat { p } _i ^{ d }, \hat { p } _j ^{ d } ) \rVert \le \overline { F } _o ^{ P }
\end{aligned}
\quad\quad(1.4)
$$<br/>

In order to achieve the convergence of the formation tracking error $e _i ^{ p }$ in Eq.(1.1), we need to design the proper thrust control input $u _{ i }$. A primary Lyapunov candidate is defined as $L _i ^{ p, 1 } = { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } / 2$ and its derivative is deduced by invoking Eq.(1.4) as

$$
\begin{aligned}
{ \dot { L } _i ^{ p, 1 } } & = { { ( e _{ i } ^{ p } ) } ^{ T } e _i ^{ v } - { ( e _{ i } ^{ p } ) } ^{ T } F _i ^{ \hat { p } } ( p _{ 0 }, \hat { p } _i ^{ d }, \hat { p } _j ^{ d } ) } \\
& \le { { ( e _{ i } ^{ p } ) } ^{ T } e _i ^{ v } + \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } + \frac { 1 } { 2 } \lVert F _i ^{ \hat { p } } ( p _{ 0 }, \hat { p } _i ^{ d }, \hat { p } _j ^{ d } ) \rVert ^{ 2 } } \\
& \le { { ( e _{ i } ^{ p } ) } ^{ T } e _i ^{ v } + \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } + \frac { 1 } { 2 } { ( \overline { F } _O ^{ p } ) } ^{ 2 } }
\end{aligned}
\quad\quad(1.5)
$$<br/>

According to Eq.(1.5), the term ${ ( e _{ i } ^{ p } ) } ^{ T } e _i ^{ v }$ requires auxiliary system to tackle with $e _i ^{ v }$. Select parameters to guarantee that $\beta _{ 1 } > 1$, $0 < \beta _{ 2 } < 1$, $\kappa _1 ^{ \chi } > 0$, $\kappa _2 ^{ \chi } > 0$, $\mu _c ^{ p } > 0$, we invoke **Lemma 4** and thereafter yield that

$$
\begin{aligned}
\begin{cases}
& { - \kappa _1 ^{ \chi } { ( e _{ i } ^{ p } ) } ^{ T } \vartheta ( e _{ i } ^{ p }, \beta _{ 1 }, \mu _c ^{ p } ) } \le { - 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } } \\
& { - \kappa _2 ^{ \chi } { ( e _{ i } ^{ p } ) } ^{ T } \vartheta ( e _{ i } ^{ p }, \beta _{ 2 }, \mu _c ^{ p } ) } \le { - 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } }
\end{cases}
\end{aligned}
\quad\quad(1.6)
$$<br/>

where $K _{ \beta } ^{ 1 } = min \lbrace \frac { 1 } { \overline { \epsilon } _{ \beta } } , \underline { \epsilon } _{ \beta } \rbrace$ is selected according to **Lemma 4**. <br/>

We can introduce $- \kappa _1 ^{ \chi } \vartheta ( e _{ i } ^{ p }, \beta _{ 1 }, \mu _c ^{ p } )$ and $- \kappa _2 ^{ \chi } \vartheta ( e _{ i } ^{ p }, \beta _{ 2 }, \mu _c ^{ p } )$ into $e _{ i } ^{ v }$ to provide upper bounds for partial terms in $e _{ i } ^{ v }$ according to Eq.(1.6), and further we introduces terms that are relevant to $L _i ^{ p, 1 }$ into ${ ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ v }$, and thereafter expect to obtain inequalities in the form of **Lemma 2**. This indicates that the velocity tracking error should be set as 

$$
\begin{aligned}
e _{ i } ^{ v } = \phi _{ i } - \kappa _1 ^{ \chi } \vartheta ( e _{ i } ^{ p }, \beta _{ 1 }, \mu _c ^{ p } ) - \kappa _2 ^{ \chi } \vartheta ( e _{ i } ^{ p }, \beta _{ 2 }, \mu _c ^{ p } )
\end{aligned}
\quad\quad(1.7)
$$<br/>

where $\chi _{ i } = - \kappa _1 ^{ \chi } \vartheta ( e _{ i } ^{ p }, \beta _{ 1 }, \mu _c ^{ p } ) - \kappa _2 ^{ \chi } \vartheta ( e _{ i } ^{ p }, \beta _{ 2 }, \mu _c ^{ p } )$ and $\phi _{ i } = e _{ i } ^{ v } - \chi _{ i }$ are provided. <br/>

By employing Young's Inequality, it can be yielded that

$$
\begin{aligned}
{ { ( e _{ i } ^{ p } ) } ^{ T } \phi _{ i } \le { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } + \frac { 1 } { 4 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } }
\end{aligned}
\quad\quad(1.8)
$$<br/>


Substitute Eq.(1.7) into ${ ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ v }$ and then reformulate Eq.(1.5) as

$$
\begin{aligned}
{ \dot { L } _i ^{ p, 1 } } & \le { { ( e _{ i } ^{ p } ) } ^{ T } \phi _{ i } - \kappa _1 ^{ \chi } { ( e _{ i } ^{ p } ) } ^{ T } \vartheta ( e _{ i } ^{ p }, \beta _{ 1 }, \mu _c ^{ p } ) - \kappa _2 ^{ \chi } { ( e _{ i } ^{ p } ) } ^{ T } \vartheta ( e _{ i } ^{ p }, \beta _{ 2 }, \mu _c ^{ p } ) + \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } + \frac { 1 } { 2 } { ( \overline { F } _O ^{ p } ) } ^{ 2 } }
\end{aligned}
$$<br/>

Invoking Eq.(1.6) and Eq.(1.8), it can be further yielded that

$$
\begin{aligned}
{ \dot { L } _i ^{ p, 1 } } & \le - 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } - 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \\
& \quad \quad + \frac { 3 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } + \frac { 1 } { 4 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ p } ) } ^{ 2 }
\end{aligned}
\quad\quad(1.9)
$$<br/>


According to Eq.(1.9), there exist redundant terms $\frac { 1 } { 4 } { ( \phi _{ i } ) } ^{ T } \phi _{ i }$ that is irrelevant with ${ ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p }$ in $\dot { L } _i ^{ p, 1 }$. Therefore, $L _i ^{ p, 1 } = \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i }$ is required to be introduced into the overall Lyapunov function. Since the virtual tracking error is defined as $\phi _{ i } = e _{ i } ^{ v } - \chi _{ i } = v _{ i } - \hat { v } _{ i } ^{ d } - \chi _{ i }$, we can obtain its derivative as

$$
\begin{aligned}
{ \dot { \phi } _{ i } } & = { \dot { v } _{ i } - \dot { \hat { v } } _{ i } ^{ d } - \dot { \chi } _{ i } } \\
& = { -g \overline { e } _{ 3 } + u _{ i } + T _{ i } R ( Q _i ^{ c } ) ( R ( Q _i ^{ e } ) - I _{ 3 } ) \overline { e } _{ 3 } / m _{ i } - F _i ^{ \hat { v } } ( v _{ 0 }, \hat { v } _i ^{ d }, \hat { v } _j ^{ d } ) - \dot { \chi } _{ i } }
\end{aligned}
\quad\quad(1.10)
$$<br/>

If we design the Lyapunov function as $L _{ i } ^{ p } = L _i ^{ p, 1 } + L _i ^{ p, 2 } = { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } + \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i }$, the Lyapunov derivative of $L _{ i } ^{ p }$ is further derived as

$$
\begin{aligned}
{ \dot { L } _i ^{ p } } & \le - 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } - 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \\
& \quad \quad + \frac { 3 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } + \frac { 1 } { 4 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ p } ) } ^{ 2 } + { ( \phi _{ i } ) } ^{ T } ( - g \overline { e } _{ 3 } + u _{ i } \\
& \quad \quad + T _{ i } R ( Q _i ^{ c } ) ( R ( Q _i ^{ e } ) - I _{ 3 } ) \overline { e } _{ 3 } / m _{ i } - F _i ^{ \hat { v } } ( v _{ 0 }, \hat { v } _i ^{ d }, \hat { v } _j ^{ d } ) - \dot { \chi } _{ i } )
\end{aligned}
$$<br/>

There exist upper bound for the thrust control input as $T _{ i } \le \overline { T }$ and lower bound for the mass among UAVs as $m _i \ge \underline { m }$. Invoking Young's inequality, we can obtain that

$$
\begin{aligned}
\begin{cases}
{ { ( \phi _{ i } ) } ^{ T } ( \frac { T _{ i } } { m _{ i } } R ( Q _i ^{ c } ) ( R ( Q _i ^{ e } ) - I _{ 3 } ) \overline { e } _{ 3 } ) } \le { \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { \overline { T } } { 2 \underline { m } } } \\
{ { ( \phi _{ i } ) } ^{ T } ( - F _i ^{ \hat { v } } ( v _{ 0 }, \hat { v } _i ^{ d }, \hat { v } _j ^{ d } ) ) } \le { \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ v } ) } ^{ 2 } }
\end{cases}
\end{aligned}
\quad\quad(1.11)
$$<br/>

Invoking Eq.(1.11), the upper bound for $\dot { L } _i ^{ p }$ is further derived as

$$
\begin{aligned}
{ \dot { L } _i ^{ p } } & \le - 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } - 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \\
& \quad \quad + \frac { 3 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } + \frac { 5 } { 4 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ p } ) } ^{ 2 } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ v } ) } ^{ 2 } + \frac { \overline { T } } { 2 \underline { m } } + { ( \phi _{ i } ) } ^{ T } ( - g \overline { e } _{ 3 } + u _{ i } \\
& \quad \quad + T _{ i } R ( Q _i ^{ c } ) ( R ( Q _i ^{ e } ) - I _{ 3 } ) \overline { e } _{ 3 } / m _{ i } - F _i ^{ \hat { v } } ( v _{ 0 }, \hat { v } _i ^{ d }, \hat { v } _j ^{ d } ) - \dot { \chi } _{ i } )
\end{aligned}
\quad\quad(1.11)
$$<br/>








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

**Lemma 3** : Consider a sequence with positive scalars $q _1, \text{ } q _2, \text{ } ..., \text{ } q_N$ where $q _k \text{ } \ge \text{ } 0$ for any $k \in \lbrace 1,...,N \rbrace$. Given that $0 < \alpha _1 \le 1$, $\alpha _2 > 1$, it can be yielded that

$$
\begin{aligned}
\sum _{ k = 1 } ^{ N } { ( q _k ) ^{ \alpha _1 } } \ge { ( \sum _{ k = 1 } ^{ N } { q _k } ) } ^{ \alpha _1 }, \quad \quad \sum _{ k = 1 } ^{ N } { ( q _k ) ^{ \alpha _2 } } \ge N ^{ 1 - \alpha _2 } { ( \sum _{ k = 1 } ^{ N } { q _k } ) } ^{ \alpha _2 }
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
\lambda _k ( x _k, \gamma ) = -1 + \frac 1 { 1 + \exp ( - \gamma x _k ) }
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


