# 5. $\text{  }$ Practical fixed-time decentralized formation controller (PFxTDFC) <br/>

This markdown file **< README - Part 5 - 1.md >** includes the Lyapunov-based theoretical instruction for the distributed controller design according to the stability proof in **Chapter 5.1**. <br/> 

Please refer to **< README - Part 5 - 2.md >** the implementation procedure of the proposed distributed control scheme in **Chapter 5.2**, and also for the parameter settings and validation of the proposed distributed control scheme in **Chapter 5.3**. <br/>

Refer to **< README - Part 5 - 3.md >** for comparative simulation studies in **Chapter 5.4** against some representitive distributed controllers published recently in IEEE Transactions. <br/> 

The Appendix includes some significant mathematical definitions and lemmas that are essential in the theoretical proof. Please refer to **< README - Part 5 - 2.md >** for the Appendix after Chapter 5.2 and before Chapter 5.3. <br/> <br/>

## 5.1 $\text{  }$ Theories and design principles <br/>

For each vertex $i \in \lbrace 1, ..., N \rbrace$, the formation tracking error is originally defined as $e _i ^{ p } = p _{ i } - p _{ 0 } - \delta _{ i }$ and $e _i ^{ v } = v _{ i } - v _{ 0 } - \dot { \delta } _{ i }$. <br/>

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
\quad\quad(1.2)
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
& = { - g \overline { e } _{ 3 } + u _{ i } + T _{ i } R ( Q _i ^{ c } ) ( R ( Q _i ^{ e } ) - I _{ 3 } ) \overline { e } _{ 3 } / m _{ i } + d _{ i } ^{ v } - F _i ^{ \hat { v } } ( v _{ 0 }, \hat { v } _i ^{ d }, \hat { v } _j ^{ d } ) - \dot { \chi } _{ i } }
\end{aligned}
\quad\quad(1.10)
$$<br/>

If we design the Lyapunov function as $L _{ i } ^{ p } = L _i ^{ p, 1 } + L _i ^{ p, 2 } = { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } + \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i }$, the Lyapunov derivative of $L _{ i } ^{ p }$ is further derived as

$$
\begin{aligned}
{ \dot { L } _i ^{ p } } & \le - 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } - 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \\
& \quad \quad + \frac { 3 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } + \frac { 1 } { 4 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ p } ) } ^{ 2 } + { ( \phi _{ i } ) } ^{ T } ( - g \overline { e } _{ 3 } + u _{ i } \\
& \quad \quad + T _{ i } R ( Q _i ^{ c } ) ( R ( Q _i ^{ e } ) - I _{ 3 } ) \overline { e } _{ 3 } / m _{ i } + d _{ i } ^{ v } - F _i ^{ \hat { v } } ( v _{ 0 }, \hat { v } _i ^{ d }, \hat { v } _j ^{ d } ) - \dot { \chi } _{ i } )
\end{aligned}
$$<br/>

There is no existing assumption for that the unknown translational external disturbance is bounded, and thus the disturbance observer, namely the translational FxTDO $\hat { d } _i ^{ v }$ proposed in **Section 4**, is required to be introduced into the translational thrust control input $u _i$. Then we can yield that the observation error $\tilde { d } _i ^{ v } = d _i ^{ v } - \hat { d } _i ^{ v }$ can be reached within fixed time, and such that the translational disturbance observation error is bounded by $\lVert \tilde { d } _i ^{ v } \rVert \le \overline { D } _{ v }$. Besides, another redundant term $- { ( \phi _{ i } ) } ^{ T } \dot { \chi } _{ i }$ in the Lyapunov derivative $\dot { L } _i ^{ p }$ should be compensate by designing a $+ \dot { \chi } _{ i }$ term in the control input $u _i$. Moreover, the ${ ( \phi _{ i } ) } ^{ T } ( - g \overline { e } _{ 3 } )$ term should be compensated by the control input. The thrust control input is specified as $u _{ i } = \overset { \frown } { u } _{ i } + g \overline { e } _{ 3 } - \hat { d } _i ^{ v } + \dot { \chi } _{ i }$ to provide the compensation for both the external disturbance $d _i ^{ v }$ and the auxiliary derivative term $- \dot { \chi } _{ i }$. <br/>

Invoking Young's inequality and considering the upper bound for the redundant rotational error $\lVert T _{ i } R ( Q _i ^{ c } ) ( R ( Q _i ^{ e } ) - I _{ 3 } ) \overline { e } _{ 3 } / m _{ i } \rVert \le \overline { \Gamma } _{ u }$, it can be yielded that

$$
\begin{aligned}
\begin{cases}
{ { ( \phi _{ i } ) } ^{ T } ( \frac { T _{ i } } { m _{ i } } R ( Q _i ^{ c } ) ( R ( Q _i ^{ e } ) - I _{ 3 } ) \overline { e } _{ 3 } ) } \le { \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { 1 } { 2 } { ( \overline { \Gamma } _{ u } ) } ^{ 2 } } \\
{ { ( \phi _{ i } ) } ^{ T } ( - F _i ^{ \hat { v } } ( v _{ 0 }, \hat { v } _i ^{ d }, \hat { v } _j ^{ d } ) ) } \le { \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ v } ) } ^{ 2 } } \\
{ { ( \phi _{ i } ) } ^{ T } \tilde { d } _i ^{ v } } \le { \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { 1 } { 2 } { ( \overline { D } _{ v } ) } ^{ 2 } }
\end{cases}
\end{aligned}
\quad\quad(1.11)
$$<br/>

Then the term to be specified can be scaled as

$$
\begin{aligned}
& { ( \phi _{ i } ) } ^{ T } ( - g \overline { e } _{ 3 } + u _{ i } + T _{ i } R ( Q _i ^{ c } ) ( R ( Q _i ^{ e } ) - I _{ 3 } ) \overline { e } _{ 3 } / m _{ i } + d _{ i } ^{ v } - F _i ^{ \hat { v } } ( v _{ 0 }, \hat { v } _i ^{ d }, \hat { v } _j ^{ d } ) - \dot { \chi } _{ i } ) \\
& \le { ( \phi _{ i } ) } ^{ T } ( - g \overline { e } _{ 3 } + \overset { \frown } { u } _{ i } + g \overline { e } _{ 3 } - \hat { d } _i ^{ v } + \dot { \chi } _{ i } + d _{ i } ^{ v } - \dot { \chi } _{ i } ) \\
& \quad \quad + \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { 1 } { 2 } { ( \overline { \Gamma } _{ u } ) } ^{ 2 } + \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ v } ) } ^{ 2 } \\
& \le { ( \phi _{ i } ) } ^{ T } ( \overset { \frown } { u } _{ i } + \tilde { d } _i ^{ v } ) + { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { 1 } { 2 } { ( \overline { \Gamma } _{ u } ) } ^{ 2 } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ v } ) } ^{ 2 }
\end{aligned}
$$<br/>


Therefore, the upper bound for $\dot { L } _i ^{ p }$ is further derived as

$$
\begin{aligned}
{ \dot { L } _i ^{ p } } & \le - 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } - 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \\
& \quad \quad + \frac { 3 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } + \frac { 7 } { 4 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ p } ) } ^{ 2 } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ v } ) } ^{ 2 } + \frac { 1 } { 2 } { ( \overline { D } _{ v } ) } ^{ 2 } + { ( \phi _{ i } ) } ^{ T } \overset { \frown } { u } _{ i }
\end{aligned}
\quad\quad(1.12)
$$<br/>

Since $\beta _{ 1 } > 1$ and $\beta _{ 2 } \in ( 0, 1 )$, we can deduce that if $\frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } \in ( 0, 1 )$ holds, there exists $\frac { 3 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } \le 3 { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } }$, and if $\frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } > 1$ holds, there exists $\frac { 3 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } \le 3 { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } }$. Besides, the same deduction can be obtained such that if $\frac { 1 } { 2 } { ( \phi ^{ i } ) } ^{ T } \phi _{ i } \in ( 0, 1 )$ holds, there exists $\frac { 7 } { 4 } { ( \phi _{ i } ) } ^{ T } \phi ^{ i } \le \frac { 7 } { 2 } { ( \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } }$, and if $\frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } > 1$, there exists $\frac { 7 } { 4 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } \le \frac { 7 } { 2 } { ( \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } }$. <br/>

It can be derived from aforementioned inequalities that

$$
\begin{aligned}
\begin{cases}
& { \frac { 3 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } } \le { 3 { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } + 3 { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } } \\
& { \frac { 7 } { 4 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } } \le { \frac { 7 } { 2 } { ( \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } + \frac { 7 } { 2 } { ( \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } }
\end{cases}
\end{aligned}
$$<br/>

Subsequently, the upper bound of the Lyapunov candidate is further denoted as <br/>

$$
\begin{aligned}
{ \dot { L } _i ^{ p } } & \le - ( 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ \chi } K _{ \beta } ^{ 1 } - 3 ) { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } - ( 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ \chi } K _{ \beta } ^{ 1 } - 3 ) { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \\
& \quad \quad + \frac { 7 } { 4 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ p } ) } ^{ 2 } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ v } ) } ^{ 2 } + \frac { 1 } { 2 } { ( \overline { D } _{ v } ) } ^{ 2 } + { ( \phi _{ i } ) } ^{ T } \overset { \frown } { u } _{ i }
\end{aligned}
\quad\quad(1.12)
$$<br/>

In the aforementioned design, we employ the inequalities from Eq.(1.6) to develop the upper bound $- 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } - 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ \chi } K _{ \beta } ^{ 1 } { ( \frac { 1 } { 2 } { ( e _{ i } ^{ p } ) } ^{ T } e _{ i } ^{ p } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } }$ for some terms in the derivative $\dot { L } _{ i } ^{ p,1 } = { ( e _{ i } ^{ p } ) } ^{ T } \dot { e } _{ i } ^{ p }$. Now we are required to turn the positive definite terms $\frac { 7 } { 2 } { ( \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } }$ and $\frac { 7 } { 2 } { ( \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } }$ in Eq.(1.13) into terms with negative sign for guaranteeing pratical fixed-time stability. Therefore, we need to take design an appropriate translational control input $\overset { \frown } { u } _{ i }$ to introduce an upper bound for the term ${ ( \phi _{ i } ) } ^{ T } \overset { \frown } { u } _{ i }$. <br/>

Similar to the inequalities derived in Eq.(1.6), we can select the parameters to guarantee that $\kappa _1 ^{ u } > 0$, $\kappa _2 ^{ u } > 0$, we invoke **Lemma 4** and thereafter yield that

$$
\begin{aligned}
\begin{cases}
& { - \kappa _1 ^{ u } { ( \phi _{ i } ) } ^{ T } \vartheta ( \phi _{ i }, \beta _{ 1 }, \mu _c ^{ p } ) } \le { - 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ u } K _{ \beta } ^{ 2 } { ( \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } } \\
& { - \kappa _2 ^{ u } { ( \phi _{ i } ) } ^{ T } \vartheta ( \phi _{ i }, \beta _{ 2 }, \mu _c ^{ p } ) } \le { - 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ u } K _{ \beta } ^{ 2 } { ( \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } }
\end{cases}
\end{aligned}
\quad\quad(1.13)
$$<br/>

where $K _{ \beta } ^{ 2 } = min \lbrace \frac { 1 } { \overline { \epsilon } _{ \beta } } , \underline { \epsilon } _{ \beta } \rbrace$ is selected according to **Lemma 4**. <br/>

We can design the auxiliary control input as 

$$
\begin{aligned}
\overset { \frown } { u } _{ i } = - \kappa _1 ^{ u } \vartheta ( \phi _{ i }, \beta _{ 1 }, \mu _c ^{ p } ) - \kappa _2 ^{ u } \vartheta ( \phi _{ i }, \beta _{ 2 }, \mu _c ^{ p } )
\end{aligned}
$$<br/>

to maintain the following inequalities that

$$
\begin{aligned}
{ ( \phi _{ i } ) } ^{ T } \overset { \frown } { u } _{ i } & = - \kappa _1 ^{ u } { ( \phi _{ i } ) } ^{ T } \vartheta ( \phi _{ i }, \beta _{ 1 }, \mu _c ^{ p } ) - \kappa _2 ^{ u } { ( \phi _{ i } ) } ^{ T } \vartheta ( \phi _{ i }, \beta _{ 2 }, \mu _c ^{ p } ) \\
& \le - 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ u } K _{ \beta } ^{ 2 } { ( \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } - 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ u } K _{ \beta } ^{ 2 } { ( \frac { 1 } { 2 } { ( \phi _{ i } ) } ^{ T } \phi _{ i } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } }
\end{aligned}
$$<br/>

Substitute the above deduction into Eq.(1.12) and yield that

$$
\begin{aligned}
{ \dot { L } _i ^{ p } } & \le - ( 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ \chi } K _{ \beta } ^{ 1 } - 3 ) { ( L _{ i } ^{ p, 1 } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } - ( 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ \chi } K _{ \beta } ^{ 1 } - 3 ) { ( L _{ i } ^{ p, 1 } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \\
& \quad \quad - ( 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ u } K _{ \beta } ^{ 2 } - \frac { 7 } { 2 } ) { ( L _{ i } ^{ p, 2 } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } - ( 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ u } K _{ \beta } ^{ 2 } - \frac { 7 } { 2 } ) { ( L _{ i } ^{ p, 2 } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \\
& \quad \quad + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ p } ) } ^{ 2 } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ v } ) } ^{ 2 } + \frac { 1 } { 2 } { ( \overline { \Gamma } _{ u } ) } ^{ 2 } + \frac { 1 } { 2 } { ( \overline { D } _{ v } ) } ^{ 2 }
\end{aligned}
$$<br/>

We select two coefficients as $A _{ 1 } = \min \lbrace 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ \chi } K _{ \beta } ^{ 1 } - 3 , 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ u } K _{ \beta } ^{ 2 } - \frac { 7 } { 2 } \rbrace$ and $A _{ 2 } = \min \lbrace 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ \chi } K _{ \beta } ^{ 1 } - 3 , 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ u } K _{ \beta } ^{ 2 } - \frac { 7 } { 2 } \rbrace$. It is easy to find the following inequalities hold

$$
\begin{aligned}
\begin{cases}
& - ( 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ \chi } K _{ \beta } ^{ 1 } - 3 ) { ( L _{ i } ^{ p, 1 } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } - ( 2 ^{ \frac { \beta _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ 1 } } { 2 } } \kappa _1 ^{ u } K _{ \beta } ^{ 2 } - \frac { 7 } { 2 } ) { ( L _{ i } ^{ p, 2 } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } \le - A _{ 1 } { ( L _{ i } ^{ p } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } \\
& - ( 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ \chi } K _{ \beta } ^{ 1 } - 3 ) { ( L _{ i } ^{ p, 1 } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } - ( 2 ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \kappa _2 ^{ u } K _{ \beta } ^{ 2 } - \frac { 7 } { 2 } ) { ( L _{ i } ^{ p, 2 } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } \le - A _{ 2 } { ( L _{ i } ^{ p } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } }
\end{cases}
\end{aligned}
$$<br/>

Define $A _{ 3 } = \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ p } ) } ^{ 2 } + \frac { 1 } { 2 } { ( \overline { F } _{ O } ^{ v } ) } ^{ 2 } + \frac { 1 } { 2 } { ( \overline { \Gamma } _{ u } ) } ^{ 2 } + \frac { 1 } { 2 } { ( \overline { D } _{ v } ) } ^{ 2 }$, and thus we can derive from the above theories that 

$$
\begin{aligned}
\dot { L } \le - A _{ 1 } { ( L _{ i } ^{ p, 1 } ) } ^{ \frac { \beta _{ 1 } + 1 } { 2 } } - A _{ 2 } { ( L _{ i } ^{ p, 2 } ) } ^{ \frac { \beta _{ 2 } + 1 } { 2 } } + A _{ 3 }
\end{aligned}
\quad\quad(1.14)
$$<br/>

Therefore, it can be yielded from **Lemma 2** that $e _i ^{ p }$ and $\phi _{ i }$ can reach practical fixed-time stable in fixed time. Since $\chi _{ i }$ shares the same convergence property as $e _i ^{ p }$, then the convergence of $\phi _{ i }$ is equivalent to $e _{ i } ^{ v }$. Finally we can obtain that the formation tracking error in position and velocity can achieve convergence in practical fixed time by designing proper **translational control input** as shown in **Chapter 5.2**. <br/>

