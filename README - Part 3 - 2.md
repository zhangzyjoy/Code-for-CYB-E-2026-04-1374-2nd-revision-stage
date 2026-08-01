# 3. $\text{  }$ Practical fixed-time distributed state observer (PFxTDSO) <br/>

This markdown file **< README - Part 3 - 1.md >** includes the Lyapunov-based theoretical instruction for the distributed observer design according to the stability proof in **Chapter 3.1**, and also includes the implementation procedure of the proposed disturbance observer scheme in **Chapter 3.2**. <br/> 

Please refer to **< README - Part 3 - 2.md >** for the parameter settings and validation of the proposed observer in **Chapter 3.3**. <br/>

Refer to **< README - Part 3 - 3.md >** for comparative simulation studies in **Chapter 3.4** against some representitive observers published recently in IEEE Transactions. <br/> <br/>


## 3.3 $\text{  }$ Parameter settings and validation <br/>

The desired linear velocity $v _{ 0 } + \dot { \delta } _{ i }$ of the $i$-th uncrewed aerial vehicle is estimated by the distributed observer denoted by Eq.(2.5) in practical fixed time. According to **Lemma 2**, if we select the parameters that are constrained by

$$
\begin{aligned}
\begin{cases}
& \gamma _{ 1 } > 1, \quad 0 < \gamma _{ 2 } < 1, \quad K _{ \gamma } ^{ 1 } > 0, \quad K _{ \gamma } ^{ 2 } > 0 \\
& \ell _1 ^{ v } > \frac { 3 ^{ \frac { \gamma _{ 1 } - 1 } { 2 } } \lambda _{ max } ( \overline { L } ) } { 2 ^{ \frac { \gamma _{ 1 } + 3 } { 2 } } K _{ \gamma } ^{ 1 } { ( \lambda _{ min } ( \overline { L } ) ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } } , \quad \ell _2 ^{ v } > \frac { \lambda _{ max } ( \overline { L } ) } { 2 ^{ \frac { \gamma _{ 2 } + 3 } { 2 } } K _{ \gamma } ^{ 1 } { ( \lambda _{ min } ( \overline { L } ) ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } } \\
& \ell _1 ^{ p } > \frac { 3 ^{ \frac { \gamma _{ 1 } - 1 } { 2 } } \lambda _{ max } ( \overline { L } ) } { 2 ^{ \frac { \gamma _{ 1 } + 3 } { 2 } } K _{ \gamma } ^{ 2 } { ( \lambda _{ min } ( \overline { L } ) ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } } , \quad \ell _2 ^{ p } > \frac { \lambda _{ max } ( \overline { L } ) } { 2 ^{ \frac { \gamma _{ 2 } + 3 } { 2 } } K _{ \gamma } ^{ 2 } { ( \lambda _{ min } ( \overline { L } ) ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } }
\end{cases}
\end{aligned}
$$<br/>

the distributed desired velocity observation of the $i$-th follower can be reached as $v _i ^{ d } \to v _{ 0 } + \dot { \delta } _{ i }$ in a practical fixed time upper bounded given by

$$
\begin{aligned}
T _O ^{ V } \le \overline { T } _O ^{ V } = \frac { 2 } { l _1 ^{ V } \eta _O ^{ V } ( \gamma _{ 1 } - 1 ) } + \frac { 2 } { l _2 ^{ V } \eta _O ^{ V } ( 1 - \gamma _{ 2 } ) }
\end{aligned}
\quad\quad(3.1)
$$<br/>

where $l _1 ^{ V } = 2 ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \gamma _{ 1 } } { 2 } } \ell _1 ^{ v } K _{ \gamma } ^{ 1 } { ( \lambda _{ \min } ( \overline { L } ) ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } - \lambda _{ \min } ( \overline { L } ) / 4$ and $l _2 ^{ V } = 2 ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } \ell _2 ^{ v } K _{ \gamma } ^{ 1 } { ( \lambda _{ \min } ( \overline { L } ) ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } - \lambda _{ \min } ( \overline { L } ) / 4$. <br/>

Moreover, when the stability of velocity observation is already reached after practical fixed time $\overline { T } _O ^{ V }$, the desired position observation of $i$-th follower will be reached in practical fixed time after achieving the velocity observation, and the convergence time is upper bounded by

$$
\begin{aligned}
T _O ^{ P } \le \overline { T } _O ^{ V } + \overline { T } _O ^{ P }, \quad \overline { T } _O ^{ P } = \frac { 2 } { l _1 ^{ P } \eta _O ^{ P } ( \gamma _{ 1 } - 1 ) } + \frac { 2 } { l _2 ^{ P } \eta _O ^{ P } ( 1 - \gamma _{ 2 } ) }
\end{aligned}
\quad\quad(3.2)
$$<br/>

where $l _1 ^{ P } = 2 ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } 3 ^{ \frac { 1 - \gamma _{ 1 } } { 2 } } \ell _1 ^{ p } K _{ \gamma } ^{ 2 } { ( \lambda _{ \min } ( \overline { L } ) ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } - \lambda _{ \min } ( \overline { L } ) / 4$ and $l _2 ^{ P } = 2 ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } \ell _2 ^{ p } K _{ \gamma } ^{ 2 } { ( \lambda _{ \min } ( \overline { L } ) ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } - \lambda _{ \min } ( \overline { L } ) / 4$. <br/> <br/>

Consider the nonlinear smooth function $\lambda _{ k } ( x _{ k }, \gamma )$ employed in $\vartheta ( x _{ k }, \alpha, \gamma )$ given in **Lemma 4** from the Appendix, it can be deduced from $\lambda _{ k } ( x _{ k }, \gamma ) = -1 + \frac { 2 } { 1 + exp( - \gamma x _{ k } ) }$ that the larger $\gamma$ we select, the increasing rate near zero $\lambda _{ k }$ will hold. Since $\lambda _{ k }$ approaches 1 when $x _{ k }$ moves away from the origin, we can assume that the function $y = x _{ k } / \overline { \epsilon }$ intersects with $y = \lambda _{ k } ( x _{ k }, \gamma )$ at $( x _{ k }, y ) = ( \pm \overline { \epsilon }, 1 )$. <br/>

It can be yielded from the function property that when $\gamma \uparrow$, the value of $\overline { \epsilon } \downarrow$ and the value of $\underline { \epsilon } \downarrow$. $K _{ \alpha } = \min \lbrace 1 / \overline { \epsilon } \uparrow, \underline { \epsilon } \rbrace$ should be larger according to Eq.(3.1) and Eq.(3.2), and thus we should choose a relatively larger $\gamma$ for the nonlinear smooth function $y = \lambda _{ k } ( x _{ k }, \gamma )$. <br/>

To maintain the lower settling time upper bound, namely, $\overline { T } _O ^{ V } + \overline { T } _O ^{ P } \downarrow$, we are required to design the parameters according to the tuning principle as $\gamma _{ 1 } \uparrow$, $\gamma _{ 2 } \downarrow$ and $\gamma _{ 2 } \in ( 0, 1 )$, $\ell _1 ^{ p } \uparrow$, $\ell _2 ^{ p } \uparrow$, $\ell _1 ^{ v } \uparrow$, $\ell _2 ^{ v } \uparrow$. Since the aforementioned parameters for the observers are relevant to the final residual set that the observation error $e _{ i,p } ^{ d }$ and $e _{ i,v } ^{ d }$ will finally converge into, the width of the residual set is required to reduce to a relatively lower level. **It is required to achieve a balance between the lower settling time and the lower residual set width when tuning the parameters for the PFxTDSO.** <br/>


