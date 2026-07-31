# 3. $\text{  }$ Practical fixed-time distributed state observer (PFxTDSO) <br/>

This markdown file **< README - Part 3 - 1.md >** includes the Lyapunov-based theoretical instruction for the distributed observer design according to the stability proof in **Chapter 3.1**, and also includes the implementation procedure of the proposed disturbance observer scheme in **Chapter 3.2**. <br/> 

Please refer to **< README - Part 3 - 2.md >** for the parameter settings and validation of the proposed observer in **Chapter 3.3**. <br/>

Refer to **< README - Part 3 - 3.md >** for comparative simulation studies in **Chapter 3.4** against some representitive observers published recently in IEEE Transactions. <br/> <br/>


## 3.3 $\text{  }$ Parameter settings and validation <br/>

The desired linear velocity $v _{ 0 } + \dot { \delta } _{ i }$ of the $i$-th uncrewed aerial vehicle is estimated by the distributed observer denoted by Eq.(2.5) in practical fixed time. According to **Lemma 2**, if we select the parameters that are constrained by

$$
\begin{aligned}
\begin{cases}
\gamma _{ 1 } & > 1, \quad 0 < \gamma _{ 2 } < 1, \quad K _{ \gamma } ^{ 1 } > 0, \quad K _{ \gamma } ^{ 2 } > 0 \\
\ell _1 ^{ v } & > \frac { 3 ^{ \frac { \gamma _{ 1 } - 1 } { 2 } } \lambda _{ max } ( \overline { L } ) } { 2 ^{ \frac { \gamma _{ 1 } + 3 } { 2 } } K _{ \gamma } ^{ 1 } { ( \lambda _{ min } ( \overline { L } ) ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } } \\
\ell _2 ^{ v } & > \frac { \lambda _{ max } ( \overline { L } ) } { 2 ^{ \frac { \gamma _{ 2 } + 3 } { 2 } } K _{ \gamma } ^{ 1 } { ( \lambda _{ min } ( \overline { L } ) ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } } \\
\ell _1 ^{ p } & > \frac { 3 ^{ \frac { \gamma _{ 1 } - 1 } { 2 } } \lambda _{ max } ( \overline { L } ) } { 2 ^{ \frac { \gamma _{ 1 } + 3 } { 2 } } K _{ \gamma } ^{ 2 } { ( \lambda _{ min } ( \overline { L } ) ) } ^{ \frac { \gamma _{ 1 } + 1 } { 2 } } } \\
\ell _2 ^{ p } & > \frac { \lambda _{ max } ( \overline { L } ) } { 2 ^{ \frac { \gamma _{ 2 } + 3 } { 2 } } K _{ \gamma } ^{ 2 } { ( \lambda _{ min } ( \overline { L } ) ) } ^{ \frac { \gamma _{ 2 } + 1 } { 2 } } }
\end{cases}
\end{aligned}
$$<br/>

the distributed desired velocity observation of the $i$-th follower can be reached as $v _i ^{ d } \to v _{ 0 } + \dot { \delta } _{ i }$ in a practical fixed time given by

$$
\begin{aligned}
T _O ^{ V } \le \overline \overline { T } _O ^{ V } =  
\end{aligned}
$$<br/>




