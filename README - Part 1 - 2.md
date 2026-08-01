# 1. $\text{  }$ Fixed-time rotational disturbance observer (FxTDO) <br/>

**Note for content organization :** 

This markdown file **< README - Part 1 - 2.md >** includes the parameter settings and validation of the proposed method in **Chapter 1.3**. <br/>

Please refer to **< README - Part 1 - 1.md >** for the Lyapunov-based theoretical instruction for rotational control scheme design according to the stability proof in **Chapter 1.1**, and also includes the implementation procedure of the proposed control scheme in **Chapter 1.2**. <br/>

Refer to **< README - Part 1 - 3.md >** for comparative simulation studies in **Chapter 1.4** against some representitive methods published recently in IEEE Transactions. <br/>

## 1.3 $\text{  }$ Parameter settings and validation <br/>

According to **Lemma 1**, **Eq.(1.7)** and **Eq.(1.3)**, parameter settings should guarantee $h _i ^{ \varpi, 1 } > 0$, $h _i ^{ \varpi, 2 } > 0$, $h _i ^{ \varpi, 3 } > 0$, $\mu _d ^{ \varpi } > 0$, $\alpha _1 ^{ \varpi } > 1$, $0 < \alpha _2 ^{ \varpi } < 1$. <br/>

The observation error of rotational disturbance $d _i ^{ \varpi }$ shares the same convergence performance metrices with the auxiliary angular velocity observation error $\tilde { \overline { \sigma } } _i ^{ \varpi }$. Since **Eq.(1.7)** holds and the parameter settings in **Eq.(1.3)** is satisfied, the upper bound for the settling time of $d _i ^{ \varpi }$, equivalent to that of $\tilde { \overline { \sigma } } _i ^{ \varpi }$ under fixed-time convergence is depicted from **Lemma 1** by <br/>

$$
\begin{aligned}
T _d ^{ \varpi } \le \overline { T } _d ^{ \varpi } = \frac { 3 ^ { \frac { \alpha _1 ^{ \varpi } - 1 } { 2 } } } { 2 ^ { \frac { \alpha _1 ^{ \varpi } - 1 } { 2 }  } c _i ^{ \varpi, 1 } K _{ \alpha } ^{ d, \varpi } ( \alpha _1 ^{ \varpi } - 1 ) } + \frac { 1 } { 2 ^ { \frac { \alpha _2 ^{ \varpi } - 1 } { 2 }  } c _i ^{ \varpi, 2 } K _{ \alpha } ^{ d, \varpi } ( 1 - \alpha _2 ^{ \varpi } ) }
\end{aligned}
\quad\quad(3.1)
$$<br/>

Consider the nonlinear smooth function $\lambda _{ k } ( x _{ k }, \gamma )$ employed in $\vartheta ( x _{ k }, \alpha, \gamma )$ given in **Lemma 4** from the Appendix, it can be deduced from $\lambda _{ k } ( x _{ k }, \gamma ) = -1 + \frac { 2 } { 1 + exp( - \gamma x _{ k } ) }$ that the larger $\gamma$ we select, the increasing rate near zero $\lambda _{ k }$ will hold. Since $\lambda _{ k }$ approaches 1 when $x _{ k }$ moves away from the origin, we can assume that the function $y = x _{ k } / \overline { \epsilon }$ intersects with $y = \lambda _{ k } ( x _{ k }, \gamma )$ at $( x _{ k }, y ) = ( \pm \overline { \epsilon }, 1 )$. <br/>

It can be yielded from the function property that when $\gamma \uparrow$, the value of $\overline { \epsilon } \downarrow$ and the value of $\underline { \epsilon } \downarrow$. $K _{ \alpha } = \min \lbrace 1 / \overline { \epsilon } \uparrow, \underline { \epsilon } \rbrace$ should be larger according to Eq.(3.1), and thus we should choose a relatively larger $\gamma$ for the nonlinear smooth function $y = \lambda _{ k } ( x _{ k }, \gamma )$. <br/>

To maintain the upper bound for a lower settling time for the rotational disturbance observation, the tuning principles for the observer parameters should be set as $\alpha _1 ^{ \varpi } \uparrow$, $\alpha _2 ^{ \varpi } \downarrow$, 





