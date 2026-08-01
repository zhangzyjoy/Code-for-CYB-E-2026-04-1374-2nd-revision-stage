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

To maintain a lower settling time upper bound for the rotational disturbance observation, the tuning principles for the observer parameters should be set as $\alpha _1 ^{ \varpi } \uparrow$ and $\alpha _2 ^{ \varpi } \downarrow$




