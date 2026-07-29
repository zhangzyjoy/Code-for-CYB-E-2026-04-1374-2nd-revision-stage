# 4. $\text{  }$ Fixed-time translational disturbance observer (FxTDO) <br/>

This markdown file **< README - Part 4 - 2.md >** includes the parameter settings and validation of the proposed disturbance observer in **Chapter 4.3**. <br/> 

Please refer to **< README - Part 4 - 2.md >** for the Lyapunov-based theoretical instruction for the disturbance observer design according to the stability proof in **Chapter 4.1**, and also for the implementation procedure of the proposed disturbance observers in **Chapter 4.2**. <br/>

Refer to **< README - Part 4 - 3.md >** for comparative simulation studies in **Chapter 4.4** against some representitive disturbance observers published recently in IEEE Transactions. <br/> <br/>


## 4.3 $\text{  }$ Parameter settings and validation

According to **Lemma 1**, **Eq.(1.7)** and **Eq.(1.3)**, parameter settings should guarantee $c _i ^{ v, 1 } > 0$, $c _i ^{ v, 2 } > 0$, $c _i ^{ v, 3 } > 0$, $\mu _d ^{ v } > 0$, $\alpha _1 ^{ v } > 1$, $0 < \alpha _2 ^{ v } < 1$. <br/>

The observation error of rotational disturbance $d _i ^{ v }$ shares the same convergence performance metrices with the auxiliary angular velocity observation error $\tilde { \overline { \sigma } } _i ^{ v }$. Since **Eq.(1.7)** holds and the parameter settings in **Eq.(1.3)** is satisfied, the upper bound for the settling time of $d _i ^{ v }$, equivalent to that of $\tilde { \overline { \sigma } } _i ^{ v }$ under fixed-time convergence is depicted from **Lemma 1** by <br/>

$$
\begin{aligned}
T _d ^{ v } \le \overline { T } _d ^{ v } = \frac { 3 ^ { \frac { \alpha _1 ^{ v } - 1 } { 2 } } } { 2 ^ { \frac { \alpha _1 ^{ v } - 1 } { 2 }  } c _i ^{ v, 1 } K _{ \alpha } ^{ d, v } ( \alpha _1 ^{ v } - 1 ) } + \frac { 1 } { 2 ^ { \frac { \alpha _2 ^{ v } - 1 } { 2 }  } c _i ^{ v, 2 } K _{ \alpha } ^{ d, v } ( 1 - \alpha _2 ^{ v } ) }
\end{aligned}
\quad\quad(3.1)$$<br/>


