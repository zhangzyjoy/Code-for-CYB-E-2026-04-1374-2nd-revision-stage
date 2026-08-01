# 2. $\text{  }$ Nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC) <br/>

**Note for content organization :** <br/> 

This markdown file **< README - Part 2 - 3.md >** includes the parameter settings and validation of the proposed method in **Chapter 2.3**. <br/>

Please refer to **< README - Part 2 - 1.md >** to find the Lyapunov-based theoretical instruction for rotational control scheme design according to the stability proof in **Chapter 2.1**. <br/> 

Refer to **< README - Part 2 - 2.md >** for the implementation procedure of the proposed control scheme in **Chapter 2.2**. <br/> 

Refer to **< README - Part 2 - 4.md >** for comparative simulation studies in **Chapter 2.4** against some representitive methods published recently in IEEE Transactions. <br/>

## 2.3 $\text{  }$ Parameter settings and validation <br/>

Utilizing the theoretical deduction in the previous README markdown file **<README - Part 2 - 1.md>**, we can infer that **the rotational error states $\varpi _i ^e$ and $\psi _i ^e$** can be driven to converge into its origin in fixed time, and further obtain the time upper bound through **Lemmas provided in Appendix 1**. <br/>

**<1> Parameter settings for torque input in reaching phase**

In reaching phase, the Lyapunov candidate for the sliding surface $S _i$ to enter the sliding phase is chosen as $V _i ^S = { ( S _{ i } ) } ^T \Lambda _{ i } S _{ i } / 2$.

$$
\begin{aligned}
{ \dot { V } _i ^S } \le { - c _i ^{ \varpi, 1 } l _i ^{ S,1 } { ( V _i ^S ) } ^{ \frac { \beta _{ \varpi } ^1 + 1 } { 2 } } - c _i ^{ \varpi, 2 } l _i ^{ S,2 } { ( V _i ^S ) } ^{ \frac { \beta _{ \varpi } ^2 + 1 } { 2 } } + 1 / { 2 { ( \overline { \eta } _i ^S ) } ^2 } }
\end{aligned}
$$<br/>

As the aforementioned inequality holds, $\varpi _i ^e$ and $\psi _i ^e$ will be driven into the sliding surface $S _i = 0$ by the applied torque input $\tau _i$ depicted by **Eq.(2.6)** with a nonlinear compensation term by **Eq.(2.5)** in fixed time. <br/>

According to **Lemma 2** and **Eq.(1.9)**, the parameters of the rotational torque control input are chosen as $\tau _i$ as $c _i ^{ \varpi, 1 } > 0$, $c _i ^{ \varpi, 2 } > 0$, $\beta _{ \varpi } ^1 > 1$, $0 < \beta _{ \varpi } ^2 < 1$. <br/>

According to **Lemma 2**, the upper bound of the settling time for the practical fixed-time stable **PCNSMS** $S _i$ in **reaching phase** is indicated as

$$
\begin{aligned}
T _r ^{ \varpi } \le \overline { T } _r ^{ \varpi } = \frac { { ( \lambda _{max} ( \Lambda _i ) ) } ^ { \frac { \beta _{ \varpi } ^1 + 1 } { 2 } } } { 2 ^{ \frac { \beta _{ \varpi } ^1 - 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ \varpi } ^1 } { 2 } } c _i ^{ \varpi, 1 } K _i ^{ \varpi } \eta _c ^{ \varpi } ( \beta _{ \varpi } ^1 - 1 ) } + \frac { { ( \lambda _{max} ( \Lambda _i ) ) } ^ { \frac { \beta _{ \varpi } ^2 + 1 } { 2 } } } { 2 ^{ \frac { \beta _{ \varpi } ^2 - 1 } { 2 } } c _i ^{ \varpi, 2 } K _i ^{ \varpi } \eta _c ^{ \varpi } ( 1 - \beta _{ \varpi } ^2 ) }
\end{aligned}
\quad\quad(3.1)
$$<br/>

where $K _i ^{ \varpi } > 0$ and $\eta _c ^{ \varpi } > 0$ hold. **The rotational error states $\varpi _i ^e$ and $\psi _i ^e$ can converge into and remain along the sliding mode surface $S _i = 0$ after the reaching phase within fixed time upper bounded by $T _r ^{ \varpi } \le \overline { T } _r ^{ \varpi }$**. <br/>

**<2> Parameter settings for nonsingular sliding mode surface in sliding phase**

The **auxiliary nonsingular sliding mode surface** is defined for any $i \in \mathsf { \mathcal { V } }$ and $k \in \lbrace x,y,z \rbrace$ as **Eq.(2.1)**. The PCNSMS is denoted by **Eq.(2.3)** with a nonlinear piecewise auxiliary rotational error by **Eq.(2.2)**. The Lyapunov candidate for the stability proof of exponential rotational error $\psi _i ^e$ is defined as $V _i ^{ \psi } = { { \lVert \overline { \psi } _i ^e \rVert } ^2 } = \sum \nolimits _{ k = x,y,z} { { ( \overline { \psi } _{ i,k } ^e ) } ^2 }$ with definition of $\overline { \psi } _{ i,k } ^e = { \psi _{ i,k } ^{ e } } / { \pi }$. Since ${ \varpi } _{i,k} ^e = - c _i ^S \Phi ( \psi _{i,k} ^e )$ is yielded when rotaional errors are maintained along the sliding surface $S _i = 0$, the Lyapunov derivative satisfies that <br/>

$$
\begin{aligned}
\dot { V } _i ^{ \psi } \le -c _i ^S \frac { { ( V _i ^{ \psi } ) } ^{ ( p + 2 ) / 2 } } { \tanh ( \gamma _{ \psi } { ( V _i ^{ \psi } ) } ^{ 1 / 2 } / 2 ) }
\end{aligned}
$$<br/>

which indicates the fixed-time stability of the exponential rotational error $\psi _i ^e$ during the sliding phase according to **Lemma 3**. The upper bound of the settling time for $\psi _i ^e$ fixed-time convergence on the sliding mode surface is depicted as

$$
\begin{aligned}
\begin{cases}
{ T _c ^{ \varpi } } \le { \overline { T } _c ^{ \varpi } } = { \overline { T } _d ^{ \varpi } + \overline { T } _r ^{ \varpi } + \overline { T } _S ^{ \varpi } } \\
{ T _S ^{ \varpi } } \le { \overline { T } _S ^{ \varpi } } = { { ( \frac { 2 } { \gamma _{ \psi } } ) } ^{ p + 2 } \frac { 1 } { c _i ^{ S } p ( 1 - p ) } }
\end{cases}
\end{aligned}
\quad\quad(3.2)
$$<br/>

where the settling time upper bound for disturbance observation is denoted as

$$
\begin{aligned}
{ \overline { T } _d ^{ \varpi } } = { \frac { 3 ^ { \frac { \alpha _1 ^{ \varpi } - 1 } { 2 } } } { 2 ^ { \frac { \alpha _1 ^{ \varpi } - 1 } { 2 }  } c _i ^{ \varpi, 1 } K _{ \alpha } ^{ d, \varpi } ( \alpha _1 ^{ \varpi } - 1 ) } + \frac { 1 } { 2 ^ { \frac { \alpha _2 ^{ \varpi } - 1 } { 2 }  } c _i ^{ \varpi, 2 } K _{ \alpha } ^{ d, \varpi } ( 1 - \alpha _2 ^{ \varpi } ) } }
\end{aligned}
\quad\quad(3.3)
$$<br/>

Consider the nonlinear smooth function $\lambda _{ k } ( x _{ k }, \gamma )$ employed in $\vartheta ( x _{ k }, \alpha, \gamma )$ given in **Lemma 4** from the Appendix, it can be deduced from $\lambda _{ k } ( x _{ k }, \gamma ) = -1 + \frac { 2 } { 1 + exp( - \gamma x _{ k } ) }$ that the larger $\gamma$ we select, the increasing rate near zero $\lambda _{ k }$ will hold. Since $\lambda _{ k }$ approaches 1 when $x _{ k }$ moves away from the origin, we can assume that the function $y = x _{ k } / \overline { \epsilon }$ intersects with $y = \lambda _{ k } ( x _{ k }, \gamma )$ at $( x _{ k }, y ) = ( \pm \overline { \epsilon }, 1 )$. <br/>

It can be yielded from the function property that when $\gamma \uparrow$, the value of $\overline { \epsilon } \downarrow$ and the value of $\underline { \epsilon } \downarrow$. $K _{ \alpha } = \min \lbrace 1 / \overline { \epsilon } \uparrow, \underline { \epsilon } \rbrace$ should be larger according to Eq.(3.1) and Eq.(3.2), and thus we should choose a relatively larger $\gamma$ for the nonlinear smooth function $y = \lambda _{ k } ( x _{ k }, \gamma )$. <br/>

To maintain an 

