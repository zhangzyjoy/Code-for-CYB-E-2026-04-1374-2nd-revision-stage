# 3. $\text{  }$ Nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC)<br/>

According to the rotational dynamics in Eq.(A.6), define an exponential coordination $\psi _i ^e = [ \Psi ( R ( Q _i ^e) ) ] _{\vee}$ which is derived through inverse logarithm mapping. We establish the rotational system error dynamics as

$$
\begin{aligned}
\begin{cases}
&\dot \psi _i ^e = \varpi _i ^e \\
&\dot \varpi _i ^e = \Lambda _i ^{-1} ( -( \varpi _i ) _\times \Lambda _i \varpi _i + \tau _i ) + ( \varpi _i ^e ) _\times ( R ( Q _i ^e ) ) ^T \varpi _i ^e - ( R ( Q _i ^e ) ) ^T \dot \varpi _i ^c
\end{cases}
\end{aligned}
\quad\quad(1.1)$$<br/>

## 3.1 $\text{  }$ Theories and design principles <br/>

To avoid the **ambiguity** of the **logarithmic mapping** on the **boundary of singular spherical neighborhood** $\partial \Omega _{\pi} = \lbrace \psi _i ^e \in \mathbb{R} ^{3} \mid \lVert \psi _i ^e \rVert = \pi \rbrace$, a specific mechanism is developed to maintain $\lVert \psi _i ^e \rVert < \pi$ under the **initial condition** of $\lVert \psi _i ^e ( 0 ) \rVert < \pi$. <br/>

A **nonsingular piecewise continuous sliding mode surface** is defined in the form as <br/>

$$
\begin{aligned}
S _i = \varpi _i ^e + c _i ^S \Phi ( \psi _i ^e )
\end{aligned}
\quad\quad(1.2)
$$<br/>

where $c _i ^S > 0$ is a constant, $\Phi ( \psi _i ^e ) = [ \Phi ( \psi _{i,x} ^e ), \Phi ( \psi _{i,y} ^e ), \Phi ( \psi _{i,z} ^e ) ] ^T$ is a piecewise differentiable auxiliary rotational error and its derivative is written as $\overline { \Phi } ( \psi _i ^e )$. <br/>

Substitute the angular velocity dynamics in Eq.(1.1) into the derivative of the nonsingular sliding mode surface Eq.(1.2) and yield <br/>

$$
\begin{aligned}
\dot { S } _i & = \dot { \varpi } _i ^e + c _i ^S \overline { \Phi } ( \psi _i ^e ) \\
& = \Lambda _i ^{-1} ( -( \varpi _i ) _\times \Lambda _i \varpi _i + \tau _i ) + ( \varpi _i ^e ) _\times ( R ( Q _i ^e ) ) ^T \varpi _i ^e - ( R ( Q _i ^e ) ) ^T \dot \varpi _i ^c + c _i ^S \overline { \Phi } ( \psi _i ^e )
\end{aligned}
\quad\quad(1.3)
$$<br/>

### 3.1.1 $\text{  }$ Rotational control torque design for the reaching phase <br/>

First we consider the **reaching phase** of the sliding surface, the objective is to drive the **piecewise continuous nonsingular sliding mode surface (PCNSMS)** ${ S } _i$ towards its origin, and thus the system state errors ${ \varpi } _i ^e$ and ${ \Phi } ( \psi _i ^e )$ can be restrained on the sliding mode surface ${ S } _i = 0$. Define a Lyapunov candidate for the reaching phase ${ S } _i$ as $V _i ^S = \frac {1} {2} ( S _i )^T \Lambda _i S _i$ and substitute Eq.(1.3) into its derivative as <br/>

$$
\begin{aligned}
\dot { V } _i ^S & = ( S _i )^T \Lambda _i \dot { S } _i = ( S _i )^T \tau _i + ( S _i )^T F _i ^S
\end{aligned}
\quad\quad(1.4)
$$<br/>

where $F _i ^S = F _i ^{ S,1 } + F _i ^{ S,2 }$ with $F _i ^{ S,1 }$ and $F _i ^{ S,2 }$ are the redundant nonlinear terms from the angular velocity error dynamics **Eq.(1.1)** and a linear term of auxiliary rotational error, respectively.

$$
\begin{aligned}
\begin{cases}
& F _i ^{ S,1 } = -( \varpi _i ) _\times \Lambda _i \varpi _i + \Lambda _i ( \varpi _i ^e ) _\times ( R ( Q _i ^e ) ) ^T \varpi _i ^e - \Lambda _i ( R ( Q _i ^e ) ) ^T \dot \varpi _i ^c \\
& F _i ^{ S,2 } = c _i ^S \Lambda _i \overline { \Phi } ( \psi _i ^e )
\end{cases}
\end{aligned}
$$<br/>

For **Eq.(1.4)**, leverage **Young's inequality** and yield <br/>

$$
\begin{aligned}
( S _i )^T F _i ^S \le \frac {1} { 2 { ( \overline { \eta } _i ^S ) } ^2 } + \frac {1} {2} ( \overline { \eta } _i ^S ) ^2 \lVert F _i ^S \rVert ^ 2 \lVert S _i \rVert ^ 2
\end{aligned}
$$<br/>

where ${ \eta } _i ^S$ is a positive constant. <br/>

Design the applied torque as $\tau _i = \overset {\frown} { \tau } _i - \frac {1} {2} ( \overline { \eta } _i ^S ) ^2 \lVert F _i ^S \rVert ^ 2 \lVert S _i \rVert ^ 2$. The Lyapunov function for **reaching phase** of **PCNSMS** is bounded by $\dot { V } _i ^S \le ( S _i ) ^T \overset { \frown } { \tau } _i + 1 / { 2 { ( \overline { \eta } _i ^S ) } ^2 }$. <br/>

According to **Lemma 2**, if we set $c_0 = 1 / { 2 { ( \overline { \eta } _i ^S ) } ^2 }$, then $( S _i )^T \overset {\frown} { \tau } _i$ should be upper bounded by $- a _1 { ( { V } _i ^S ) } ^ { b _1 } - a _2 { ( { V } _i ^S ) } ^ { b _2 }$. Set constant parameters as $l _i ^{ S,1 } > 0$, $l _i ^{ S,2 } > 0$, $\beta _{ \varpi } ^1 > 1$, $0 < \beta _{ \varpi } ^2 < 1$. It is easy to obtain $( \beta _{ \varpi } ^1 + 1 ) / 2 > 1$ and $0 < ( \beta _{ \varpi } ^2 + 1 ) / 2 < 1$. Leveraging **Lemma 5**, the upper bound of $( S _i )^T \vartheta ( S _i, \beta _{ \varpi } ^1, \mu _c ^{ \Theta } )$ and $( S _i )^T \vartheta ( S _i, \beta _{ \varpi } ^2, \mu _c ^{ \Theta } )$ is yielded as <br/>

$$
\begin{aligned}
\begin{cases}
-( S _i )^T \vartheta ( S _i, \beta _{ \varpi } ^1, \mu _c ^{ \Theta } ) & \le - 2 ^{ \frac { \beta _{ \varpi } ^1 + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ \varpi } ^1 } { 2 } } K _i ^{ \varpi } { ( ( ( S _i )^T S _i ) / 2 ) } ^{ \frac { \beta _{ \varpi } ^1 + 1 } { 2 } } \\
& \le - 2 ^{ \frac { \beta _{ \varpi } ^1 + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ \varpi } ^1 } { 2 } } K _i ^{ \varpi } { ( ( ( S _i )^T \Lambda _i S _i ) / 2 ) } ^{ \frac { \beta _{ \varpi } ^1 + 1 } { 2 } } / { ( \lambda _{max} ( \Lambda _i ) ) } ^ { \frac { \beta _{ \varpi } ^1 + 1 } { 2 } } \\
-( S _i )^T \vartheta ( S _i, \beta _{ \varpi } ^2, \mu _c ^{ \Theta } ) & \le - 2 ^{ \frac { \beta _{ \varpi } ^2 + 1 } { 2 } } K _i ^{ \varpi } { ( ( ( S _i )^T S _i ) / 2 ) } ^{ \frac { \beta _{ \varpi } ^2 + 1 } { 2 } } \\
& \le - 2 ^{ \frac { \beta _{ \varpi } ^2 + 1 } { 2 } } K _i ^{ \varpi } { ( ( ( S _i )^T \Lambda _i S _i ) / 2 ) } ^{ \frac { \beta _{ \varpi } ^2 + 1 } { 2 } } / { ( \lambda _{max} ( \Lambda _i ) ) } ^ { \frac { \beta _{ \varpi } ^2 + 1 } { 2 } }
\end{cases}
\end{aligned}
$$<br/>

Since Lyapunov function is defined as $V _i ^S = \frac {1} {2} ( S _i )^T \Lambda _i S _i$, and thus yield

$$
\begin{aligned}
\begin{cases}
-( S _i )^T \vartheta ( S _i, \beta _{ \varpi } ^1, \mu _c ^{ \Theta } ) & \le - l _i ^{ S,1 } { ( V _i ^S ) } ^{ \frac { \beta _{ \varpi } ^1 + 1 } { 2 } } \\
-( S _i )^T \vartheta ( S _i, \beta _{ \varpi } ^2, \mu _c ^{ \Theta } )
& \le - l _i ^{ S,2 } { ( V _i ^S ) } ^{ \frac { \beta _{ \varpi } ^2 + 1 } { 2 } }
\end{cases}
\end{aligned}
\quad\quad(1.5)
$$<br/>

where the coefficients are depicted as

$$
\begin{aligned}
\begin{cases}
& l _i ^{ S, 1 } = 2 ^{ \frac { \beta _{ \varpi } ^1 + 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ \varpi } ^1 } { 2 } } K _i ^{ \varpi } / { ( \lambda _{max} ( \Lambda _i ) ) } ^ { \frac { \beta _{ \varpi } ^1 + 1 } { 2 } } \\
& l _i ^{ S, 2 } = 2 ^{ \frac { \beta _{ \varpi } ^2 + 1 } { 2 } } K _i ^{ \varpi } / { ( \lambda _{max} ( \Lambda _i ) ) } ^ { \frac { \beta _{ \varpi } ^2 + 1 } { 2 } }
\end{cases}
\end{aligned}
\quad\quad(1.6)
$$<br/>

Therefore, an appropriate design for auxiliary torque input $\overset {\frown} { \tau } _i$ is to satisfy

$$
\begin{aligned}
( S _i ) ^T \overset {\frown} { \tau } _i = - c _i ^{ \varpi, 1 } l _i ^{ S, 1 } ( S _i )^T \vartheta ( S _i, \beta _{ \varpi } ^1, \mu _c ^{ \Theta } ) - c _i ^{ \varpi, 2 } l _i ^{ S, 2 } ( S _i )^T \vartheta ( S _i, \beta _{ \varpi } ^2, \mu _c ^{ \Theta } )
\end{aligned}
\quad\quad(1.7)
$$<br/>

Given constant parameters that satisfy $c _i ^{ \varpi, 1 } > 0$, $c _i ^{ \varpi, 2 } > 0$, the auxiliary torque control input is developed as

$$
\begin{aligned}
\overset {\frown} { \tau } _i = - c _i ^{ \varpi, 1 } l _i ^{ S,1 } \vartheta ( S _i, \beta _{ \varpi } ^1, \mu _c ^{ \Theta } ) - c _i ^{ \varpi, 2 } l _i ^{ S,2 } \vartheta ( S _i, \beta _{ \varpi } ^2, \mu _c ^{ \Theta } )
\end{aligned}
\quad\quad(1.8)
$$<br/>

We obtain the **auxiliary torque** $\overset {\frown} { \tau } _i$ with an additive term $- c _i ^{ \varpi, 3 } S _i$ on the basis of Eq.(1.8). Therefore, a semi-definite negative term $- c _i ^{ \varpi, 3 } \lVert S _i \rVert ^2$ is introduced. Leveraging **Eq.(1.6)** and **Eq.(1.7)**, with the inequalities given in **Eq.(1.5)**, the **Lyapunov derivative** $\dot { V } _i ^S$ is further derived as

$$
\begin{aligned}
\dot { V } _i ^S & \le ( S _i )^T \overset {\frown} { \tau } _i + 1 / { 2 { ( \overline { \eta } _i ^S ) } ^2 } - c _i ^{ \varpi, 3 } \lVert S _i \rVert ^2 \\
& \le ( S _i )^T \overset {\frown} { \tau } _i + 1 / { 2 { ( \overline { \eta } _i ^S ) } ^2 } \\
& \le - c _i ^{ \varpi, 1 } l _i ^{ S,1 } { ( V _i ^S ) } ^{ \frac { \beta _{ \varpi } ^1 + 1 } { 2 } } - c _i ^{ \varpi, 2 } l _i ^{ S,2 } { ( V _i ^S ) } ^{ \frac { \beta _{ \varpi } ^2 + 1 } { 2 } } + 1 / { 2 { ( \overline { \eta } _i ^S ) } ^2 }
\end{aligned}
\quad\quad(1.9)
$$<br/>

According to **Eq.(1.9)** and **Lemma 2**, **PCNSMS** $S _i$ can achieve **practical fixed-time stability** during the **reaching phase** only if the coefficients are bounded by $c _i ^{ \varpi, 1 } > 0$, $c _i ^{ \varpi, 2 } > 0$, $\beta _{ \varpi } ^1 > 1$, $0 < \beta _{ \varpi } ^2 < 1$. <br/>

Therefore, the rotational torque input is developed as

$$
\begin{aligned}
{ \tau } _i = & - c _i ^{ \varpi, 1 } l _i ^{ S,1 } \vartheta ( S _i, \beta _{ \varpi } ^1, \mu _c ^{ \Theta } ) - c _i ^{ \varpi, 2 } l _i ^{ S,2 } \vartheta ( S _i, \beta _{ \varpi } ^2, \mu _c ^{ \Theta } ) \\
& - c _i ^{ \varpi, 3 } S _i - \frac {1} {2} ( \overline { \eta } _i ^S ) ^2 \lVert F _i ^S \rVert ^ 2 \lVert S _i \rVert ^ 2
\end{aligned}
\quad\quad(1.10)
$$<br/>


### 3.1.2 Nonsingular sliding mode surface design for sliding phase <br/>

During the **reaching phase**, the **PCNSMS** converges to zero, and thus the angular velocity error ${ \varpi } _i ^e$ and the exponential rotational error $\psi _i ^e$ are constrained along the sliding surface $S _i = 0$ within a practical fixed time, namely ${ \varpi } _{i,k} ^e = - c _i ^S \Phi ( \psi _{i,k} ^e )$ for any $k \in \lbrace x, y, z \rbrace$. <br/>

To guarantee notational simplicity, an **auxiliary rotational error variable** is depicted as $\overline { \psi } _{ i } ^e = { \psi _ { i } ^e } / { \pi }$ with each entries to be $\overline { \psi } _{ i,k } ^e = { \psi _ { i,k } ^e } / { \pi }$ . Similarly, an **auxiliary angular velocity error** is denoted as $\overline \varpi _{ i, k } ^e = { \varpi } _{ i, k } ^e / \pi$. The **PCNSMS** $S _i$ is defined as a piecewise continuous function, and an auxiliary sliding mode surface $\overline S _i$ is considered to determine the threshold of the piecewise conditions for the function ${ \Phi } ( \psi _i ^e )$. <br/>

Considering the **sliding phase**, the rotational errors are restricted on the sliding surface $S _i = 0$. Meanwhile, $\overline S _i = 0$ always holds when the sliding mode surface $S _i = 0$ is reached, and ${ \varpi } _{i,k} ^e = - c _i ^S \Phi ( \psi _{i,k} ^e )$ is maintained during the **sliding phase**. To investigate the fixed-time stability of the **exponential rotational coordination** ${ \varpi } _{i,k} ^e$, a Lyapunov candidate is defined as 

$$
\begin{aligned}
V _i ^{ \psi } = \lVert \overline { \psi } _i ^e \rVert ^2 = \sum \nolimits _{ k = x,y,z} { { ( \overline { \psi } _{ i,k } ^e ) } ^2 }
\end{aligned}
\quad\quad(1.11)
$$<br/>

Take the derivative of Eq.(1.11) and obtain $\dot { V } _i ^{ \psi } = \sum \nolimits _{ k = x,y,z } { ( 2 { \overline { \psi } _{ i,k } ^{ e } } { \overline { \varpi } _{ i,k } ^{ e } } ) }$. According to **Lemma 3**, with parameters bounded by $0 < p < 1$ and ${ \gamma } _{ \psi }$, the following inequality

$$
\begin{aligned}
\dot { V } _i ^{ \psi } \le -c _i ^S \frac { { ( V _i ^{ \psi } ) } ^{ ( p + 2 ) / 2 } } { \tanh ( \gamma _{ \psi } { ( V _i ^{ \psi } ) } ^{ 1 / 2 } / 2 ) }
\end{aligned}
\quad\quad(1.12)
$$<br/>

is required to fulfill the **fixed-time** stability property of $\overline { \psi } _{ i,k } ^e$, that is, ${ \psi } _{ i,k } ^e = 0$ is reached in fixed time. <br/>

Since ${ \sum \nolimits _{ k = x,y,z } { ( \overline { \psi } _{ i,k } ^e ) } ^2 } \le { { ( \sum \nolimits _{ k = x,y,z } \lvert { \overline { \psi } _{ i,k } ^e } \rvert ) } ^2 }$ always holds for each $i$-th UAV and $k$-th dimention, then we can yield the following inequality

$$
\begin{aligned}
{ { ( V _i ^{ \psi } ) } ^{ 1 / 2 } } = { { ( { \sum \nolimits _{ k = x,y,z } { { ( \overline { \psi } _{ i,k } ^e ) } ^2 } } ) } ^{ 1 / 2 } } \le { { ( { ( { \sum \nolimits _{ k = x,y,z } \lvert { \overline { \psi } _{ i,k } ^e } \rvert } ) ^2 } ) } ^{ 1 / 2 } }
\end{aligned}
\quad\quad(1.13)
$$<br/>

Given that for any $x \in { \mathbb { R } }$, the nonlinear function $f (x) = \frac { x ^ { p + 2 } } { \tanh ( { \gamma } x / 2 ) }$ is monotonically increasing. Therefore, according to Eq.(1.13) that ${ { ( V _i ^{ \psi } ) } ^{ 1 / 2 } } \le { \sum \nolimits _{ k = x,y,z } \lvert { \overline { \psi } _{ i,k } ^e } \rvert }$, the Lyapunov function is further formulated as

$$
\begin{aligned}
{ - c _i ^{ S } { \frac { { ( V _i ^{ \psi } ) } ^{ ( p + 2 ) / 2 } } { \tanh ( { \gamma } _{ \psi } { ( V _i ^{ \psi } ) } ^{ 1 / 2 } / 2 ) } } } \ge { - c _i ^{ S } { \frac { { ( \sum \nolimits _{ k = x,y,z } \lvert { \overline { \psi } _{ i,k } ^e } \rvert ) } ^{ p + 2 } } { \tanh ( \gamma _{ \psi } \sum \nolimits _{ k = x,y,z } \lvert { \overline { \psi } _{ i,k } ^e } \rvert / 2 ) } } }
\end{aligned}
\quad\quad(1.14)
$$<br/>

In order to guarantee fixed-time convergence of ${ \psi } _{ i,k } ^e$, Eq.(1.12) is required to be guaranteed. It can be deduced that Eq.(3.12) can be established by proving 

$$
\begin{aligned}
{ \dot { V } _i ^{ \psi } } \le { - c _i ^{ S } { \frac { { ( \sum \nolimits _{ k = x,y,z } \lvert { \overline { \psi } _{ i,k } ^e } \rvert ) } ^{ p + 2 } } { \tanh ( \gamma _{ \psi } \sum \nolimits _{ k = x,y,z } \lvert { \overline { \psi } _{ i,k } ^e } \rvert / 2 ) } } }
\end{aligned}
\quad\quad(1.15)
$$<br/>

If Eq.(1.15) and Eq.(1.14) already hold, then Eq.(1.12) can be yielded. <br/>

Given the inequalities as ${ \sum \nolimits _{ k = x,y,z } { { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } ^{ p + 2 } } } \ge { { { ( \sum \nolimits _{ k = x,y,z } { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } ) } ^{ p + 2 } } / { 3 ^{ p + 1 } } }$ and ${ \tanh ( { \gamma } _{ \psi } \sum \nolimits _{ k = x,y,z } { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } / 2 ) } \ge { \tanh ( { \gamma } _{ \psi } { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } / 2 ) }$, we can subsequently derive

$$
\begin{aligned}
{ \frac { { ( \sum \nolimits _{ k = x,y,z } { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } ) } ^{ p + 2 } } { \tanh ( \gamma _{ \psi } \sum \nolimits _{ k = x,y,z } \lvert { \overline { \psi } _{ i,k } ^e } \rvert / 2 ) } } & \le { 3 ^{ p + 1 } { \frac { \sum \nolimits _{ k = x,y,z } { { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } ^{ p + 2 } } } { \tanh ( \gamma _{ \psi } \sum \nolimits _{ k = x,y,z } \lvert { \overline { \psi } _{ i,k } ^e } \rvert / 2 ) } } } \\
& \le { 3 ^{ p + 1 } { \frac { \sum \nolimits _{ k = x,y,z } { { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } ^{ p + 2 } } } { \tanh ( \gamma _{ \psi } \lvert { \overline { \psi } _{ i,k } ^e } \rvert / 2 ) } } } \\
& = { 3 ^{ p + 1 } { \sum \nolimits _{ k = x,y,z } { \frac { { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } ^{ p + 2 } } { \tanh ( \gamma _{ \psi } \lvert { \overline { \psi } _{ i,k } ^e } \rvert / 2 ) } } } }
\end{aligned}
\quad\quad(1.16)
$$<br/>

Since $\overline { \psi } _{ i,k } ^e$ and $\tanh ( \gamma _{ \psi } \sum \nolimits _{ k = x,y,z } { \overline { \psi } _{ i,k } ^e } / 2 )$ have the same sign, it is easy to derive that $\frac { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } { \tanh ( \gamma _{ \psi } \lvert { \overline { \psi } _{ i,k } ^e } \rvert / 2 ) } = \frac { \overline { \psi } _{ i,k } ^e } { \tanh ( \gamma _{ \psi } { \overline { \psi } _{ i,k } ^e } / 2 ) }$. Then Eq.(1.16) can be further transformed into <br/>

$$
\begin{aligned}
{ 3 ^{ p + 1 } { \sum \nolimits _{ k = x,y,z } { \frac { { { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } ^{ p + 1 } } { \overline { \psi } _{ i,k } ^e } } { \tanh ( \gamma _{ \psi } { \overline { \psi } _{ i,k } ^e } / 2 ) } } } } \ge { \frac { { ( \sum \nolimits _{ k = x,y,z } { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } ) } ^{ p + 2 } } { \tanh ( \gamma _{ \psi } \sum \nolimits _{ k = x,y,z } \lvert { \overline { \psi } _{ i,k } ^e } \rvert / 2 ) } }
\end{aligned}
\quad\quad(1.17)
$$<br/>

Hence, it suffices to show that if

$$
\begin{aligned}
{ \dot { V } _i ^{ \psi } } \le { - c ^i _S { 3 ^{ p + 1 } } { \sum \nolimits _{ k = x,y,z } { \frac { { { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } ^{ p + 1 } } { \overline { \psi } _{ i,k } ^e } } { \tanh ( \gamma _{ \psi } { \overline { \psi } _{ i,k } ^e } / 2 ) } } } }
\end{aligned}
\quad\quad(1.18)
$$<br/>

holds, then Eq.(1.15) is established. According to Eq.(1.14), Eq.(1.15) is sufficient for that Eq.(1.12) holds. <br/>

Considering **Lemma 3**, we can derive that **Eq.(1.12)** holds can prove that ${ \psi } _{ i,k } ^e$ is fixed-time stable during the **sliding phase** when the nonsingular sliding mode surface already converges to $S _i = 0$ after the **reaching phase**. <br/>

**Therefore, it can be obtained that Eq.(1.17) is sufficient for the proof that ${ \psi } _{ i,k } ^e$ can reach fixed-time stability on sliding phase.** <br/>

During the **sliding phase**, ${ \varpi } _{ i,k } ^e = - c _i ^S \Phi ( \psi _{i,k} ^e )$ can be yielded, and the Lyapunov derivative $\dot { V } _i ^{ \psi } = \sum \nolimits _{ k = x,y,z } { ( 2 { \overline { \psi } _{ i,k } ^e } { \overline { \varpi } _{ i,k } ^e } ) } = - \sum \nolimits _{ k = x,y,z } { { c _i ^S } { \overline { \psi } _{ i,k } ^e } { \overline { \Phi } ( \psi _{i,k} ^e ) } }$ is developed to satisfy the form of Eq.(1.17). The **auxiliary rotational error** is deduced from $\overline { \Phi } ( \psi _{i,k} ^e ) = { \Phi } ( \psi _{i,k} ^e ) / { \pi }$ as

$$
\begin{aligned}
{ \Phi ( \psi _{i,k} ^e ) } & = { { \pi } { \overline { \Phi } ( \psi _{i,k} ^e ) } } \\ 
& = { 3 ^{ p + 1 } { \frac { { \pi } c ^i _S } { c ^i _S { \overline { \psi } _{ i,k } ^e } } \frac { { { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } ^{ p + 1 } } { \overline { \psi } _{ i,k } ^e } } { \tanh ( \gamma _{ \psi } { \overline { \psi } _{ i,k } ^e } / 2 ) } } } \\
& = { 3 ^{ p + 1 } { { \pi } ^{ -p } } { \lvert { { \psi } _{ i,k } ^e } \rvert } ^{ p + 1 } / { ( 2 \tanh ( \gamma _{ \psi } \overline { \psi } _{ i,k } ^e / 2 ) ) } }
\end{aligned}
\quad\quad(1.19)
$$<br/>

If the **sliding phase** is reached with $S _i = 0$ and the rotational errors are driven along the nonsingular sliding mode surface, the **auxiliary nonlinear sliding mode surface** is designed as

$$
\begin{aligned}
{ \overline { S } _{ i,k } } & = { { \varpi } _{ i,k } ^e + 3 ^{ p + 1 } c _i ^S \frac { { { \pi } ^{ -p } } { \lvert { { \psi } _{ i,k } ^e } \rvert } ^{ p + 1 } } { 2 \tanh ( \gamma _{ \psi } \overline { \psi } _{ i,k } ^e / 2 ) } }
\end{aligned}
\quad\quad(1.20)
$$<br/>

Moreover, if the auxiliary sliding surface $\overline S _{i,k} = 0$ holds, then the PCNSMS are consistent with the form of $\overline S _{i,k}$ as

$$
\begin{aligned}
{ S _{ i,k } } & = { { \varpi } _{ i,k } ^e + 3 ^{ p + 1 } c _i ^S \frac { { { \pi } ^{ -p } } { \lvert { { \psi } _{ i,k } ^e } \rvert } ^{ p + 1 } } { 2 \tanh ( \gamma _{ \psi } \overline { \psi } _{ i,k } ^e / 2 ) } } \quad\quad \text{  if } \overline S _{i,k} = 0
\end{aligned}
\quad\quad(1.21)
$$<br/>

when the rotational errors are remained on the **sliding phase**. <br/>

We investigate **PCNSMS** $S _i$ to follow **Eq.(1.21)**, the same form with the auxiliary sliding surface, to guarantee **Eq.(1.12)** and to thereafter achieve the fixed-time stable property of $\psi _i ^e$. Moreover, the **auxiliary nonsingular sliding mode surface** $\overline { S } _i$ is established as the switching threshold to determine the **piecewise function** $\Phi ( \psi _{i,k} ^e )$ in PCNSMS $S _i$. <br/> <br/>


### 3.1.2 $\text{  }$ Nonsingular sliding mode surface design for reaching phase <br/>

On the **reaching phase**, we already developed an applied torque $\tau _i$ rotational control input to maintain a **practical fixed-time stable** $S _i$ **PCNSMS** derived from **Eq.(1.4) to Eq.(1.10)** inspired by **Lemma 2**. <br/>

On the **sliding phase**, a **fixed-time stable** $\psi _i ^e = { [ \Psi ( R ( Q _i ^e ) ) ] _{ \vee } }$ **exponential rotational error** can be achieved by developing nonsingular sliding mode surface according to **Eq.(1.11) to Eq.(1.20)** inspired by **Lemma 3**. <br/>

During the **sliding phase**, $S _i$ and $\overline { S } _i$ are determined according to **Eq.(1.20) and Eq.(1.21)**, respectively. Then the Lyapunov candidate of exponential rotational error is defined as $V _i ^S = { { \lVert \overline { \psi } _i ^e \rVert } ^2 } = { \sum \nolimits _{ k = x,y,z } { ( \overline { \psi } _i ^e ) ^2 } }$. Since the angular velocity is depicted by ${ \varpi } _{ i,k } ^e = S _i - c _i ^S { \Phi ( { \psi } _{ i,k } ^e ) }$, the derivative of $V _i ^S$ is given as <br/>

$$
\begin{aligned}
{ \dot { V } _i ^{ \psi } } & = { \sum \nolimits _{ k = x,y,z } { ( 2 { \overline { \psi } _{ i,k } ^e } { \overline { \varpi } _{ i,k } ^e } ) } } \\
& = { \sum \nolimits _{ k = x,y,z } ( 2 { \overline { \psi } _{ i,k } ^e } { ( S _{ i,k } - c _i ^S { \Phi ( { \psi } _{ i,k } ^e ) } ) } / { \pi } ) }
\end{aligned}
\quad\quad(1.22)
$$<br/>

Given the assumption that the initial rotational error satisfies $\lVert { {\psi} _i ^e ( 0 ) } \rVert < \pi$, namely, for any $k \in \lbrace x,y,z \rbrace$ that $\lvert \frac { \psi _{ i,k } ^e ( 0 ) } { \pi } \rvert < 1$ holds. The PCNSMS satisfies $\lVert S _i ( t ) \rVert < + \infty$ and is upper bounded by $\underset { i \in { \mathsf { \mathcal { V } } }, k \in { \lbrace x,y,z \rbrace } } { \mathop { \max } } \lbrace { \lvert { S _{ i,k } } \rvert } \rbrace \le \overline { S } _M$. <br/>

Consider the first term $2 { \frac { { \psi } _{ i,k } ^e } { \pi } } { \frac { S _i } { \pi } }$ of the Lyapunov derivative $\dot { V } _i ^{ \psi }$ in Eq.(1.22), we can derive that ${ 2 { \frac { { \psi } _{ i,k } ^e } { \pi } } { \frac { S _i } { \pi } } } \le { 2 { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } { \lvert \frac { S _{ i,k } } { \pi } \rvert } } < { \frac { 2 } { \pi } \lvert { S _{ i,k } } \rvert } \le { \frac { 2 } { \pi } \overline { S } _M }$, and thereafter obtain

$$
\begin{aligned}
{ \sum \nolimits _{ k = x,y,z } { ( 2 { \frac { { \psi } _{ i,k } ^e } { \pi } } { \frac { S _i } { \pi } } ) } } \le { \frac { 6 } { \pi } { \overline { S } _M } }
\end{aligned}
\quad\quad(1.23)
$$<br/>

The specific design for PCNSMS on sliding phase has been illustrated in chapter 3.1.2. On reaching phase, $S _{ i,k } \neq 0$ and $\overline S _{ i,k } \neq 0$ holds. The switching threshold for piecewise auxiliary nonsingular sliding mode surface $\overline { S } _{ i,k } \neq 0$ on reaching phase is denoted as $\lvert { \psi _{ i,k } ^e } \rvert = { \varepsilon } _{ i,k } ^S$. <br/>

First we assume that the PCNSMS $S _i$ is defined as Eq.(1.21) even in partial piecewise intervals on reaching phase. Substitute the auxiliary rotational error Eq.(1.19) on sliding phase into Eq.(1.22) and yield

$$
\begin{aligned}
{ \dot { V } _i ^{ \psi } } & = { \sum \nolimits _{ k = x,y,z } ( 2 { \overline { \psi } _{ i,k } ^e } { ( \frac { S _{ i,k } } { \pi } - 3 ^{ p + 1 } c _i ^S \frac { { \lvert { \overline { \psi } _{ i,k } ^e } \rvert } ^{ p + 1 } } { 2 \tanh ( \gamma _{ \psi } { \overline { \psi } _{ i,k } ^e } / 2 ) } ) } ) } \\
& = { \sum \nolimits _{ k = x,y,z } ( 2 \frac { { \psi } _{ i,k } ^e } { \pi } \frac { S _{ i,k } } { \pi } - 3 ^{ p + 1 } c _i ^S \frac { { \lvert { \frac { { \psi } _{ i,k } ^e } { \pi } } \rvert } ^{ p + 2 } } { \tanh ( \gamma _{ \psi } { \lvert \frac { { \psi } _{ i,k } ^e } { \pi } \rvert } / 2 ) } ) }
\end{aligned}
\quad\quad(1.24)
$$<br/>

Since $\tanh ( \gamma _{ \psi } { \lvert \frac { { \psi } _{ i,k } ^e } { \pi } \rvert } / 2 ) \in ( 0,1 )$ holds, then we can derive

$$
\begin{aligned}
{ - 3 ^{ p + 1 } c _i ^S \sum \nolimits _{ k = x,y,z } { \frac { { \lvert { \frac { { \psi } _{ i,k } ^e } { \pi } } \rvert } ^{ p + 2 } } { \tanh ( \gamma _{ \psi } { \lvert \frac { { \psi } _{ i,k } ^e } { \pi } \rvert } / 2 ) } } } \le { - 3 ^{ p + 1 } c _i ^S \sum \nolimits _{ k = x,y,z } { { \lvert \frac { { \psi } _{ i,k } ^e } { \pi } \rvert } ^{ p + 2 } } }
\end{aligned}
\quad\quad(1.25)
$$<br/>

For $p \in (0,1)$, $1 < p + 1 < 2$ holds. The initial condition guarantees $\lvert { \psi _{ i,k } ^e (0) } \rvert < \pi$ and then we can obtain

$$
\begin{aligned}
{ { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^{ p + 2 } } \ge { { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^{ 3 } } \ge { { ( { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^2 ) } ^2 }
\end{aligned}
$$<br/>

It can be further deduced that 

$$
\begin{aligned}
{ \sum \nolimits _{ k = x,y,z } { { ( { \lvert \frac { { \psi } _{ i,k } ^e } { \pi } \rvert } ^2 ) } ^2 } } \ge { \frac { 1 } { 3 } { { ( \sum \nolimits _{ k = x,y,z } { { \lvert \frac { { \psi } _{ i,k } ^e } { \pi } \rvert } ^2 } ) } ^2 } }
\end{aligned}
$$<br/>

By summing over $k \in \lbrace x,y,z \rbrace$, it can be yielded that 

$$
\begin{aligned}
{ - 3 ^{ p + 1 } c _i ^S \sum \nolimits _{ k = x,y,z } { { \lvert \frac { { \psi } _{ i,k } ^e } { \pi } \rvert } ^{ p + 2 } } } \le { - 3 ^{ p + 1 } c _i ^S \sum \nolimits _{ k = x,y,z } { { ( { { \lvert \frac { { \psi } _{ i,k } ^e } { \pi } \rvert } ^2 } ) } ^2 } } = { - 3 ^p c _i ^S { \lVert \frac { { \psi } _{ i } ^e } { \pi } \rVert } ^4 }
\end{aligned}
\quad\quad(1.26)
$$<br/>

Substitute Eq.(1.25) and Eq.(1.26) into Eq.(1.24) and yield

$$
\begin{aligned}
{ \dot { V } _i ^{ \psi } } \le { \frac { 6 } { \pi } { \overline { S } _M } - 3 ^p c _i ^S { { \lVert \frac { { \psi } _{ i } ^e } { \pi } \rVert } ^4 } }
\end{aligned}
\quad\quad(1.27)
$$<br/>

Therefore, we obtain the **convergence region** for $\psi _i ^e$ during the first stage of the reaching phase as

$$
\begin{aligned}
{ \Omega _{ \psi } } = { \lbrace \psi _i ^e \mid 0 \le { \lVert { \psi } _{ i } ^e \rVert } \le { { ( { { \frac { 2 { \pi } ^3 } { c _i ^S } } 3 ^{ 1 - p } \overline { S } _M } ) } ^{ \frac { 1 } { 4 } } } \rbrace }
\end{aligned}
\quad\quad(1.28)
$$<br/>

The above deduction implies that $\lVert { \psi } _{ i } ^e \rVert > { ( { \frac { 2 { \pi } ^3 } { c _i ^S } } 3 ^{ 1 - p } \overline { S } _M ) } ^{ \frac { 1 } { 4 } }$, and thus $\dot { V } _i ^{ \psi } < 0$ is obtained given that the the exponential rotational error $\psi _i ^e$ falls outside the residual set denoted by Eq.(1.28). <br/>

**Once $\psi _i ^e$ falls outside Eq.(1.28), it will evolve along the direction of the negative gradient of the Lyapunov function, which finally drives it to enter the convergence region denoted by Eq.(1.28).** Under this consideration, the PCNSMS is developed as Eq.(1.21) to guarantee the exponential rotational error term to converge into the neighbor set Eq.(1.28). <br/>

Simultaneously, **the PCNSMS $S _i$** is driven by **the torque input depicted by Eq.(1.10)** to converge into its origin in a practical fixed time during the **reaching phase**. We can deduce that the width of the convergence region denoted by Eq.(1.28) will decrease as the upper bound of the PCNSMS $\overline { S } _M \ge \underset { i \in { \mathsf { \mathcal { V } } }, k \in { \lbrace x,y,z \rbrace } } { \mathop { \max } } \lbrace { \lvert { S _{ i,k } } \rvert } \rbrace$ declines during the reaching phase. Subsequently, the upper bound $\overline { S } _M$ of the sliding surface declines towards zero simultaneously with the PCNSMS before achieving ${ ( { \frac { 2 { \pi } ^3 } { c _i ^S } } 3 ^{ 1 - p } \overline { S } _M ) } ^{ \frac { 1 } { 4 } } < { \varepsilon } _{ i,k } ^S$. Eq.(1.21) can always be considered as $S _i$ under the condition that ${ \overline { S } _{ i,k } } \neq 0, \text{  } { \varepsilon } _{ i,k } ^S \le \lVert \psi _i ^e \rVert < \pi$ holds. <br/>

If the condition ${ ( { \frac { 2 { \pi } ^3 } { c _i ^S } } 3 ^{ 1 - p } \overline { S } _M ) } ^{ \frac { 1 } { 4 } } < { \varepsilon } _{ i,k } ^S$ is fulfilled in reaching phase, the designed PCNSMS should guarantee that Eq.(1.27) holds under the condition that ${ \overline { S } _{ i,k } } \neq 0, \text{  } \lVert \psi _i ^e \rVert < { \varepsilon } _{ i,k } ^S$ because Eq.(1.27) guarantees that $\psi _i ^e$ can be driven into the Eq.(1.28) convergence region. <br/>

Under the condition ${ \overline { S } _{ i,k } } \neq 0, \text{  } { \varepsilon } _{ i,k } ^S \le \lVert \psi _i ^e \rVert < \pi$, the auxiliary rotational error to be designed is denoted as $\Phi _k ( \psi _{ i,k } ^e ) = \Phi _k ^1 ( \psi _{ i,k } ^e ) + \Phi _k ^2 ( \psi _{ i,k } ^e )$. The Lyapunov derivative is further denoted as 

$$
\begin{aligned}
{ \dot { V } _i ^{ \psi } } = { 2 \sum \nolimits _{ k = x,y,z } { ( \frac { \psi _{ i,k } ^e } { \pi } ) } } \le \frac { 6 } { \pi } \overline { S } _M - 2 \sum \nolimits _{ k = x,y,z } { ( \frac { \psi _{ i,k } ^e } { \pi } \frac { c _i ^S } { \pi } { ( \Phi _k ^1 ( \psi _{ i,k } ^e ) + \Phi _k ^2 ( \psi _{ i,k } ^e ) ) } ) }
\end{aligned}
\quad\quad(1.29)
$$<br/>

According to **Lemma 4**, it can be derived that

$$
\begin{aligned}
{ \sum \nolimits _{ k = x,y,z } { \lvert \frac { \psi _i ^e } { \pi } \rvert } ^4 } = { \sum \nolimits _{ k = x,y,z } { { ( { \lvert \frac { \psi _i ^e } { \pi } \rvert } ^2 ) } ^2 } } \ge { \frac { 1 } { 3 } ( \sum \nolimits _{ k = x,y,z } { \lvert \frac { \psi _i ^e } { \pi } \rvert } ^2 ) ^2 } = { \frac { 1 } { 3 } \lVert \frac { \psi _i ^e } { \pi } \rVert ^4 }
\end{aligned}
$$<br/>

Therefore, the latter term in Eq.(1.28), $- 3 ^p c _i ^S { { \lVert \frac { { \psi } _{ i } ^e } { \pi } \rVert } ^4 }$ , is partially formulated as

$$
\begin{aligned}
{ - \frac { 2 } { 3 } c _i ^S { \lVert \frac { \psi _i ^e } { \pi } \rVert } ^4 } \ge - 2 { c _i ^S } \sum \nolimits _{ k = x,y,z } { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert ^4 } = -2 \sum \nolimits _{ k = x,y,z } { ( { \frac { \psi _{ i,k } ^e } { \pi } } { \frac { c _i ^S } { \pi } } ( { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^{ 2 } \psi _{ i,k } ^e ) ) }
\end{aligned}
\quad\quad(1.30)
$$<br/>

According to Eq.(1.30), in order to obtain $- \frac { 2 } { 3 } c _i ^S { \lVert \frac { \psi _i ^e } { \pi } \rVert } ^4$ in the Lyapunov derivative, it can be deduced from Eq.(1.30) that

$$
\begin{aligned}
{ -2 { \frac { c _i ^S } { \pi } } \sum \nolimits _{ k = x,y,z } { ( { \frac { \psi _{ i,k } ^e } { \pi } } { \beta _{ \Phi } ^{ 1 } } ( { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^{ 2 } \psi _{ i,k } ^{ e } ) ) } } \le { - { \frac { 2 } { 3 } } { c _i ^S } { \beta _{ \Phi } ^{ 1 } } { \lVert \frac { \psi _i ^e } { \pi } \rVert } ^{ 4 } }
\end{aligned}
\quad\quad(1.31)
$$<br/>

Therefore, the first term of auxiliary rotational error in Eq.(1.29) is defined as 

$$
\begin{aligned}
{ \Phi _k ^{ 1 } ( \psi _{ i,k } ^e ) } = { { \beta _{ \Phi } ^{ 1 } } { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^{ 2 } \psi _{ i,k } ^e }
\end{aligned}
\quad\quad(1.32)
$$<br/>


According to **Lemma 4**, given that $p \in (0,1)$ and $1 < p + 1 < 2$, then we can derive that

$$
\begin{aligned}
{ - { \lVert \frac { \psi _{ i,k } ^e } { \pi } \rVert } ^4 } = - { { ( \sum \nolimits _{ k = x,y,z } { { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^2 } ) } ^2 } \ge { - { ( \sum \nolimits _{ k = x,y,z } { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^{ p + 1 } ) } ^2 }, \quad \quad { \frac { 1 } { 3 } { { ( \sum \nolimits _{ k = x,y,z } { { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^{ p + 1 } } ) } ^2 } } \le { \sum \nolimits _{ k = x,y,z } { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^{ 2 ( p + 1 ) } }
\end{aligned}
$$<br/>

and we further obtain that $\frac { 1 } { 3 } { { ( \sum \nolimits _{ k = x,y,z } { { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^{ p + 1 } } ) } ^2 } \le \sum \nolimits _{ k = x,y,z } { { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^{ 2 ( p + 1) } }$. Similar as Eq.(1.31), the second term of auxiliary rotational error in Eq.(1.29) is bounded by

$$
\begin{aligned}
{ - { \frac { 2 } { 3 } } c _i ^S { \beta _{ \Phi } ^{ 2 } } { \lVert \frac { \psi _i ^e } { \pi } \rVert } ^{ 4 } } \ge { - 2 { c _i ^S } { \beta _{ \Phi } ^{ 2 } } \sum \nolimits _{ k = x,y,z } { { \lvert { \frac { \psi _{ i,k } ^e } { \pi } } \rvert } ^{ 2 ( p + 1) } } } \ge { - 2 { \frac { c _i ^S } { \pi } } \sum \nolimits _{ k = x,y,z } { \frac { \psi _{ i,k } ^e } { \pi } { \Phi _k ^2 ( \psi _{ i,k } ^e ) } } }
\end{aligned}
$$<br/>

Consider the last inequality in the above expression as an equality, and then we derive the second term of auxiliary rotational error as

$$
\begin{aligned}
{ \Phi _k ^2 ( \psi _{ i,k } ^e ) } = { \beta _{ \Phi } ^2 { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^{ 2p } \psi _{ i,k } ^e }
\end{aligned}
\quad\quad(1.33)
$$<br/>

Subsequently, when the parameter scope exists as ${ \overline { S } _{ i,k } } \neq 0, \text{  } { \varepsilon } _{ i,k } ^S \le \lVert \psi _i ^e \rVert < \pi$, the PCNSMS is developed as

$$
\begin{aligned}
{ S _{ i,k } } = { \varpi } _{ i,k } ^e + c _i ^S { \psi _{ i,k } ^e } ( \beta _{ \Phi } ^{ 1 } { { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^2 } + \beta _{ \Phi } ^{ 2 } { { \lvert \frac { \psi _{ i,k } ^e } { \pi } \rvert } ^{ 2p } } )
\end{aligned}
\quad\quad(1.34)
$$<br/>

**Note : Please refer to <README - Part 2-2.md> for the rest of the content.** <br/>


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

**Lemma 3 (Proposed Fixed-Time Stability Criteria)** : If a radially unbounded positive definite function $V ( t , x )$ satisfies $\dot V \le -k \frac { V ^{ ( p + 2 ) / 2 } } { \tanh ( \gamma V ^{ 1 / 2 } ) }$ such that $k > 0$, $\gamma > 0$, $0 < p < 1$, then the origin of $x ( t )$ converges in fixed time upper bounded by

$$
\begin{aligned}
T _s \le \overline T _s = \frac { 2 ^{ p + 2 } } { \gamma ^{ p + 2 } k p ( 1 - p ) }
\end{aligned}
$$<br/>

**Lemma 4** : Consider a sequence with positive scalars $q _1, \text{ } q _2, \text{ } ..., \text{ } q_N$ where $q _k \text{ } \ge \text{ } 0$ for any $k \in \lbrace 1,...,N \rbrace$. Given that $0 < \alpha _1 \le 1$, $\alpha _2 > 1$, it can be yielded that

$$
\begin{aligned}
\sum _{ k = 1 } ^{ N } { ( q _k ) ^{ \alpha _1 } } \ge { ( \sum _{ k = 1 } ^{ N } { q _k } ) } ^{ \alpha _1 }, \quad \quad \sum _{ k = 1 } ^{ N } { ( q _k ) ^{ \alpha _2 } } \ge N ^{ 1 - \alpha _2 } { ( \sum _{ k = 1 } ^{ N } { q _k } ) } ^{ \alpha _2 }
\end{aligned}
$$<br/>


## A.2 $\text{  }$ Design of nonlinear smooth sigmoid function <br/>

A newly introduced **nonlinear smooth sigmoid vector** is defined as <br/>

$$
\begin{aligned}
\vartheta ( \overline x, \alpha, \gamma ) = [ \vartheta _k ( \overline x _1, \alpha, \gamma ), ..., \vartheta _k ( \overline x _n, \alpha, \gamma ) ] ^T
\end{aligned}
\quad\quad(A.2)
$$<br/>

with each entry denoted as <br/>

$$
\begin{aligned}
\vartheta _k ( \overline x _k, \alpha, \gamma ) = \lvert x _k \rvert ^ \alpha \lambda _k ( x _k, \gamma )
\end{aligned}
\quad\quad(A.3)
$$<br/>

$\lambda _k ( x _k, \gamma )$ is a sigmoid-like bounded function defined as <br/>

$$
\begin{aligned}
\lambda _k ( x _k, \gamma ) = -1 + \frac 1 {1 + \exp ( - \gamma x _k )}
\end{aligned}
\quad\quad(A.4)
$$<br/>

where $\gamma > 0$ controls the growth rate around the zero crossing. <br/>

**Lemma 5** : For a vector $\overline x = [ x _1, ..., x _n ] ^T \in \mathbb R ^n$ , $\alpha > 0$, and $\gamma > 0$, define $\vartheta ( \overline x , \alpha, \gamma ) = [ \vartheta _1 ( x _1, \alpha, \gamma ), ..., \vartheta _n ( x _n, \alpha, \gamma ) ]^T$ with each entries $\vartheta _k ( x _k, \alpha, \gamma )$ as the form of Eq.(3), the inequalities are yielded as

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
- $\Lambda_{i}$ : inertia matrix 
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
- $Q_{i} = [\rho_{i}, q_{i}^{T}]^{T} = [\rho_{i}, q_{i}^{1}, q_{i}^{2}, q_{i}^{3}]^T$ : quaternion
- $Q_{i}^{c}$, $Q_{i}^{e}$ : quaternion command, error
- $R(Q_{i})$, $R(Q_{i}^{c})$, $R(Q_{i}^{e})$ : rotation matrix, command, error
- $\varpi_{i}$, $\varpi_{i}^{c}$, $\varpi_{i}^{e}$ : angular velocity, command, error
- $\psi_i^e = [\Psi(R(Q_i^e))]_\vee$ : rotational error in Lie Algebra
- $\overline S_i$ = $[ \overline S_{i,x}, \overline S_{i,y}, \overline S_{i,z} ]^T$ : auxiliary sliding mode surface
- $\Phi(\psi_i^e) = [ \Phi_x(\psi_{i,x} ^e), \Phi_y(\psi_{i,y} ^e), \Phi_z(\psi_{i,z} ^e)]^T$ : piecewise auxiliary rotational error
- $\overline \Phi (\psi_i^e) = [ \overline \Phi _x(\psi ^e _{i,x}), \overline \Phi _y(\psi ^e _{i,y}), \overline \Phi _z(\psi ^e _{i,z})]^T$ : first derivative of piecewise auxiliary rotational error
- $F_i^S$ : rotation compensation term
- $\tau_i$ : applied torque rotational control input

