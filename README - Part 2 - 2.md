## 3.2 $\text{  }$ Controller implementation procedure <br/>

For any $k \in \lbrace { x,y,z } \rbrace$, define an auxiliary sliding mode surface

$$
\begin{aligned}
\overline S _{i,k} = \varpi ^e _{i,k} + c _i ^S {\frac {3 ^{p+1} \pi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^{p+1}} {2 \tanh ( \gamma {\frac {\psi ^e _{i,k}} \pi} / 2 )} }
\end{aligned}
\quad\quad(2.1)$$<br/>

For any $k \in \lbrace x,y,z \rbrace$, define piecewise auxiliary rotational error

$$
\begin{aligned}
\Phi _k ( \psi ^e _{i,k}) = 
\begin{cases}
&\frac {3 ^{p+1} \pi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^{p+1}} {2 \tanh ( \gamma {\frac {\psi ^e _{i,k}} \pi} / 2 )} \\
& \quad\quad \text{  if } \overline S _{i,k} = 0, \text{  or  } \overline S _{i,k} \ne 0,\text{  } \lvert \psi ^e _{i,k} \rvert > \overline \varepsilon ^{S} _{i,k} \\
&\psi ^e _{i,k} ( \beta ^1 _\Phi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^2 + \beta ^2 _\Phi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^{2p} ) \\
& \quad\quad \text{  if } \overline S _{i,k} \ne 0,\text{ } \lvert \psi ^e _{i,k} \rvert \le \overline \varepsilon ^{S} _{i,k}
\end{cases}
\end{aligned}
\quad\quad(2.2)$$<br/>

Sliding mode surface

$$
\begin{aligned}
S _{i,k} = \varpi ^e _{i,k} + c _i ^S \Phi _k ( \psi ^e _{i,k})
\end{aligned}
\quad\quad(2.3)
$$<br/>

The derivative of piecewise auxiliary rotational error

$$
\begin{aligned}
\overline \Phi _k ( \psi ^e _{i,k}) = 
\begin{cases}
&\frac { 3 ^{p+1} (p+1) \frac {\psi ^e _{i,k}} \pi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^{p-1} \varpi ^e _{i,k} } {2 \tanh ( \gamma {\frac {\psi ^e _{i,k}} \pi} / 2 )} \\
&+ \frac {3 ^{p+1}} 4 \gamma \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^{p+1} (\frac { \mathrm{sech} ( \gamma \psi ^e _{i,k} / 2 \pi )} {\tanh ( \gamma \psi ^e _{i,k} / 2 \pi )} ) ^2 \varpi ^e _{i,k}, \\
& \quad\quad \text{             if } \overline S _{i,k} = 0, \text{  or  } \overline S _{i,k} \ne 0,\text{  } \lvert \psi ^e _{i,k} \rvert > \overline \varepsilon ^{S} _{i,k} \\
&( 2 \beta ^1 _\Phi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^2 + ( 2p + 1 ) \beta ^2 _\Phi \lvert \frac {\psi ^e _{i,k}} \pi \rvert ^{2p} ) \varpi ^e _{i,k}, \\
& \quad\quad \text{             if } \overline S _{i,k} \ne 0,\text{  } \lvert \psi ^e _{i,k} \rvert \le \overline \varepsilon ^{S} _{i,k} \\
\end{cases}
\end{aligned}
\quad\quad(2.4)
$$<br/>


The rotation compensation term

$$
\begin{aligned}
F _i ^S = &- ( \varpi _i ) _\times \Lambda _i \varpi _i + \Lambda _i ( \varpi _i ^e ) _\times ( R ( Q _i ^e ) ) ^T \varpi _i ^c \\
&- \Lambda _i ( R ( Q _i ^e ) ) ^T \dot \varpi _i ^c + c _i ^S \Lambda _i \overline \Phi _k ( \psi ^e _{i,k})
\end{aligned}
\quad\quad(2.5)$$<br/>

The applied torque control input

$$
\begin{aligned}
\tau _i = &- c _i ^{\varpi,1} \vartheta ( S_i, \beta ^1 _\varpi, \mu _c ^\Theta ) - c _i ^{\varpi,2} \vartheta ( S_i, \beta ^2 _\varpi, \mu _c ^\Theta ) \\
&- c _i ^{\varpi,3} S_i - \hat d _i ^\varpi - \frac 1 2 ( \overline \eta _i ^S ) ^2 \lVert F _i ^S \rVert ^2 S_i
\end{aligned}
\quad\quad(2.6)$$<br/>

where $\hat d _i ^\varpi$ is obtained by FxTDO in rotational system. <br/>

Update rotational dynamics, including angular velocity, rotational matrix and their tracking errors

$$
\begin{aligned}
\begin{cases}
&\dot \varpi _i = ( \Lambda _i ) ^{-1} ( - ( \varpi _i ) _\times \Lambda _i \varpi _i + \tau _i + d _i ^\varpi ) \\
&\dot R ( Q _i ) = R ( Q _i ) ( \varpi _i ) _\times \\
&\varpi _i ^e = \varpi _i - ( R ( Q _i ^e ) ) ^T \varpi _i ^c \\
&\dot R ( Q _i ^e ) = R ( Q _i ^e ) ( \varpi _i ^e ) _\times \\
&\dot \psi ^e _i = \varpi _i ^e
\end{cases}
\end{aligned}
\quad\quad(2.7)$$<br/>

## 3.3 $\text{  }$ Parameter settings and validation <br/>

According to **Lemma 2** and **Eq.(1.9)**, the parameters of the rotational torque control input are chosen as $\tau _i$ as $c _i ^{ \varpi, 1 } > 0$, $c _i ^{ \varpi, 2 } > 0$, $\beta _{ \varpi } ^1 > 1$, $0 < \beta _{ \varpi } ^2 < 1$. <br/>

According to **Lemma 2**, the settling time for the practical fixed-time convergence of **PCNSMS** $S _i$ is indicated as

$$
\begin{aligned}
T _r ^{ \varpi } \le \overline { T } _r ^{ \varpi } = \frac { { ( \lambda _{max} ( \Lambda _i ) ) } ^ { \frac { \beta _{ \varpi } ^1 + 1 } { 2 } } } { 2 ^{ \frac { \beta _{ \varpi } ^1 - 1 } { 2 } } 3 ^{ \frac { 1 - \beta _{ \varpi } ^1 } { 2 } } c _i ^{ \varpi, 1 } K _i ^{ \varpi } \eta _c ^{ \varpi } ( \beta _{ \varpi } ^1 - 1 ) } + \frac { { ( \lambda _{max} ( \Lambda _i ) ) } ^ { \frac { \beta _{ \varpi } ^2 + 1 } { 2 } } } { 2 ^{ \frac { \beta _{ \varpi } ^2 - 1 } { 2 } } c _i ^{ \varpi, 2 } K _i ^{ \varpi } \eta _c ^{ \varpi } ( 1 - \beta _{ \varpi } ^2 ) }
\end{aligned}
$$<br/>

where $K _i ^{ \varpi } > 0$ and $\eta _c ^{ \varpi } > 0$ hold. <br/>




## 3.4 $\text{  }$ Validation for comparison <br/>


