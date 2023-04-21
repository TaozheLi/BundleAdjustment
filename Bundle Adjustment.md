# Bundle Adjustment
From my perspective, the target of Bundle Adjustment is just trying to reduce error between prediction and observation. From perspective of optimization, our goal is to minimum function $e = z- h(T, P)$ ,  $T$ denotes translation matrix and $P$ denotes real world coordinate system.  Before jumping into optimization. we are about to review whole process of real world point projecting to image plane and write down optimization function.
 ## from real world point $P$ to Image plane $z$
 Assuming there exist a camera pose $Ti$ and time stamp $i$ and  a real world point $P_j$. Process as following:
 - From real world coordinate system to robot system by $P^{'}_{j} = R_{i} P_j + t_i$ or $P^{'}_j = T_i P_j$,  <font color = red>but realizing that $P_j$ in first formula with 3 dimensions and another one is 4 dimensions.
 - From robot system to image plane by Pinhole Model and Distortion Model. More specified,  $z^{prediction}_{ij} = KP^{'}_{j}$, <font color=red> remember to normalize and take distortion model (if necessary)  before taking  formula. </color>  

After these 2 steps, we acquire a prediction of real world point $P_j$ at camera pose $T_i$.Assuming there exists a corresponding observation $z^{observation}_{ij}$, we could define one error $e_{ij} = z^{obseravtion}_{ij} - z^{prediction}_{ij}$ Thus optimization function could be written as follows at situation with lots of observations.
$$
argmin \, \underset{*T_i, P_j} {Error} = \frac{1}{2}\sum_i\sum_j(e_{ij})^{2} = \frac{1}{2}\sum_i\sum_j(z^{obseravtion}_{ij} - z^{prediction}_{ij})^{2}
$$

## Optimization
Our target is to optimize aforementioned function. Thus, differential of $T$ and $P$ needed  to be computed so that we could take iterative method, such as Gaussian Method and LM method, to find solutions.

But first, we will compute $\frac{\partial e_{ij}}{\partial T_{i}}$ and $\frac{\partial e_{ij}}{\partial P_{j}}$ . 
$$
\frac{\partial e_{ij}}{\partial T_{i}} = \frac{\partial (z^{observation}_{ij} - z^{prediction}_{ij})}{\partial T_i} = - \frac{\partial z^{prediction}_{ij}}{\partial T_i} = - \frac{\partial KP^{'}_j}{\partial T_i}  = -  \frac{\partial KP^{'}_j}{\partial P^{'}_j} * \ \frac{\partial P^{'}_j}{\partial T_i}
\\
\frac{\partial KP^{'}_j}{\partial P^{'}_j} = \left [ \begin{matrix} \frac{\partial u_j}{\partial P^{'}_j} \\ \frac{\partial v_j}{\partial P^{'}_j}  \end{matrix} \right ] = \left [ \begin{matrix} \frac{\partial u_j}{\partial X^{'}_j}   \frac{\partial u_j}{\partial Y^{'}_j}  \frac{\partial u_j}{\partial Z^{'}_j} \\ \frac{\partial v_j}{\partial X_j}  \frac{\partial v_j}{\partial Y_j}  \frac{\partial v_j}{\partial Z_j}  \end{matrix} \right ] = \left [ \begin{matrix}  \frac{f_x}{Z} ,\, 0 \, ,-\frac{f_x  * \, X}{Z^2} \\ 0 \, ,\frac{f_y}{Z} \, , -\frac{f_y*Y}{Z^2} \end{matrix} \right ]_{2 *3}
\\
\frac {\partial P^{'}_j}{\partial T_i} = \frac {\partial T_i P_j}{\partial T_i} = \left [ \begin{matrix} I, \, -(R_iP_j + t_j)^{skew \, matrix} \\ 0^T \;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\ 0^T\end{matrix} \right ]_{4*6} 
\\
after \; drop \;last\; dimension:\frac {\partial P^{'}_j}{\partial T_i} = \frac {\partial T_i P_j}{\partial T_i} = \left [ \begin{matrix} I, \, -(R_iP_j + t_j)^{skew \, matrix}  \end{matrix} \right  ]_{3*6} 
\\
finally: \frac{\partial e_{ij}}{\partial T_{i}}= -\frac{\partial KP^{'}_j}{\partial P^{'}_j}  \frac {\partial P^{'}_j}{\partial T_i}_{2*6}
$$
Next one
$$
\frac{\partial e_{ij}}{\partial P_{j}} = -  \frac{\partial KP^{'}_j}{\partial P^{'}_j} * \ \frac{\partial P^{'}_j}{\partial P_j} = \frac{\partial KP^{'}_j}{\partial P^{'}_j} * R_i
$$


