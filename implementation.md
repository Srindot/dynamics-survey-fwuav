# Implementation of the Aerodynamics of Flapping Wing UAV

This document contains all the equations extracted from the reference paper: *"An Improved Quasi-Steady Model Capable of Calculating Flexible Deformation for Bird-Sized Flapping Wings"* by Tianyou Mao et al., which outline the formulation for implementing the aerodynamics of a flapping wing UAV.

## 1. Aerodynamic Solver

The core idea of the quasi-steady (QS) model is to simplify unsteady flow problems by decomposing them into a series of instantaneous steady-state problems. The instantaneous dynamic equation for each discrete time point is given by:

$$
F_{inst} = F_{tran} + F_{rot} + F_{add} + F_{iner} \quad (1)
$$
where $F_{tran}$ is the translational circulatory component, $F_{rot}$ is the rotational circulatory component, $F_{add}$ is the added mass force, and $F_{iner}$ is the inertial force.

The translational circulatory force is computed as:

$$
F_{tran} = \frac{1}{2} \rho |v_b + v_e|^2 \int_{A} C_{tran}^e dS^e \quad (2)
$$

The lift and drag coefficients of the translational circulation are obtained from empirical formulas based on the element's relative angle of attack $\alpha^e$:

$$
\begin{cases} 
C_{L,tran}^e = 1.870 \sin(1.872\alpha^e) \\
C_{D,tran}^e = 1.667 - 1.585 \cos(1.968\alpha^e)
\end{cases} \quad (3)
$$

The rotational circulation force caused by passive chordwise twist is given by:

$$
F_{rot} = \frac{1}{2} C_{rot} \rho \omega_\lambda \int_{A} \dot{\varepsilon}_c^e r_s^e r_c^e dS^e \quad (4)
$$
with
$$
\omega_\lambda = \frac{A}{2} \frac{\partial g(f, t)}{\partial t} \quad (5)
$$
and
$$
C_{rot} = 2\pi(0.75 - \hat{x}_0) \quad (6)
$$

The added mass and inertial forces are expressed as:

$$
F_{add} = \frac{\pi}{4}\rho \int_{A} (r_c^e)^3 \left[ (\dot{\omega}_\lambda \sin\alpha^e + \omega_\lambda \dot{\alpha}^e \cos\alpha^e)r_s^e - \ddot{\alpha}^e \frac{r_c^e}{4} \right] dS^e \quad (7)
$$

$$
F_{iner} = \dot{\omega}_\lambda \int_{A} r_s^e \rho_w^e B_w^e dS^e \quad (8)
$$

Through coordinate transformation, the thrust force $F_T$, lift force $F_L$, and yaw centripetal force $F_C$ can be obtained:

$$
\begin{bmatrix} F_T \\ -F_L \\ F_C \end{bmatrix} = R_y(-\alpha_0) R_x(\theta) \begin{bmatrix} 0 \\ 0 \\ F_{inst} \end{bmatrix} \quad (9)
$$

## 2. Structural Deformation Solver

The external load at the $j$-th node of the membrane element is expressed as:

$$
P_{w,j}^{ext} = \sum_{i = \{k\}} \frac{f_{i,inst}^e}{3} \quad (10)
$$

The coordinate transformation from the wing coordinate system to the generalized coordinates is given by:

$$
r_j(u_j, v_j) = J_{pr} p_j(x_j, y_j) \quad (11)
$$

The in-plane coordinate change from normal displacement $h_j$ is:

$$
\begin{cases}
u_j' = \left(1 + \frac{h_j^2}{u_j^2 + v_j^2}\right)^{1/2} u_j \\
v_j' = \left(1 + \frac{h_j^2}{u_j^2 + v_j^2}\right)^{1/2} v_j
\end{cases} \quad (12)
$$

For a three-node Constant Strain Triangle (CST) element, the shape functions are:

$$
N_i = \frac{1}{2A}(a_i + b_i u + c_i v), \quad i = 1, 2, 3 \quad (13)
$$

The strain function matrix $[B]$ is derived as:

$$
[B] = \frac{1}{2A} \begin{bmatrix} b_1 & 0 & b_2 & 0 & b_3 & 0 \\ 0 & c_1 & 0 & c_2 & 0 & c_3 \\ c_1 & b_1 & c_2 & b_2 & c_3 & b_3 \end{bmatrix} \quad (14)
$$

The elemental stiffness matrix of the wing membrane is:

$$
K_w^e = \int_{A} B^T D B dA \quad (15)
$$

The in-plane load of the membrane nodes based on the global stiffness equation:

$$
[P_w^{in}] = [K_w][\Delta r_w] \quad (16)
$$

The load component on the $Z_w$ axis:

$$
P_{w,j}^{in, z} = P_{w,j}^{in} h_j (u_j^2 + v_j^2 + h_j^2)^{-1/2} \quad (17)
$$

The total load at each wing membrane node:

$$
P_{w,j} = P_{w,j}^{ext} + P_{w,j}^{in, z} \quad (18)
$$

Contact force distribution function at the wing vein boundary:

$$
R_j(u) = \begin{cases} \vartheta_j u + \varrho_j, & 0 \leq u \leq u_j \\ \frac{\vartheta_j u_j}{u_j - u_L}(u - u_L) + \varrho_j, & u_j < u \leq u_L \end{cases} \quad (19)
$$

$$
R_j(v) = \begin{cases} \varsigma_j v + \varrho_j, & 0 \leq v \leq v_j \\ \frac{\varsigma_j v_j}{v_j - v_L}(v - v_L) + \varrho_j, & v_j < v \leq v_L \end{cases} \quad (20)
$$

The undetermined coefficients are solved via equilibrium equations:

$$
\begin{cases}
\int_{0}^{u_L} R_j(u) du + \int_{0}^{v_L} R_j(v) dv = P_{w,j} \\
\int_{0}^{u_L} u R_j(u) du = P_{w,j} v_j \\
\int_{0}^{v_L} v R_j(v) dv = P_{w,j} u_j
\end{cases} \quad (21)
$$

The load at the $i$-th wing vein node is:

$$
P_{b,i} = P_{b,i}^{iner} + \sum R_j(r) l_e \quad (22)
$$

The inertial force of the wing vein element:

$$
P_{b,i}^{iner} = \rho_c \frac{\pi d_c^2}{4} l_e y_i \frac{\partial \omega_\lambda}{\partial t} \quad (23)
$$

The beam element stiffness matrix is:

$$
K_b^e = \frac{E_c I_c}{l_e^3} \begin{bmatrix} 12 & 6l_e & -12 & 6l_e \\ 6l_e & 4l_e^2 & -6l_e & 2l_e^2 \\ -12 & -6l_e & 12 & -6l_e \\ 6l_e & 2l_e^2 & -6l_e & 4l_e^2 \end{bmatrix} \quad (24)
$$

The deformation distribution matrix of each wing vein node:

$$
[q_b] = [K_b]^{-1}[P_b] \quad (25)
$$

## 3. Interaction Variables

The normal vector of the wing element plane is obtained via the cross product:

$$
\vec{v}_n^e = \vec{p_1^e p_2^e} \times \vec{p_2^e p_3^e} \quad (26)
$$

The chordwise twist angle $\varepsilon_c^e$ of the element:

$$
\varepsilon_c^e = \frac{\pi}{2} - \arccos\left( \frac{\vec{v}_n^e \cdot \vec{g}_n}{|\vec{v}_n^e| \cdot |\vec{g}_n|} \right) \quad (27)
$$

The spanwise twist angle $\varepsilon_s^e$ of the element:

$$
\varepsilon_s^e = \frac{\pi}{2} - \arccos\left( \frac{\vec{v}_n^e \cdot \vec{b}_n}{|\vec{v}_n^e| \cdot |\vec{b}_n|} \right) \quad (28)
$$

## 4. Measurement & Vision Coordinate Transformation

Coordinate transformation from the camera coordinate system to the wing coordinate system:

$$
p_w = k_{cw} R_{cw} (p_c + T_{cw}) \quad (29)
$$

Where the projection and scaling matrix $k_{cw}$ is:

$$
k_{cw} = \text{diag}\left\{ \frac{k_\Lambda}{\cos\alpha_0}, \frac{k_\Lambda}{\cos\alpha_0} \right\} \quad (30)
$$

The rotation matrix is:

$$
R_{cw} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} \quad (31)
$$

And the translation matrix is defined as:

$$
T_{cw} = \vec{O_c O_w} \quad (32)
$$
