# Tractor-trailer model in bicycle mode

A response from Gemini:

A kinematic model for a tractor-trailer (or truck with a single trailer) simplifies the vehicle physics by assuming **no tire slip** (pure rolling contact). This model uses the **steering angle ($\delta$)** and **longitudinal acceleration / throttle-braking input ($a$)** as the main control inputs.

By setting up the equations in continuous time, every state derivative is a smooth, continuous function of the state and control inputs, making the entire system **differentiable with respect to the inputs**.

---

### **1. System Definitions & Geometry**

Let the tractor be modeled as a standard single-track Bicycle Model, with a single trailer hinged at the rear axle of the tractor.

* **Control Inputs $\mathbf{u} = [\delta, a]^T$:**
* $\delta$: Front wheel steering angle (rad)
* $a$: Longitudinal acceleration / throttle input ($\text{m/s}^2$)


* **State Variables $\mathbf{x} = [x_0, y_0, \theta_0, v_0, \theta_1]^T$:**
* $(x_0, y_0)$: Position of the tractor's rear axle center ($\text{m}$)
* $\theta_0$: Heading angle of the tractor ($\text{rad}$)
* $v_0$: Longitudinal velocity of the tractor ($\text{m/s}$)
* $\theta_1$: Heading angle of the trailer ($\text{rad}$)


* **Geometric Constants:**
* $L_0$: Wheelbase of the tractor ($\text{m}$)
* $L_1$: Length from the tractor rear axle hitch point to the trailer axle ($\text{m}$)
* $M$: Hitch offset distance behind the tractor rear axle ($\text{m}$) *(if hitch is directly on the rear axle, $M = 0$)*



---

### **2. Kinematic Equations of Motion**

Assuming no side-slip on any of the tires, the continuous-time dynamics $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$ are defined as:

$$\begin{aligned} \dot{x}_0 &= v_0 \cos(\theta_0) \\ \dot{y}_0 &= v_0 \sin(\theta_0) \\ \dot{\theta}_0 &= \frac{v_0}{L_0} \tan(\delta) \\ \dot{v}_0 &= a \\ \dot{\theta}_1 &= \frac{v_0}{L_1} \sin(\theta_0 - \theta_1) - \frac{M}{L_1} \dot{\theta}_0 \cos(\theta_0 - \theta_1) \end{aligned}$$

#### **Simplified Trailer Angle Dynamics (Assuming $M = 0$)**

If the trailer hitch sits directly above the tractor's rear axle ($M = 0$), the hitch angle difference $\gamma = \theta_0 - \theta_1$ simplifies the trailer heading derivative to:

$$\dot{\theta}_1 = \frac{v_0}{L_1} \sin(\theta_0 - \theta_1)$$

---

### **3. Differentiability Analysis**

To use this model in gradient-based optimal control (e.g., LQR, MPC, trajectory optimization, or automatic differentiation frameworks like PyTorch/JAX), we require the Jacobian matrices $A = \frac{\partial f}{\partial \mathbf{x}}$ and $B = \frac{\partial f}{\partial \mathbf{u}}$.

The continuous formulation is fully differentiable across all smooth state spaces except near $\vert{}\delta\vert{} \to \frac{\pi}{2}$ (steering past physical mechanical limits).

#### **Input Jacobians ($B = \frac{\partial f}{\partial \mathbf{u}}$)**

$$B = \begin{bmatrix} \frac{\partial \dot{x}_0}{\partial \delta} & \frac{\partial \dot{x}_0}{\partial a} \\ \frac{\partial \dot{y}_0}{\partial \delta} & \frac{\partial \dot{y}_0}{\partial a} \\ \frac{\partial \dot{\theta}_0}{\partial \delta} & \frac{\partial \dot{\theta}_0}{\partial a} \\ \frac{\partial \dot{v}_0}{\partial \delta} & \frac{\partial \dot{v}_0}{\partial a} \\ \frac{\partial \dot{\theta}_1}{\partial \delta} & \frac{\partial \dot{\theta}_1}{\partial a} \end{bmatrix} = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ \frac{v_0}{L_0} \sec^2(\delta) & 0 \\ 0 & 1 \\ -\frac{M}{L_1 L_0} v_0 \cos(\theta_0 - \theta_1) \sec^2(\delta) & 0 \end{bmatrix}$$

When $M = 0$, $\frac{\partial \dot{\theta}_1}{\partial \delta} = 0$, making trailer rotation indirectly controlled through tractor velocity and orientation.

## Redefining the trailer angle variable

Redefining the state variables using the relative hitch angle (or joint angle) **$\gamma = \theta_0 - \theta_1$** instead of absolute trailer heading $\theta_1$ significantly simplifies the state vector dynamics, reducing trigonometric coupling across global frames.

---

### **1. State Vector & Definitions**

* **State Variables $\mathbf{x} = [x_0, y_0, \theta_0, v_0, \gamma]^T$:**
* $(x_0, y_0, \theta_0, v_0)$: Tractor position, heading, and velocity
* **$\gamma$**: Relative hitch angle ($\gamma = \theta_0 - \theta_1$)


* **Control Inputs $\mathbf{u} = [\delta, a]^T$:**
* $\delta$: Steering angle ($\text{rad}$)
* $a$: Throttle / acceleration ($\text{m/s}^2$)



---

### **2. Derivation of Relative Hitch Dynamics ($\dot{\gamma}$)**

Differentiating $\gamma = \theta_0 - \theta_1$ with respect to time:

$$\dot{\gamma} = \dot{\theta}_0 - \dot{\theta}_1$$

Substituting the expressions for $\dot{\theta}_0 = \frac{v_0}{L_0}\tan(\delta)$ and $\dot{\theta}_1 = \frac{v_0}{L_1}\sin(\gamma) + \frac{M}{L_1}\dot{\theta}_0\cos(\gamma)$:

$$\dot{\gamma} = \frac{v_0}{L_0}\tan(\delta) - \left[ \frac{v_0}{L_1}\sin(\gamma) + \frac{M}{L_1}\left(\frac{v_0}{L_0}\tan(\delta)\right)\cos(\gamma) \right]$$

Factoring out $v_0$:

$$\dot{\gamma} = v_0 \left[ \frac{\tan(\delta)}{L_0} \left(1 - \frac{M}{L_1}\cos(\gamma)\right) - \frac{\sin(\gamma)}{L_1} \right]$$

#### **Simplified Model ($M = 0$, Hitch directly on Rear Axle)**

When the hitch sits directly above the tractor's rear axle ($M = 0$), the hitch angle derivative reduces to a remarkably clean form:

$$\dot{\gamma} = v_0 \left( \frac{\tan(\delta)}{L_0} - \frac{\sin(\gamma)}{L_1} \right)$$

---

### **3. Simplified Continuous System Dynamics**

$$\begin{aligned} \dot{x}_0 &= v_0 \cos(\theta_0) \\ \dot{y}_0 &= v_0 \sin(\theta_0) \\ \dot{\theta}_0 &= \frac{v_0}{L_0} \tan(\delta) \\ \dot{v}_0 &= a \\ \dot{\gamma} &= v_0 \left( \frac{\tan(\delta)}{L_0} - \frac{\sin(\gamma)}{L_1} \right) \quad (\text{for } M=0) \end{aligned}$$

### **Why This Formulation Is Better**

1. **Independent of Global Heading:** $\dot{\gamma}$ depends only on internal states ($v_0, \gamma$) and the input $\delta$. It is completely decoupled from global absolute heading $\theta_0$ and position $(x_0, y_0)$.
2. **Cleaner Jacobians for MPC/LQR:** The linearizations simplify noticeably because $\frac{\partial \dot{\gamma}}{\partial \theta_0} = 0$.
3. **Direct Jackknife Monitoring:** The jackknife safety constraint simply becomes an uncoupled bound on a single state variable: $\vert{}\gamma\vert{} \le \gamma_{\text{max}}$.

---

### **4. Exact Input Jacobian Matrix ($B = \frac{\partial f}{\partial \mathbf{u}}$)**

For $M = 0$, the partial derivatives with respect to inputs $\mathbf{u} = [\delta, a]^T$ become:

$$B = \frac{\partial f}{\partial \mathbf{u}} =  \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ \frac{v_0}{L_0} \sec^2(\delta) & 0 \\ 0 & 1 \\ \frac{v_0}{L_0} \sec^2(\delta) & 0 \end{bmatrix}$$

Notice that $\frac{\partial \dot{\gamma}}{\partial \delta} = \frac{\partial \dot{\theta}_0}{\partial \delta}$, showing a direct linear relationship between front steering input and hitch angle rate change.