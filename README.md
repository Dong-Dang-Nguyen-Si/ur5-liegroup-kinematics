\# UR5 Numerical Inverse Kinematics Solver



A Python implementation of numerical inverse kinematics (IK) for the \*\*Universal Robots UR5\*\* manipulator using the \*\*Product of Exponentials (PoE)\*\* formulation and \*\*Lie group theory\*\*, based on the framework introduced in \*Modern Robotics: Mechanics, Planning, and Control\* (Lynch \& Park).



\---



\## 📌 Project Overview



This repository demonstrates the kinematic modeling and body-frame numerical inverse kinematics resolution for a 6-DOF UR5 robotic arm. It implements a Newton-Raphson iterative solver with multi-start random initialization to guarantee robust convergence towards the target end-effector pose, even under challenging initial configurations.



\---

&#x20;Key Features



\* \*\*Product of Exponentials (PoE) Modeling:\*\*

&#x20; \* Defines the home configuration matrix $M \\in SE(3)$ and spatial screw axes ($S$-list).

&#x20; \* Converts spatial screw axes to body screw axes ($B$-list) using the adjoint map $\[\\text{Ad}\_{T\_{bs}}]$.



\* \*\*Body-Frame Newton-Raphson IK (`IKinBodyIterates`):\*\*

&#x20; \* Iteratively drives error twist $\\mathcal{V}\_b = \\left\[ \\log(T\_{sb}^{-1} T\_{sd}) \\right]^\\vee \\in se(3)$ towards zero.

&#x20; \* Employs the Moore-Penrose pseudoinverse of the body Jacobian ($J\_b^\\dagger$) for numerical joint updates:

&#x20;   $$\\Delta\\theta = J\_b^\\dagger(\\theta) \\mathcal{V}\_b$$

&#x20; \* Strictly bounds error convergence through separate angular tolerance ($e\_\\omega \\le 10^{-3} \\text{ rad}$) and linear tolerance ($e\_v \\le 10^{-3} \\text{ m}$).



\* \*\*Random-Restart Convergence Scheme:\*\*

&#x20; \* Mitigates divergence and local minima/singularities by generating random initial joint guesses $\\theta\_0 \\sim \\mathcal{U}(-\\pi, \\pi)^6$.

&#x20; \* Ensures termination within a bounded number of restarts.



\* \*\*Trajectory Logging:\*\*

&#x20; \* Records the entire convergence history of joint angles across iterations and exports the dataset directly to `iterates.csv` for post-processing and motion simulation.



\---



\## 🛠️ Tech Stack \& Requirements



\* \*\*Language:\*\* Python 3.8+

\* \*\*Dependencies:\*\*

&#x20; \* `numpy`

&#x20; \* `modern\_robotics`

