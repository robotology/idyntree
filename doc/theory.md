# iDynTree Theory Document

This document summarises the mathematical conventions underlying the **iDynTree** library.
It is intended to complement the API‑level documentation with a concise reference aimed at developers and researchers, suitable to be easily given in input to Large Language Models (LLMs).

> [!WARNING]
> The Markdown+LaTeX support on GitHub has several limitations . The suggested way to consume for a human this file is to open it in Visual Studio Code, and then preview it with `Ctrl+Shift+V`, for example using Visual Studio Code online via this URL: https://github.dev/robotology/idyntree/blob/main/doc/theory.md .


If you are not familiar with the topics, this document is probably not a good document for a first introduction, refer instead to this document and their references:
* [Multibody dynamics notation (version 2)](https://pure.tue.nl/ws/portalfiles/portal/139293126/A_Multibody_Dynamics_Notation_Revision_2_.pdf)
* [Modelling, Estimation and Identification of Humanoid Robots Dynamics](https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf)

---

## Basic Notation

| Symbol | Meaning |
|--------|---------|
| $A,B,\dots$ | Frames (right‑handed, orthonormal) |
| $A$ or $W$ | $A$ (absolute) or $W$ (world) typically indicates the inertial frame w.r.t. that is used as a reference for the kinematics and dynamics computations. |
| ${}^{A}o_{B}\in\mathbb R^{3}$ | Position vector of the origin of frame $B$ expressed in frame $A$ |
| ${}^{A}R_{B}\in\mathbb R^{3 \times 3}$ | Rotation matrix from frame $B$ to frame $A$ |
| ${}^{A}\overline{o}_{B}\in\mathbb R^{4}$ | Homogeneous position vector: ${}^{A}\overline{o}_{B} = \begin{bmatrix} {}^{A}o_{B} \\ 1 \end{bmatrix}$ |
| ${}^{A}H_{B}\in\mathbb R^{4 \times 4}$ | Homogeneous transformation matrix: ${}^{A}H_{B} = \begin{bmatrix} {}^{A}R_{B} & {}^{A}o_{B} \\ 0^T & 1 \end{bmatrix}$ , such that ${}^{A}\overline{o}_{C} = {}^{A}H_{B} \, {}^{B}\overline{o}_{C}$ |
| $u^{\wedge} \in \mathbb{R}^{3 \times 3}$ | Skew-symmetric matrix of 3D vector $u = \begin{bmatrix} u_x \\ u_y \\ u_z \end{bmatrix}$: $u^{\wedge} = \begin{bmatrix} 0 & -u_z & u_y \\ u_z & 0 & -u_x \\ -u_y & u_x & 0 \end{bmatrix}$ |
| $(\cdot)^{\vee}: \mathbb{R}^{3 \times 3} \to \mathbb{R}^3$ | 3D vee operator (inverse of wedge): $(u^{\wedge})^{\vee} = u$. |
| $u \times v$ | 3D cross product: $u \times v = u^{\wedge} v$ for $u, v \in \mathbb{R}^3$ |
| ${}^{A}X_B \in \mathbb{R}^{6 \times 6}$ | Transformation matrix for 6D velocities: ${}^{A}X_B = \begin{bmatrix} {}^{A}R_B & 0_{3 \times 3} \\ {}^{A}o_B^{\wedge} {}^{A}R_B & {}^{A}R_B \end{bmatrix}$ |
| ${}_{A}X^B \in \mathbb{R}^{6 \times 6}$ | Transformation matrix for 6D forces: ${}\_{A}X^B = \begin{bmatrix} {}^{A}R_B & {}^{A}o_B^{\wedge} {}^{A}R_B \\ 0_{3 \times 3} & {}^{A}R_B \end{bmatrix}$ |
| $\mathrm{v}^{\wedge} \in \mathbb{R}^{4 \times 4}$ | 4D matrix representation of 6D vector $\mathrm{v} = \begin{bmatrix} v \\ \omega \end{bmatrix}$: $\mathrm{v}^{\wedge} = \begin{bmatrix} \omega^{\wedge} & v \\ 0^T & 0 \end{bmatrix}$ |
| $(\cdot)^{\vee}: \mathbb{R}^{4 \times 4} \to \mathbb{R}^6$ | 6D vee operator (inverse of 6D wedge): $(\mathrm{v}^{\wedge})^{\vee} = \mathrm{v}$|
| $\mathrm{v} \times \mathrm{u}$ | 6D cross product: $\mathrm{v} \times \mathrm{u} = \begin{bmatrix} \omega_v^{\wedge} & v_v^{\wedge} \\ 0_{3 \times 3} & \omega_v^{\wedge} \end{bmatrix} \mathrm{u} = \begin{bmatrix} \omega_v \times u_v + v_v \times \omega_u \\ \omega_v \times \omega_u \end{bmatrix}$ for $\mathrm{v} = \begin{bmatrix} v_v \\ \omega_v \end{bmatrix}, \mathrm{u} = \begin{bmatrix} u_v \\ \omega_u \end{bmatrix}$ |


Properties:
* Rotation matrix inverse is transpose: ${}^{B}R_{A} = {}^{A}R_{B}^{-1} = {}^{A}R_{B}^T$
* Composition rule for rotations: ${}^A R_C = {}^A R_B {}^B R_C $
* Composition rule for homogeneous transf: ${}^A H_C = {}^A H_B {}^B H_C $


---

## 3D Angular Velocity Representation

| Representation | Symbol | Definition | Physical Meaning |
|----------------|--------|------------|------------------|
| **Left-trivialized** | ${}^{B}{\omega}_{A,B} \in \mathbb{R}^{3}$ | $({}^{A}R_{B}^{-1} \dot{{}^{A}R_{B}})^{\vee}$ | Angular velocity vector expressed in frame $A$ coordinates |
| **Right-trivialized** | ${}^{A}{\omega}_{A,B} \in \mathbb{R}^{3}$ |  $(\dot{{}^{A}R_{B}} {}^{A}R_{B}^{-1})^{\vee}$  | Angular velocity vector expressed in frame $B$ coordinates |

Properties:
* Change of frame: ${}^{A}{\omega}_{A,B} = {}^{A}R_B {}^{B}{\omega}_{A,B}$
* Composition rule: ${}^{A}{\omega}_{A,C} = {}^{A}{\omega}_{A,B} + {}^{A}\omega_{B,C}$



### 6D velocity representations


| Representation | Symbol | Definition | Components |
|----------------|--------|------------|------------|
| **Left-trivialized** | ${}^{B}\mathrm{v}_{A,B} \in \mathbb{R}^{6}$ | $({}^{A}H_{B}^{-1} \dot{{}^{A}H_{B}})^{\vee}$ | $\begin{bmatrix} {}^{B}v_{B} \\ {}^{B}\omega_{A,B} \end{bmatrix} = \begin{bmatrix} {}^{A}R_{B}^{-1} {}^{A}\dot{o}_{B} \\ ({}^{A}R_{B}^{-1} \dot{{}^{A}R_{B}})^{\vee} \end{bmatrix}$ |
| **Right-trivialized** | ${}^{A}\mathrm{v}_{A,B} \in \mathbb{R}^{6}$ | $(\dot{{}^{A}H_{B}} {}^{A}H_{B}^{-1})^{\vee}$ | $\begin{bmatrix} {}^{A}v_{B} \\ {}^{A}\omega_{A,B} \end{bmatrix} = \begin{bmatrix} {}^{A}\dot{o}_{B} - \dot{{}^{A}R_{B}} {}^{A}R_{B}^{-1} {}^{A}o_B \\ (\dot{{}^{A}R_{B}} {}^{A}R_{B}^{-1})^{\vee} \end{bmatrix}$ |
| **Mixed** (iDynTree default) | ${}^{A[B]}\mathrm{v}_{A,B} \in \mathbb{R}^{6}$ | ${}^{A[B]}X_B {}^{B}\mathrm{v}\_{A,B}$ = ${}^{A[B]}X_A {}^{A}\mathrm{v}_{A,B}$ | $\begin{bmatrix} {}^{A}v_{o_B} \\ {}^{A}{\omega}_{A,B} \end{bmatrix} = \begin{bmatrix} {}^{A}\dot{o}_{B} \\ (\dot{{}^{A}R_{B}} {}^{A}R_{B}^{-1})^{\vee} \end{bmatrix}$ |


---

## Multibody model: the `iDynTree::Model` class

### Links, joints and the *floating* base

* The `iDynTree::Model` represents a **unordered graph** of $N_L$ (`iDynTree::getNrOfLinks()`) rigid body (called links); and $N_J$ (`iDynTree::getNrOfJoints()`) joints, where each joint connects *an unordered pair of links*.
* Each link and each joint are associated with both a string called **name**, and a integer (between $0$ and $N_L-1$ for links and $0$ and $N_J-1$ for joints) called **index**.
* All models are **floating‑base** by construction.  Consequently the *world/universe* frame is *not* part of the link graph.
* For example, a single‑link model represents a free floating body in space and has has `iDynTree::getNrOfLinks()==1` and `getNrOfJoints()==0`.

### Frames attached to a link and additional frames

Each link $L$ is associated with a body‑fixed frame, that is identified with the same name and index of the link. Furthermore, a single link can have multiple "additional frames" (besides the link frame).

---

## The `iDynTree::IJoint` interface

A joint connecting two links is represented by an instance of a class that inherits from the `iDynTree::IJoint` C++ interface (i.e. pure virtual class). As the `iDynTree::Model` represent links as joint as an *undirected graph*, a joint is associated with two links called *first* and *second* link, but that **does not mean** that the *first* is the parent and the *second* is the child: the `iDynTree::IJoint` interface is fully direction agnostic, and joint properties can be queried by the method of the interface specifying with an argument which link should be considered the parent ($P$) and which the child ($C$).

The position of a joint $J$ is represented by a vector $\theta \in \mathbb{R}^{N_{pc}^J}$, where $N_{pc}$ is the number of the position coordinates of the joint (`iDynTree::Joint::getNrOfPosCoords()`), while its velocity is represented by $\nu_\theta \in \mathbb{R}^{N_{dof}^J}$, where $N_{dof}^J$ is the number of degrees of freedom of the joint (`iDynTree::Joint::getNrOfDOFs()`).

For simple joints we have that $N_{pc}^J = N_{dof}^J$ and $\nu_\theta  = \dot{\theta}$, while this may not be true for complex joints such as the spherical joint.

The fundamental mathematical abstraction of the joint is its called forward kinematics, i.e. the mapping between its position $\theta \in \mathrm{R}^{N_{pc}^J}$ and the homogenous transform between the links $B$ and $D$ to which it is connected:

$$
{}^C H_P(\theta) : \mathbb{R}^{N_{pc}^J} \mapsto \mathbb{R}^{4 \times 4}
$$

This quantity is the quantity that is retuned by the `iDynTree::IJoint::getTransform(const VectorDynSize& jntPos, const LinkIndex child, const LinkIndex parent)` method, where the argument are interpreted as in the following:
* `jntPos` is the full vector of all the position coordinates of all the internal joints of the `iDynTree::Model`, the actual position coordinates of the joint are extracted by taking the $N_{pc}^J$ elements starting at the location `iDynTree::IJoint::getPosCoordsOffset()` of the vector.
* `child` out of the two links to which the joint is connected, this is the one that should be considered as $C$
* `parent` out of the two links to which the joint is connected, this is the one that should be considered as $P$

This function completely describes all the kinematic properties of the joint. However, as iDynTree is a C++ library that does not return a differentiable representation of a function, but just implements the function itself in C++, it is also necessary to expose somehow the properties related to the velocity of the joint. This is implemented by functions that return the so-called **spatial motion subspace**, that is defined as follows:

$$
{}^C \mathrm{s}_{P, C}(\theta) \in \mathbb{R}^{6 \times N_{dof}^J}, \quad {}^C \mathrm{v}_{P,C} = {}^C \mathrm{s}_{P, C}(\theta) \nu_\theta
$$

In theory, the quantity ${}^C \mathrm{s}_{P, C}(\theta)$ depends on $\theta$, but for many simple joints it doesn't, so the `iDynTree::IJoint` interface assumes that it is actually independent of $\theta$.

There are $N_{dof}^J$ columns in the ${}^C \mathrm{s}_{P, C}$ matrix, and each column is returned by the `iDynTree::IJoint::getMotionSubspaceVector(int dof_i, const LinkIndex child, const LinkIndex parent)` method, where the arguments are:
* `dof_i` a number from 0 to $N_{dof}^J-1$ that identified the column to retrieve of ${}^C \mathrm{s}_{P, C}$.
* `child` out of the two links to which the joint is connected, this is the one that should be considered as $C$
* `parent` out of the two links to which the joint is connected, this is the one that should be considered as $P$

Note that both `getTransform` and `getMotionSubspaceVector` are perfectly symmetrical, i.e. you can exchange which link you consider `child` and which one you consider `parent`, and the interface (and their relative definitions) continue to work as intended.

How are ${}^C H_P(\theta)$ and ${}^C \mathrm{s}_{P, C}(\theta)$ related?

As you can write that using the definition of left-trivialized 6D velocity that:

$$
{}^P H_C {}^C \mathrm{v}_{P,C}^{\wedge} = {}^P \dot{H}_C
$$

substituting the spatial motion subspace matrix, we have:

$$
{}^P H_C {}^C \left(\mathrm{s}_{P, C}(\theta) \nu_\theta\right)^{\wedge} = {}^P \dot{H}_C
$$

This relation is used in Joint-related unit tests to make sure that the joint methods are consistent with the mathemathical definition of the `iDynTree::IJoint` interface.

### Position Derivative-Velocity Jacobian

In addition to the spatial motion subspace matrix, the `iDynTree::IJoint` interface also provides a method to compute the Jacobian matrix that relates joint velocities to the time derivative of joint position coordinates. This is particularly important for joints where the position and velocity parameterizations differ (e.g., joints using constrained representations such as unit complex numbers or unit quaternions).

The **position derivative-velocity Jacobian** is defined as the matrix $J_{\theta}(\theta) \in \mathbb{R}^{N_{pc}^J \times N_{dof}^J}$ such that:

$$
\frac{d}{dt}\theta = J_{\theta}(\theta) \nu_\theta
$$

where:
- $\theta \in \mathbb{R}^{N_{pc}^J}$ are the position coordinates of the joint
- $\nu_\theta \in \mathbb{R}^{N_{dof}^J}$ are the velocity coordinates of the joint
- $J_{\theta}(\theta)$ is the $N_{pc}^J \times N_{dof}^J$ Jacobian matrix

This matrix is returned by the `iDynTree::IJoint::getPositionDerivativeVelocityJacobian(const iDynTree::Span<const double> jntPos, MatrixView<double>& positionDerivative_J_velocity)` method, where:
* `jntPos` is the full vector of all position coordinates of all joints in the model
* `positionDerivative_J_velocity` is the output matrix (properly sized by the caller)


### Supported joint models

Internally, all store save the following information:

| Method | Math | Description |
|:----------------:|:---------------------:|:---------:|
| `IJoint::getFirstAttachedLink` | $L_1$ | `iDynTree::LinkIndex` of the first link attached. |
| `IJoint::getSecondAttachedLink` | $L_2$ | `iDynTree::LinkIndex` of the second link attached. |
| `IJoint::getRestTransform` | ${}^{L_1} H_{L_2}$ or ${}^{L_2} H_{L_1}$ | This is the transform between the two links when the joint is in the rest (typically zero) position. |

Note that the "first" and "second" links are named like that just for convenient, both the "first" and the "second" link could be the parent in the previously discussed methods of the `iDynTree::IJoint` interface.

Beside this informations, each specific type of joint has its own parameters.

#### 0-dof joints

##### Fixed joint `iDynTree::FixedJoint`

| Code                  | Quantity    | Value |
|:---------------------:|:-----------:|:---:|
| `getNrOfPosCoords()` | $N_{pc}^J$ | 0 |
| `getNrOfDOFs()`  |  $N_{dof}^J$ | 0 |

This is the simplest type of joint, it represents two links that are rigidly attached to each other. The returned transform is always the rest transform, and the ${}^C \mathrm{s}_{P, C}$ motion subspace matrix is a matrix with $0$ columns.

#### 1-dof joints

##### Revolute joint `iDynTree::RevoluteJoint`

| Code                  | Quantity    | Value |
|:---------------------:|:-----------:|:---:|
| `getNrOfPosCoords()` | $N_{pc}^J$ | 1 |
| `getNrOfDOFs()`  |  $N_{dof}^J$ | 1 |


##### Prismatic joint `iDynTree::PrismaticJoint`

| Code                  | Quantity    | Value |
|:---------------------:|:-----------:|:---:|
| `getNrOfPosCoords()` | $N_{pc}^J$ | 1 |
| `getNrOfDOFs()`  |  $N_{dof}^J$ | 1 |

##### Revolute joint using unit complex number for position coordinates `iDynTree::RevoluteSO2Joint`

| Code                  | Quantity    | Value |
|:---------------------:|:-----------:|:---:|
| `getNrOfPosCoords()` | $N_{pc}^J$ | 2 |
| `getNrOfDOFs()`  |  $N_{dof}^J$ | 1 |

---

##  References
- Traversaro Silvio, "Modelling, Estimation and Identification of Humanoid Robots Dynamics", PhD thesis, 2017, https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf .
  - Section 2.3.3 for spatial vector representations and mixed velocity convention
  - Section 2.4 for motion subspace matrices
  - Section 3.2 for joint models and transforms
- iDynFor theory background, https://github.com/ami-iit/idynfor/blob/master/doc/theory_background.md .

