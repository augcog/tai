# C106B Discussion 1: Dynamical Systems & Linear Control  

# 1 Introduction  

# 1.1 Where will discussions fit in to 106B?  

Discussions in 106B will primarily be to reinforce your understanding of concepts from lecture with practice problems. In some weeks, we’ll introduce some new foundational material in discussion so the following lectures can cover active areas of research in those subjects. Discussions will always be recorded and live streamed over zoom.  

# 1.2 What content will we cover in this class?  

In this class, we’ll cover topics ranging from nonlinear control to path planning, vision, and optimal control and reinforcement learning (RL). As we get into more advanced topics, we’ll provide you with the tools you need to get started in topics such as analysis, probability, and machine learning - we’ll only assume 106A as a background.  

# 1.3 Learning Community  

In this class, no student should feel left behind! Please ask us or any of your classmates if you have any questions at all! We want to make this course a wonderful learning experience for all of you.  

# 2 State Space  

Nonlinear systems of differential equations come in all shapes and sizes! When trying to perform general analysis of these systems, this can be challenging! Is there some convention we can use to treat general nonlinear systems?  

In general, we say that any (time invariant) nonlinear system may be described by the following equations:  

$$
\begin{array}{r l r}&{\dot{x}=f(x,u),\ x\in\mathbb{R}^{n},u\in\mathbb{R}^{m}\qquad\qquad\qquad\qquad}&{\ S t a t e\ E q u a t i o n}\\ &{y=h(x,u)\ y\in\mathbb{R}^{p}\qquad\qquad\qquad\qquad}&{\ O u t p u t\ E q u a t i o n}\end{array}
$$  

This description of a system is called state space. What are the different components?  

1. $x$ : State vector, contains smallest set of variables needed to completely describe system configuration   
2. $u$ : Input vector, contains variables we have total control over   
3. $_y$ : Output vector, commonly contains variables we are interested in controlling or those we measure with sensors  

To put an $n^{t h}$ order nonlinear differential equation, $x^{(n)}=h(x,u)$ , $x,u\in\mathbb{R}$ into state space form, we transform it into a system of $n$ first order equations using phase variables.  

$$
\begin{array}{r}{q_{0}=x,\;q_{1}=\dot{x},\;\ldots q_{n-1}=x^{(n-1)}}\\ {\left[\begin{array}{c}{\dot{q}_{0}}\\ {\vdots}\\ {\dot{q}_{n-1}}\end{array}\right]=\left[\begin{array}{c}{q_{1}}\\ {\vdots}\\ {h(q_{0},u)\right]}\\ {\dot{q}=f(q,u)}\end{array}
$$  

Problem 1: Consider a planar quadrotor of mass m and inertia about the $x$ axis I which is constrained to move in the yz plane. The dynamics of the quadrotor are described by:  

$$
\begin{array}{l}{{m\ddot{y}=-F\sin\theta}}\\ {{m\ddot{z}=F\cos\theta-m g}}\\ {{\,\,\,\,I\ddot{\theta}=M}}\end{array}
$$  

Where $F\in\mathbb{R}$ and $M\in\mathbb{R}$ are a inputs to the system.  

![](images/f4f98753eaf2379fd4df9c76f6f084a9323744ec5e5ce05548acdbbbf944088a.jpg)  

Rewrite the dynamics of the planar quadrotor as system of first order differential equations of the form:  

$$
\dot{q}=f(q,u)
$$  

Hint: Find a set of phase variables for each differential equation and put them all together into one vector!  

Solution: We choose the state vector: $q\,=\,[q_{1},q_{2},q_{3},q_{4},q_{5},q_{6}]\,=\,[y,z,\theta,{\dot{y}},{\dot{z}},{\dot{\theta}}]$ and an input vector $u=[u_{1},u_{2}]=[F,M]$ Using this state vector, we can rewrite the dynamics in state space as:  

$$
\begin{array}{r}{\left[\begin{array}{c}{\dot{q}_{1}}\\ {\dot{q}_{2}}\\ {\dot{q}_{3}}\\ {\dot{q}_{4}}\\ {\dot{q}_{5}}\\ {\dot{q}_{6}}\end{array}\right]=\left[\begin{array}{c}{\dot{q}_{4}}\\ {\dot{q}_{5}}\\ {\dot{q}_{6}}\\ {-u_{1}\sin q_{3}/m}\\ {u_{1}\cos q_{3}/m-g}\\ {u_{2}/I}\end{array}\right]}\end{array}
$$  

# 3 Linear Differential Equations  

Oftentimes, we deal with linear differential equations of the form:  

$$
{\dot{x}}=A x
$$  

Where $A\in\mathbb{R}^{n\times n},x\in\mathbb{R}^{n}$ . The solution to this differential equation for an initial condition $x(0)=x_{0}$ is:  

$$
x(t)=e^{A t}x_{0}
$$  

Where $e^{A t}\in\mathbb{R}^{n\times n}$ is the matrix exponential of $A t$ , defined:  

$$
e^{A t}=I+A t+{\frac{(A t)^{2}}{2!}}+{\frac{(A t)^{3}}{3!}}+\ldots
$$  

Problem 2: Show that the matrix exponential of a diagonal matrix $A\in\mathbb{R}^{n\times n}$ is computed:  

$$
e^{A t}={\left[\begin{array}{l l l}{e^{\lambda_{1}t}}&{\ldots}&{0}\\ {\vdots}&{\ddots}&{\vdots}\\ {0}&{\ldots}&{e^{\lambda_{n}t}{\biggr]}}\end{array}\right]}
$$  

Where $\lambda_{i}$ are the eigenvalues of $A$ .  

Solution: We know that the $m^{t h}$ power of a diagonal matrix with eigenvalues $\lambda_{1},\lambda_{2},...,\lambda_{n}$ is computed:  

$$
A^{m}=\left[\begin{array}{c c c c}{\lambda_{1}^{m}}&{\ldots}&{0}\\ {\vdots}&{\ddots}&{\vdots}\\ {0}&{\ldots}&{\lambda_{n}^{m}}\end{array}\right]\in\mathbb{R}^{n\times n}
$$  

Using this fact, we plug $A t$ into the series definition of the matrix exponential, and find:  

$$
e^{A t}=\left[\begin{array}{c c c c}{(1+\lambda_{1}t+\frac{(\lambda_{1}t)^{2}}{2!}+\ldots)}&{\ldots}&{0}\\ {\vdots}&{\ddots}&{\vdots}\\ {0}&{\ldots}&{(1+\lambda_{n}t+\frac{(\lambda_{n}t)^{2}}{2!}+\ldots)}\end{array}\right]\in\mathbb{R}^{n\times n}
$$  

We now recognize each of these diagonal entries as the scalar Taylor series of $e^{\lambda_{i}t}$ . Thus:  

$$
e^{A t}={\left[\begin{array}{l l l}{e^{\lambda_{1}t}}&{\ldots}&{0}\\ {\vdots}&{\ddots}&{\vdots}\\ {0}&{\ldots}&{e^{\lambda_{n}t}}\end{array}\right]}
$$  

This completes the proof!  

# 4 Concepts of Stability  

An equilibrium point of a system ${\dot{x}}=f(x,u)$ is a point $(x_{e},u_{e})$ where:  

$$
0=f(x_{e},u_{e})
$$  

At an equilibrium point, the evolution of the system is “frozen.” Notice that the zero vector $x\,=\,0$ is always an equilibrium point of ${\dot{x}}=A x$ .  

Let’s think of a rough definition of what it means for an equilibrium point to be stable. If we start at an initial condition $x_{0}$ close to an equilibrium point and remain close for all time, the equilibrium point is stable. Otherwise, the equilibrium point is unstable.  

The following problem highlights a special case of a relationship we’ll explore more in homework 1.  

Problem 3: Suppose that $A\in\mathbb{R}^{n\times n}$ is a diagonal matrix with real eigenvalues. Consider the linear differential equation:  

$$
{\dot{x}}=A x,\;x\in\mathbb{R}^{n\times n}
$$  

Show that $l i m_{t\rightarrow\infty}x(t)=0$ for any initial condition $x(0)\,=\,x_{0}\,\in\,\mathbb{R}^{n}$ if all of the eigenvalues of $A$ are less than zero. What does this tell us about the effect of eigenvalues on the stability of $x_{e}=0$ ?  

Solution: We know that the solution to this differential equation is given:  

$$
x(t)=e^{A t}x_{0}
$$  

For any initial condition $x_{0}$ . Using the previous question, we know that $e^{A t}$ for diagonal $A$ is computed:  

$$
e^{A t}={\left[\begin{array}{l l l}{e^{\lambda_{1}t}}&{\ldots}&{0}\\ {\vdots}&{\ddots}&{\vdots}\\ {0}&{\ldots}&{e^{\lambda_{n}t}}\end{array}\right]}
$$  

Thus, multiplying by the initial condition vector, we get:  

$$
x(t)=e^{A t}x_{0}={\left[\begin{array}{l}{e^{\lambda_{1}t}x_{01}}\\ {\qquad\vdots}\\ {e^{\lambda_{n}t}x_{0n_{-}}}\end{array}\right]}
$$  

This will converge to zero only when all of the eigenvalues of $A$ are less than zero (since they are all real numbers). This tells us if our eigenvalues are real, $x_{e}=0$ is stable if the eigenvalues are all negative.  

# 5 State Feedback  

We previously saw that the stability of the origin of a linear system is linked to the eigenvalues of the matrix $A$ . Can we use this knowledge to stabilize unstable linear systems?  

Suppose we have a linear system, this time with a scalar input $u\in\mathbb R$ , as follows:  

$$
{\dot{x}}=A x+B u,\;x\in\mathbb{R}^{n},u\in\mathbb{R}
$$  

If an eigenvalue of $A$ has a real component greater than zero, this system will be naturally unstable if we don’t provide the system with some control input. Let’s try the input:  

$$
u=-K x,\;K=[k_{1}\;k_{2}\;...\;k_{n}]
$$  

Plugging this into our system:  

$$
{\dot{x}}=(A-B K)x
$$  

Using this input, we can move the eigenvalues of many linear systems to stable locations! This is a form of feedback control known as state feedback. Note that state feedback also extends to the multi-input case.  