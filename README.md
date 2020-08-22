# MATLAB Control of Planar Robots
MATLAB scripts to implement common control techniques on planar robots.
<!---
1. 1-DoF
   1. Impedance
      1. Joint
      1. Task
   1. Admittance
      1. Joint
      1. Task
1. 2-DoF
   1. Impedance
      1. Joint
      1. Task
   2. Admittance
      1. Joint
      2. Task
     ---> 
## Robot model
The robot dynamics (Siciliano and Khatib, 2008) are described by the following matrices:

H: Inertia\
C: Coriolis forces\
F: Friction forces\
G: Gravity forces

To simpler planar models, these matrices are intuitive and easy to calculate means of Newton's equilibrium equations or by the Lagrage method.

## Environment model
Here, the environment with which the robot interacts is modeled as a lumped-element system consisting of a damping and a stiffness (B<sub>env</sub> and K<sub>env</sub>).

Here, the environments are divided into two types: 
1. <b>Stiff</b> environments, in the sense that the environment is something that, to a certain extent, holds its own shape and interacts with the robot through its end-effector.
2. <b>Soft</b> (or fluid) environments, in the sense that the environment surrounds the robot (i.e. it does not have a particular object shape), opposing to different degrees to the robot's movement. In this case, the interaction is rather assumed to occur at joint level only.

## Force sensor model (optional)
Here, the dynamics of the force sensor are neglected. In case they are considered, they can be modeled as the environment (i.e. with lumped-elements, B<sub>fs</sub> and K<sub>fs</sub>) and placed in series between the robot end-effector and the environment.

Force sensors are usually modeled as pure-stiffness elements. The resulting interaction force, in the case a force sensor is present, and in the case K<sub>env</sub> >>> and K<sub>fs</sub>, would be the force sensor stiffness (K<sub>int</sub>) times the force sensor displacement.

F<sub>int</sub> = K<sub>fs</sub>x<sub>fs</sub>

which can be considered as extent as the force the environment exerts onto the robot.

## Control architecture
To deal with the interactions with the surrounding environment, two control loops exist within the present architecture. The inner-loop controller (which runs at a faster rate as the outer loop) always deals with reference tracking, whatever that reference might be (position, force, etc.), whereas the outer control loop deals with the interactions that may occur with the environment, whether this environment is virtual or real.

In the absence of interaction, the outer loop is not active and the inner loop rules the robot movement.\
In the case the robot interacts with the environment, both loops are active.\
The <i>reason d'être</i> of the outer loop is to change the robot reference, which is input in the inner loop.
   
### Inner Control Loop
   The inner controller is a Proportional-Derivative-Integrative (PID). There are instances in which it is simplified to simpler forms by tuning one of the gains to zero.\
   With respect to the <b>admittance scheme</b>, the inner controller is a <b>position controller</b>.\
   With respect to the <b>impedance scheme</b>, the inner controller is a <b>force controller</b>.
   
### Outer Control Loop
   The outer controller models the interaction of the robot with the environment. It is either of impedance (Z) or admittance (Y) type (HOGAN, 1985).\
   Both schemes models the interaction in order to achieve the behavior of a second-order system described by a desired inertia M<sub>d</sub>, damping B<sub>d</sub> and stiffness K<sub>d</sub>.
   With regards to the impedance controller, it generates a force output (F) as a displacement error (x) is sensed:
   
   Z = M<sub>d</sub>xs<sup>2</sup> + B<sub>d</sub>xs + Kx
   
   With regards to the admittance controller, it generates a displacement (x) as a force error (F) is sensed:
   
   Y = (M<sub>d</sub>xs<sup>2</sup> + B<sub>d</sub>xs + Kx)<sup>-1</sup> = Z<sup>-1</sup>
   
   It is said that impedance and admittance form a dual relationship.
   
## Task Space and Joint Space
### Task Space
In task space, the interaction of the robot with the environment is represented by linear quantities, i.e. forces and vertical/horizontal displacements. It is as if the robot was moving around and, suddenly, something blocked its end-effector, for instance a wall or any other obstacle.

Thus, the environment offers resistance to robot displacement over X and Y directions, but the robot can rotate its joints freely (as long as the X and Y position of the end-effector do not change).

### Joint Space
In joint space, the interaction of the robot with the environment manifests as angular quantities, i.e. torques and angular displacements. It is as if the robot had the movement of its joints constrained to certain values (similarly to the human joints), or as if the joints themselves had some sort of damping within its bearings.

Thus, the environment offers resistance the angular displacement performed by the joints of the robot. This restriction may occur only to certain values of displacement (again, similarly to human joints) or over the whole range of values of the joint.

### Which one is right?
There is no right or wrong whatsoever. It depends solely on the application, the situation you are trying to model.\
In case you are trying to model the interaction of a industrial robot (UR5 for instance) with the surrounding environment, so it does not hit walls blindly, and rather stops to prevent penetrating them, task space is more adequate.

Now imagine the case you are trying to model a robotic arm which interacts with a fluid: the robotic arm attached to a submarine, for instance. The interaction with the environment occurs at joint level, that is, the environment generates resistive torques that preven the robot from moving accordingly. In this case, the joint space approach is more adequate.

The same is applicable with respect not only to the interaction with respect to the environment (outer control loop) but also with respect to the inner-control loop as well.

If you want to follow a desired joint trajectory or torque reference, joint space is your way to go.\
Whereas if you want to follow a desired trajectory in space (X,Y,Z) coordinates, task space is what suits better.

Performance restrictions, application, etc. may also lead you to approach the problem one way or another.

## One degree-of-freedom
![imp-1dof-rot](https://github.com/iikaro/planar-robot-control/blob/master/drawings/imp-1dof/imp-1dof-joint.png)

### Impedance


## References
SICILIANO, B.; KHATIB, O. Springer Handbook of Robotics. 1st edition. Springer-Verlag Berlin Heidelberg, 2008. 1611 p.\
HOGAN, N. Impedance Control: An Approach to Manipulation: Parts I, II and III. Journal of Dynamic Systems, Measurement, and Control, v. 107, n. 1, p. 1–24,03 1985. ISSN 0022-0434.

## Supporting MATLAB Scripts
Here I provide a list of supporting MATLAB scripts which were not written by me. However, I may have made a few changes and adaptations to their original scripts (very few I'd say) so I'd recommed people to always refer to the scripts on this repository rather than the ones from MATLAB page.\
Brandon Kuczenski (2020). hline and vline (https://www.mathworks.com/matlabcentral/fileexchange/1039-hline-and-vline), MATLAB Central File Exchange. Retrieved August 22, 2020. 
