# Trajectory Planning Model for SCARA Manipulator

A Simulation Model for trajectory Planning developed in MATLAB and Simulink to generate desired trajectory using Trapezoidal Velocity Profile and providing joint torques using Inverse Dynamic Control aproach to manipulate a SCARA Robot carrying 5 kg load at end effector.

## **Trajectory Generation**

The Objective is to compute the trajectory in the operational space using Trapezoidal Velocity Profile. Four segments are given starting from the initial and final points. For each segment, position are calculated using the Trapezoidal Velocity formula.

Once the position coordinates of the segments are found then the value of the arc lengths can be found using position coordinates in the q(t) matrix. These values of arc lengths is saved in array "sj" for segment. "sj" is found by getting the norm of the position and the previous position in the q(t) matrix and it is then added to the previous element of "sj".

Anticipation time is given as 0.2 seconds. This means that the trajectory will end in 3.4 seconds instead After considering the anticipation time and using this vector of arc length, “Pe” is calculated, which  the position in the robot operational space. This position is plotted with respect to time in a 3D frame to visualize the trajectory. Below, there is figures of position, velocity and acceleration in 2d frame and trajectory in operational space.

![Traj Gen (1)](https://user-images.githubusercontent.com/73630123/221020555-67bb25af-cb2e-4f6d-aa29-0f2533e75918.jpg)

## **Inverse Dynamic Control Approach**

The objective is to use Inverse Dynamic Control system to obtain linearization using non-linear state feedback. The Inverse Dynamic Control has two subsystems where one obtains linear and decoupled I/O relationships and the other controls the system. 

### ***Second Order Inverse Kinematics Algorithm***

The files “direct_kin”, “jacobian”, “jacobian_dot” and “jacobian_inverse” contributes to the subsystem which controls the linear system. The aim of direct kinematics is to compute the pose of the end effector as a function of joint variables. This subsystem utilizes a second order inverse algorithm to compute the joint values using the geometrical Jacobian formula. The generated_traj.mat file works as the input to the second order algorithm which is basically the desired trajectory. After calculating the desired trajectory, it can be verified that the subsystem is working properly as the error values of the desired and obtained trajectory becomes
0. This subsystem gives the outputs: qd, qd_dot and qd_dot_dot which is then taken as inputs in the other subsystem.

![Second Order Algorithm](https://user-images.githubusercontent.com/73630123/221025141-4708284a-079f-4902-8612-458c6d20d599.jpg)

### ***Inverse Dynamic Controller***

The Inverse Dynamic control system is implemented by computing the joint torques applicable
on the manipulator carrying a 5kg load. The second order inverse algorithm is utilized to control
the system and then joint torques are calculated with respect to the q i.e., joint values which are
then fed in the manipulator block. The reason of using manipulator block is that we are applying
this control system in a simulation environment and not physical SCARA arm. Since the physical
manipulator is not present, we are inverting B(q) matrix to calculate the q_dot_dot which then is
used to obtain q_dot and q.

![Inverse Dynamic Control](https://user-images.githubusercontent.com/73630123/221023976-6f42cc3b-de28-4fe2-b7d8-c8ae46799dd6.jpg)

