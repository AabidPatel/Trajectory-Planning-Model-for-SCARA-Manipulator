# Trajectory Planning Model for SCARA Manipulator

A Simulation Model for trajectory Planning developed in MATLAB and Simulink to generate desired trajectory using Trapezoidal Velocity Profile and providing joint torques using Inverse Dynamic Control aproach to manipulate a SCARA Robot carrying 5 kg load at end effector.

## **Trajectory Generation:**

The Objective is to compute the trajectory in the operational space using Trapezoidal Velocity Profile. Four segments are given starting from the initial and final points. For each segment, position are calculated using the Trapezoidal Velocity formula.

Once the position coordinates of the segments are found then the value of the arc lengths can be found using position coordinates in the q(t) matrix. These values of arc lengths is saved in array "sj" for segment. "sj" is found by getting the norm of the position and the previous position in the q(t) matrix and it is then added to the previous element of "sj".

Anticipation time is given as 0.2 seconds. This means that the trajectory will end in 3.4 seconds instead of 4 seconds. This is because, the first segment will be for 0.6 seconds then the second segment will be from 0.4 to 1.8 seconds as the second segment’s duration is 1.4 seconds. Since the third segment’s duration is 1.4 seconds, the segment will be from 1.6 to 3.0 seconds. Similarly, the fourth segment will be from 2.8 to 3.4 seconds as the duration of fourth segment is 0.6 seconds.

After considering the anticipation time and using this vector of arc length, “Pe” is calculated, which is basically the position in the robot operational space. This position is plotted with respect to time in a 3D frame to visualize the trajectory. Below, there is figures of position, velocity and acceleration in 2d frame and trajectory in operational space.

![Traj Gen (1)](https://user-images.githubusercontent.com/73630123/221020555-67bb25af-cb2e-4f6d-aa29-0f2533e75918.jpg)

## **Inverse Dynamic Control Approach:**

***Second Order Inverse Kinematics Algorithm:***

![Second Order Algorithm](https://user-images.githubusercontent.com/73630123/221025141-4708284a-079f-4902-8612-458c6d20d599.jpg)

***Inverse Dynamic Controller:***

![Inverse Dynamic Control](https://user-images.githubusercontent.com/73630123/221023976-6f42cc3b-de28-4fe2-b7d8-c8ae46799dd6.jpg)

