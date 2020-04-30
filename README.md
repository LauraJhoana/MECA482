![](chicostateeng.png)
# MECA-482: Control System Design

Furuta Pendulum

Spring 2020, California State University of Chico

Project Managers: Danny Cervantes, Luis Dominguez, Victor Landa, Laura Jhoana Lopez, and Juan Ruiz 

### Table of Contents
- [1. Introduction](#1-Introduction)
- [2. Modeling](#2-Modeling)
- [3. Sensor Calibration](#3-Sensor_Calibration)
- [4. Controller Design and Simulations](#4-Controller_Design_and_Simulations)
- [5. Appendix A: Simulation Code](#5-Appendix_A:_Simulation_Code)
- [6. References](#6-References)

## 1. Introduction
The Furuta Pendulum consists of a driven arm which rotates in the horizontal plane and a pendulum attached to that arm which is free to rotate in the vertical plane. The goal of the project is to use a control system to balance a beam up vertically by controlling the motors position on a gantry. There are 2 encoders that measure the 360 degree position of the motor and the angle of the beam with respect to the floor. The system will keep the beam upright and prevent it from falling by moving to the location needed to balance the beam using a 90 degree angle. This project is purely virtual therefore, our group used MATLAB and MATLAB's SIMULINK to demonstrate the inverted pendulum project with linear and non-linear control algorithms. To display how the project works virtually, the program CoppeliaSim was used which the Furuta Pendulum was created and programmed with the MATLAB code we created.

## 2. Modeling
Controller Calculations:
	Variables :
x = position of arm
θ = angle of pendulum with respect to ground
F = applid force
m = mass of pendulum
l = length of pendulum 
g = gravity
fo = coefficient of friction
With two degrees of freedom the following equations are produced by the system:

----------------------------------------------------------------------------------
## 3. Sensor Calibration
There was no sensor calibration for this project due to the entirety of the project being virtual.

-----------------------------------------------------------------------------------------------------
## 4. Controller Design and Simulation

-----------------------------------------------------------------------------------------------------

## 5. Appendix A: Simulation Code

--------------------------------------------------
## 6. References
	1.



