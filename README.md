# URDF-Desc-6DOF-to-SerialLink
Interpretation  of an URDF file of a 6 degrees of freedom robot manipulator into SerialLink using DHFactor function  

**Date** created 2021 Q1  
**Author** Diego P  
Robotics Toolbox 10  
Matlab / Octave  


## Description
The project takes a six degrees of freedom robot manipulator described in an URDF file and is written into a sintaxis compatible with the Matlab function DH-Factor.
Once the Links are defined by default, the parameters of limits for every join are added.
A direct parsing is not a valid approach, because not every URDF description has a valid solution for SerialLink. URDF allows branches when defining the links of the robots and also can have turns in all the axis. Serialink is limited only to Z-turns and no branched links.

Definition of links is made with the DHFactor function.
Overwritting the limits values are made with SerialLink.Link() function.

## Original Robot

![Gauss robot in real life](https://github.com/patchdev/URDF-Desc-6DOF-to-SerialLink/blob/main/img/gauss_image_design.png?raw=true)

## SerialLink Output
