# Segbot with color based state machine
In this project, a custom Segway robot is used which is operated using NI myRio platform. The platform also has a Texas Instruments' MSP430 chip mounted on it. The goal of this project is to make the robot balance on its 2 wheels while the camera hooked up on the robot looks for commands in the forms of colors. Depending on the color the webcam sees, the robot executes different trajectories.

# How to view the files?
In the "segway_open_loop" folder inside the zip file, open the "Segway" labview project. The "segway_MSP.c" file contains the code for indicator logic. When the segbot turns right, the right LED blinks and similarly when the segbot turns left, the left LED blinks. 
