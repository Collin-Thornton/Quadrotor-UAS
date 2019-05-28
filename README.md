"# Quadrotor-UAS" 

Project: To build and code a quadcopter drone capable of maintaining stable flight over a duration of time as a self-driven project in high school.

Summary of code:

The code is fairly straight forward, with sections for variable definitions, PWM input timers, a setup loop, and main loop. The main loop is divided into two main sections: the first an outer loop that runs the interrupts for the MPU6050 DMP (concept taken from the example in Jeff Rowberg's I2CDevLib MPU6050_DMP6 Arduino sketch) and an inner loop that contains the control algorithms. 

This inner loop begins with low voltage alarms and motor arming switch before it handles incoming sensor data. It then finds the angular velocity of all three axises by taking the first derivative of the Euler angles outputted by the MPU6050 processing. This is a somewhat redundant process, but justified in that it allows the program to make use of the more efficient processing of the MPU6050 DMP.

Once the data is handled, the program reaches the section for parsing commands from serial input. Many of the commands deal with adjusting PI values of the various control loops, which makes for efficient tuning over a BLE connection.

Finally, the program moves to the control algorithm. There are two flight settings, ACRO and ANGLE. The first implements only one PI loop, which regulates the angular velocity on the three axises. The second implements two PI loops, an inner and outer. The outer loop, which only runs every three cycles of the program, takes user input from the R/C controller and maps it to a specific degree of tilt. The error found between the user input and existing tilt angle is then passed to the inner loop, which regulates angular velocity. This inner loop takes the input, scales it, and then uses it as a reference point to acclerate or decelerate the motors. 
