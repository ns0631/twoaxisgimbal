Two-Axis Camera Gimbal
######

Overview
********

Reads acceleration & angular velocity from IMU, estimates attitude using EKF.
Turns servos accordingly in PID control loop.

Requirements
************

#. Nucleo-F446RE
#. MPU-6050 IMU

Building and Running
********************

#. Build: `west build -p auto -b nucleo_f446re twoaxisgimbal`
#. Flash: `west flash`
