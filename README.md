# sensor-fusion-quadcopter

## Sources

[DCM](http://www.starlino.com/wp-content/uploads/data/dcm_tutorial/Starlino_DCM_Tutorial_01.pdf)

[dcm_tutorial](http://www.starlino.com/dcm_tutorial.html)

[IMU and AHRS](http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

[kalman filter](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)


## AHRS CODE

We finally wrote AHRE code in which the accelerometer , gyroscope and magnetometer's data was fused together to give a reliable ,fast and less unerring data . For this we combined the best parts of all these sensors and using kalman filter we tried to minimize the error.

see our code ahrs.cpp
