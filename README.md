# sensor-fusion-quadcopter

## Sources

[DCM](http://www.starlino.com/wp-content/uploads/data/dcm_tutorial/Starlino_DCM_Tutorial_01.pdf)

[dcm_tutorial](http://www.starlino.com/dcm_tutorial.html)

[IMU and AHRS](http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

[kalman filter](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)


## AHRS CODE

We finally wrote AHRE code in which the accelerometer , gyroscope and magnetometer's data was fused together to give a reliable ,fast and less unerring data .

Download the code `ahrs.cpp`

In your offboard node paste the ahrs.cpp and make required changes in the CMakeLists.txt and package.xml . See our offboard node inside this repository . OR download the whole offboard packege.

Now `catkin_make` and then source it.

Now run `rosrun offboard imusf`

You can compare the published data from imusf and the rostopic /imu/data

## Kalman code

We wrote the kalman code which you can find here in the kalman

> Next we wanted to compare the real pixhawk's data with our code's published data.

* Update the Firmware
* Check the port using `ls /dev/px4/ttyAMm` and then change to appropriate port inside the px4.launch file
* 
