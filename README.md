# sensor-fusion-quadcopter

## Sources

[DCM](http://www.starlino.com/wp-content/uploads/data/dcm_tutorial/Starlino_DCM_Tutorial_01.pdf)

[dcm_tutorial](http://www.starlino.com/dcm_tutorial.html)

[IMU and AHRS](http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

[kalman filter](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)

## PX4_FLOW CAM

PX4_flow camera gives the velocity of quadcopter and the depth w.r.t. quadcopter . 

To install the camera `git clone https://github.com/cvg/px-ros-pkg.git` in the catkin_ws/src folder.


## Working with Firmware
> To compare the real pixhawk's data with our code's published data.

* Update the Firmware
* Check the port using `ls /dev/px4/ttyAMm` and then change to appropriate port inside the px4.launch file
* Launch locate `roscd mavors px4.launch`
* Here you can see the required rostopic 

## AHRS CODE

AHRS code gives us the accelerometer , gyroscope and magnetometer's data was fused together to give a reliable ,fast and less unerring data .

Download the code `imusf.cpp`

In your offboard node paste the imusf.cpp and make required changes in the CMakeLists.txt and package.xml . See our offboard node inside this repository . OR download the whole offboard packege.

Now `catkin_make` and then source it.

Now run `rosrun offboard imusf` 

You can compare the published data from imusf and the rostopic /imu/data

#### Working of imusf code

From our magnetometer's , accelerometer's and gyroscope's data we calculated the orientation of our quadcopter without using the imu's data . Use this quaternion and to get euler angles (roll , pitch , yaw)..

## Kalman code

Kalman code which basically fuses the data which we are receiving from the sensors and our predicted value to give optimized value.

There are basically two step involved:-
1.Prediction step.
2.Update step.
The basic logic behind this algorithm is that we used a recursive loop to calculate the value itratively.

> Prediction Step:-
> In this we use the final result that we got from previous step and use Newton Laws to get the predicted value for  the next value.

> Update Step:-
> In this setup we use our data from prediction step and fused it with sensors data using kalman equation.
[kalman filter](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/ )
