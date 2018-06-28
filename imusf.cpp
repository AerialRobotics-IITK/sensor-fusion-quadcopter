#include <Eigen/Geometry>
#include <math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/MagneticField.h>
#include <iostream>

using namespace std;
using namespace Eigen;

#define sampleFreq	80.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

Quaternionf quat;
 double mx,my,mz;

 Quaternionf quatfinal;

mavros_msgs::State current_state;

 void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	Quaternionf qgyro = Eigen::Quaternionf(0.0f,gx,gy,gz);

	Quaternionf qDot = Eigen::Quaternionf(1.0f,0.0f,0.0f,0.0f);

	Quaternionf qh = Eigen::Quaternionf(1.0f,0.0f,0.0f,0.0f);

	Quaternionf qb = Eigen::Quaternionf(1.0f,0.0f,0.0f,0.0f);

	Quaternionf qacc = Eigen::Quaternionf(0.0f,ax,ay,az);

	Quaternionf qmag = Eigen::Quaternionf(0.0f,mx,my,mz);

	Quaternionf delf = Eigen::Quaternionf(1.0f,0.0f,0.0f,0.0f);

	//taking the derivative of quaternion
  	qDot=quat*qgyro;

    //Since real and imaginary parts are dealt seprately while mutiplying with a constant
	qDot.w()=qDot.w()*0.5;
	qDot.vec()=qDot.vec()*0.5;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {


        qacc.normalize();
        qmag.normalize();
        	//calculating the h quaternion which is the mag data in the earth frame
		qh=((quat*qmag)*quat.conjugate());
		qb.x()=sqrt(qh.x()*qh.x()+qh.y()*qh.y());
    	qb.z()=qh.z();
    	qb.w()=0;
    	qb.y()=0;

    	qb.normalize();
        // Gradient decent algorithm corrective step

        MatrixXf Fg(3,1);
        MatrixXf Fb(3,1);
       	MatrixXf Jb(3,4);
       	MatrixXf Jg(3,4);

       	//calcuating the acceleration covariance matrix
    	Fg<<2*(quat.x()*quat.z()-quat.w()*quat.y())-qacc.x(),
            2*(quat.w()*quat.x()+quat.y()*quat.z())-qacc.y(),
            2*(0.5-quat.x()*quat.x()-quat.y()*quat.y())-qacc.z();



       	Fb<<2*qb.x()*(0.5-quat.y()*quat.y()-quat.z()*quat.z())+2*qb.z()*(quat.x()*quat.z()-quat.w()*quat.y())-mx,
            2*qb.x()*(quat.x()*quat.y()-quat.w()*quat.z())+2*qb.z()*(quat.w()*quat.x()+quat.y()*quat.z())-my,
            2*qb.x()*(quat.w()*quat.y()-quat.x()*quat.z())+2*qb.z()*(0.5-quat.x()*quat.x()-quat.y()*quat.y())-mz;

    	//this is jacobian of the accelerometer
    	Jg<<-2*quat.y(),2*quat.z(),-2*quat.w(),2*quat.x(),
            2*quat.x(),2*quat.w(),2*quat.z(),2*quat.y(),
            0,-4*quat.x(),-4*quat.y(),0;

    	Jb<<-2*qb.z()*quat.y(),2*qb.z()*quat.z(),-4*qb.x()*quat.y()-2*qb.z()*quat.w(),-4*qb.x()*quat.z()+2*qb.z()*quat.x(),
            -2*qb.x()*quat.z()+2*qb.z()*quat.x(),2*qb.x()*quat.y()+2*qb.z()*quat.w(),2*qb.x()*quat.x()+2*qb.z()*quat.z(),-2*qb.x()*quat.w()+2*qb.z()*quat.y(),
            2*qb.x()*quat.y(),2*qb.x()*quat.z()-4*qb.z()*quat.x(),2*qb.x()*quat.w()-4*qb.z()*quat.y(),2*qb.x()*quat.x();

    	MatrixXf Fgb(6,1);
        MatrixXf Jgb(6,4);

        Fgb<<Fg,
             Fb;
    	Jgb<<Jg,
             Jb;

    	MatrixXf quatMat(4,1);
    	quatMat = Jgb.transpose()*Fgb;


    	delf = Eigen::Quaternionf(quatMat(0,0), quatMat(1,0), quatMat(2,0), quatMat(3,0));

    	qDot.w()=qDot.w()-(betaDef*delf.w());
    	qDot.vec()=qDot.vec()-(betaDef * delf.vec());

    	quat.w()+=qDot.w()*(1.0f / sampleFreq);
    	quat.vec()+=qDot.vec()*(1.0f / sampleFreq);

    	quat.normalize();
	}
 }



void imumagnetic(const sensor_msgs::MagneticField::ConstPtr &msg)
{
	mx=msg->magnetic_field.x;
	my=msg->magnetic_field.y;
	mz=msg->magnetic_field.z;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg){

  double gx,gy,gz,ax,ay,az;double x,y,z,w;

  gx = msg->angular_velocity.x;
  gy = msg->angular_velocity.y;
  gz = msg->angular_velocity.z;

  ax = msg->linear_acceleration.x;
  ay = msg->linear_acceleration.y;
  az = msg->linear_acceleration.z;



  MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz);
 // R=quat.toRotationMatrix();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imusf_node");
    ros::NodeHandle nh;

    ros::Subscriber mag_sub = nh.subscribe<sensor_msgs::MagneticField>
            ("/mavros/imu/mag", 10, imumagnetic);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("/mavros/imu/data_raw",10,imuCallback);
    ros::Publisher ahrs_pub = nh.advertise<geometry_msgs::Quaternion>
             ("imusf", 100);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


	quat = Eigen::Quaternionf(1,0,0,0);
  geometry_msgs::Quaternion quatfinal ;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    while(ros::ok()){

        //cout<<"quaternion"<< quat.w()<<   quat.vec()<<endl;
        quatfinal.x=quat.x();
        quatfinal.y=quat.y();
        quatfinal.z=quat.z();
        quatfinal.w=quat.w();

        ahrs_pub.publish(quatfinal);

       ros::spinOnce();
       rate.sleep();
   }


    return 0;
}
