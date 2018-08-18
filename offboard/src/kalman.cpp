#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <kalman/matrix6.h>
#include <kalman/vector6.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/Householder>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/Eigen>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <px_comm/OpticalFlow.h>
#include <iostream>
#include "stdio.h"

using namespace std;
using namespace Eigen;

Matrix<float,3,3> M;
/*I am assuming I get Vx Vy H A from some sensor*/
 float Vx,Vy,Z,Ax,Ay,Az;        /*converted below H to be perpemdicular to quad*/
//
 Matrix<float,3,1> E;
//
//
 float x=0,y=0,z=0,vx=0,vy=0,vz=0;
 float t,tt;
//
 Matrix<float,6,1> s;      //initialise s
//
//
Matrix<float,6,6> F;

Matrix<float,6,3> B;

Matrix<float,3,6> H;

Matrix<float,6,6> P;




Matrix<float,6,6> Ft;

Matrix<float,6,3> Ht;

Matrix<float,6,6> Q;      /*decide Q for external disturbances*/
Matrix<float,3,3> R;     /**/


Matrix<float,3,1> u;

//
//


void VH_callback(const px_comm::OpticalFlow::ConstPtr& msg){
  Vx=msg->velocity_x;
  Vy=msg->velocity_y;
  Z=msg->ground_distance;             // DO z COS THETA
  Z=Z/(M(3,3));
  E << Vx,Vy,Z;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  double x,y,z,w;

  x=msg->orientation.x;
  y=msg->orientation.y;
  z=msg->orientation.z;
  w=msg->orientation.w;
  Quaternionf quat;
  quat=Eigen::Quaternionf(x,y,z,w);
  M=quat.toRotationMatrix();


  Ax=msg->linear_acceleration.x;
  Ay=msg->linear_acceleration.y;
  Az=msg->linear_acceleration.z;

  u << Ax,Ay,Az;
}





void step(){
   Matrix<float,6,1> s2;
  u=M*u;

  s2 << 0,0,0,0,0,0;
  s2=(F*s)+(B*u);

  Matrix<float,6,6> P2;
  P2=(F*P)*Ft+Q;

  s=s2;
  P=P2;
  Matrix<float,3,1> Ec;
  Ec=H*s;


}












int main(int argc, char **argv) {
  ros::init(argc,argv,"kalman");
  ros::NodeHandle nh;

  ros::Subscriber px4_sub = nh.subscribe<px_comm::OpticalFlow>
          ("/px4flow/opt_flow", 10, VH_callback);


  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
          ("/mavros/imu/data",10,imuCallback);
  ros::Publisher pub = nh.advertise<kalman::matrix6>
          ("kalman/result", 400);
//  ros::Publisher pub_co = nh.advertise<kalman::matrix6>
  //        ("kalman/covariance", 400);

  ros::Rate rate(400);
  t=1/400;
  tt=0.5*(t*t);
  s << 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f;
  F << 1,0,0,t,0,0,0,1,0,0,t,0,0,0,1,0,0,t,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1;
  B << tt,0,0,
       0,tt,0,
       0,0,tt,
       t,0,0,
       0,t,0,
       0,0,t;
  H << 0,0,0,1,0,0,
       0,0,0,0,1,0,
       0,0,1,0,0,0;
  P << 1,0,0,0,0,0,
       0,1,0,0,0,0,
       0,0,1,0,0,0,
       0,0,0,1,0,0,
       0,0,0,0,1,0,
       0,0,0,0,0,1;
  Ht << 0,0,0,
        0,0,0,
        0,0,1,
        1,0,0,
        0,1,0,
        0,0,0;

  R << 1,0,0,
       0,1,0,
       0,0,1;
  Ft=F.transpose();

  while(ros::ok()){
       step();
       Matrix<float,6,3> K;
       Matrix<float,3,3> temp;
       temp=((H*(P*Ht))+R);
       temp=temp.inverse();
       K=P*(Ht*temp);
       Matrix<float,6,1> sf;
       sf=s+K*(E-(M*(H*s)));
       Matrix<float,6,6> Pf;
       Pf=P-(K*(H*P));
    //
    // //  pub_co.publish(Pf);
      P=Pf;
      s=sf;
      kalman::vector6 sm;
  /*    kalman::matrix6 Pm;
      sm.r1.m1=sf(1,1);
      sm.r1.m2=sf(1,2);
      sm.r1.m3=sf(1,3);
      sm.r1.m4=sf(1,4);
      sm.r1.m5=sf(1,5);
      sm.r1.m6=sf(1,6);
      sm.r2.m1=sf(2,1);
      sm.r2.m2=sf(2,2);
      sm.r2.m3=sf(2,3);
      sm.r2.m4=sf(2,4);
      sm.r2.m5=sf(2,5);
      sm.r2.m6=sf(2,6);
      sm.r3.m1=sf(3,1);
      sm.r3.m2=sf(3,2);
      sm.r3.m3=sf(3,3);
      sm.r3.m4=sf(3,4);
      sm.r3.m5=sf(3,5);
      sm.r3.m6=sf(3,6);
      sm.r4.m1=sf(4,1);
      sm.r4.m2=sf(4,2);
      sm.r4.m3=sf(4,3);
      sm.r4.m4=sf(4,4);
      sm.r4.m5=sf(4,5);
      sm.r4.m6=sf(4,6);
      sm.r5.m1=sf(5,1);
      sm.r5.m2=sf(5,2);
      sm.r5.m3=sf(5,3);
      sm.r5.m4=sf(5,4);
      sm.r5.m5=sf(5,5);
      sm.r5.m6=sf(5,6);
      sm.r6.m1=sf(6,1);
      sm.r6.m2=sf(6,2);
      sm.r6.m3=sf(6,3);
      sm.r6.m4=sf(6,4);
      sm.r6.m5=sf(6,5);
      sm.r6.m6=sf(6,6);*/
      sm.m1=sf(1,1);
      sm.m2=sf(1,2);
      sm.m3=sf(1,3);
      sm.m4=sf(1,4);
      sm.m5=sf(1,5);
      sm.m6=sf(1,6);
      pub.publish(sm);







  }


  return 0;
}
