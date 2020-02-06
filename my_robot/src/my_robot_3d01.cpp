#include "ros/ros.h"  
#include "std_msgs/Float64.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/LU"

using namespace std;
using namespace Eigen;
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_robot"); 

  ros::NodeHandle nh;
  

  ros::Publisher  pub_joint1, pub_joint2;
   
  // set sampling time

  float T = 0.01;
  	
  ros::Rate rate(1/T);
  

  std_msgs::Float64 target_joint1, target_joint2;


  pub_joint1 = nh.advertise<std_msgs::Float64>("/my_robot/arm1_joint_effort_controller/command", 10);
  pub_roll_joint2 = nh.advertise<std_msgs::Float64>("/my_robot/arm2_roll_joint_effort_controller/command", 10);
  pub_pitch_joint2 = nh.advertise<std_msgs::Float64>("/my_robot/arm2_pitch_joint_effort_controller/command", 10);




    // set robot data

	float mass[3] 	  = {5.478,0.65,0.172};
	float mass_all    = mass[0]+mass[1]+mass[2];
	float length[3]   = {0.15,0.15,0.15};
	float length_g[3] = {0.075,0.075,0.075};
	float inertia[2]  = {0.5*mass[1]*length[1]*length[1],
			    		 0.5*mass[2]*length[2]*length[2]};
	float g = 9.8;

	// set phi in order to easily

	float phi[6] = {mass[1]*length_g[1]*length_g[1],
					mass[2]*length[1]*length[1],
					mass[2]*length[1]*length_g[2],
					mass[1]*length_g[1],
					mass[2]*length[1],
					mass[2]*length_g[2]};

	// set gain

	float kp = 10;
	float kd = 5;

	// set force of foot

	float F[3][1] = {{0},
					 {0},
					 {mass_all*g}};

	// set initial value q0 q1 q2

	float q[3] = {0,
				  40*M_PI/180,
		          100*M_PI/180};
	float q_dot[3] = {0,0,0};
	float q_2dot[3]; 
	float u[3];

	// set target trajectory 

	float qd[3] = {0,
				   80*M_PI/180,
				   20*M_PI/180};
	float qd_dot[3]  = {0,0,0};
	float qd_2dot[3] = {0,0,0};
	
	// set timer

	int t = 0;

	// caluculation by eular

  	while (ros::ok()) { 

		// set dynamics

		float Hb[3][3] = {{mass_all,0,0},
						  {0,mass_all,0},
						  {0,0,mass_all}};
			      
		float Hbj[3][3] = {{0,(phi[3]+phi[4])*sin(q[1])+phi[5]*sin(q[1]+q[2]),phi[5]*sin(q[1]+q[2])},
						   {-(phi[3]+phi[4])*sin(q[0]),0,0},
						   {(phi[3]+phi[4])*sin(q[0]),-(phi[3]+phi[4])*cos(q[1])-phi[5]*cos(q[1]+q[2]),-phi[5]*cos(q[1]+q[2])}};

		float Hj[3][3] = {{phi[0]+phi[1]+inertia[0]+inertia[1],-(phi[0]+phi[1])*sin(q[0])*cos(q[1])-phi[2]*sin(q[0])*cos(q[1]+q[2]),-phi[2]*sin(q[0])*cos(q[1]+q[2])},
						  {-(phi[0]+phi[1])*sin(q[0])*cos(q[1])-phi[2]*sin(q[0])*cos(q[1]+q[2]),phi[0]+phi[1]+2*phi[2]*cos(q[2])+inertia[0]+inertia[1],phi[2]*cos(q[2])+inertia[1]},
						  {-phi[2]*sin(q[0])*cos(q[1]+q[2]),phi[2]*cos(q[2])+inertia[1],inertia[1]}};


		float Cb[3][1] = {{(phi[3]+phi[4])*q_dot[1]*q_dot[1]*cos(q[1])+phi[5]*(q_dot[1]+q_dot[2])*(q_dot[1]+q_dot[2])*cos(q[1]+q[2])},
						  {(phi[3]+phi[4])*q_dot[0]*q_dot[0]*sin(q[0])},
						  {(phi[3]+phi[4])*(q_dot[0]*q_dot[0]*cos(q[0])+q_dot[1]*q_dot[1]*sin(q[1]))+phi[5]*(q_dot[1]+q_dot[2])*(q_dot[1]+q_dot[2])*sin(q[1]+q[2])}};

		float Cj[3][1] = {{(phi[0]+phi[1])*q_dot[1]*q_dot[1]*sin(q[0])*sin(q[1])+phi[2]*(q_dot[1]+q_dot[2])*(q_dot[1]+q_dot[2])*sin(q[0])*sin(q[1]+q[2])},
						  {-(phi[0]+phi[1])*q_dot[0]*q_dot[0]*cos(q[0])*cos(q[1])-phi[2]*(q_dot[0]*q_dot[0]*cos(q[0])*cos(q[1]+q[2]))+2*q_dot[1]*q_dot[2]*sin(q[2])+q_dot[2]*q_dot[2]*sin(q[2])},
						  {phi[2]*(-q_dot[0]*q_dot[0]*cos(q[0])*cos(q[1]+q[2]))+q_dot[1]*q_dot[1]*sin(q[2])}};

	
		float Gb[3][1] = {{0},
						  {0},
						  {mass_all*g}};

		float Gj[3][1] = {{(phi[3]+phi[4])*g*sin(q[0])},
						  {-(phi[3]+phi[4])*g*cos(q[1])-phi[5]*g*cos(q[1]+q[2])},
						  {-phi[5]*g*cos(q[1]+q[2])}};

		float J[3][3] = {{0,length[1]*sin(q[1])+length[2]*sin(q[1]+q[2]),length[2]*sin(q[1]+q[2])},
						 {-length[1]*cos(q[0]),0,0},
						 {length[1]*sin(q[0]),-length[1]*cos(q[1])-length[2]*cos(q[1]+q[2]),-length[2]*cos(q[1]+q[2])}};

		

		q_2dot = ;
		
		

		u[0] = M[0]*tem2[0]+M[1]*tem2[1]+tem4[0];
		u[1] = M[2]*tem2[0]+M[3]*tem2[1]+tem4[1];

		q_dot[0] = q_dot[0]+T*q_2dot[0];
		q_dot[1] = q_dot[1]+T*q_2dot[1];

		q[0] = q[0]+T*q_dot[0];
		q[1] = q[1]+T*q_dot[1];
		
		
	
		// publish data

		target_joint1.data = -u[1];	
		target_joint2.data = u[0];
		pub_joint1.publish(target_joint1);   // knee 
    	pub_joint2.publish(target_joint2);   // hip	
	
		cout << "------------------------------------------------------" << endl;
		cout << "knee_torque = " << u[1] << "Nm  knee_velocity = " << q_dot[1] << endl;
		cout << "hip_torque  = " << u[0] << "Nm  hip_velocity  = " << q_dot[0] << endl;
		
		t = t+1;

	ros::spinOnce(); 
	rate.sleep();
	
  	}
  
	return 0;
}




