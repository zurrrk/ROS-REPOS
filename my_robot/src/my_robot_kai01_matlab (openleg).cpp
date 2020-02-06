#include "ros/ros.h"  
#include "std_msgs/Float64.h"
#include <stdio.h>
#include <string.h>
#include <iostream>

using namespace std;
 
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
  pub_joint2 = nh.advertise<std_msgs::Float64>("/my_robot/arm2_joint_effort_controller/command", 10);




        // set robot data

	float mass[3] 	  = {5.238,0.65,0.172};
	float length[3]   = {0.15,0.15,0.15};
	float length_g[3] = {0.075,0.075,0.075};
	float inertia[2]  = {0.5*mass[1]*length[1]*length[1],
			    		 0.5*mass[2]*length[2]*length[2]};
	float g = 9.8;

	// set phi in order to easily

	float phi[5] = {mass[0]*length[2]*length[2]+mass[1]*length[2]*length[2]+mass[2]*length_g[2]*length_g[2],
					mass[1]*length_g[1]*length_g[1]+mass[0]*length[1]*length[1],
					mass[1]*length[2]*length_g[1]+mass[0]*length[1]*length[2],
					mass[2]*g*length_g[2]+(mass[0]+mass[1])*g*length[2],
					mass[1]*g*length_g[1]+mass[0]*g*length[1]};

	// set gain

	float kp = 10;
	float kd = 5;

	// set estimate error

	float alpha = 1.0;

	// set target trajectory

	float qd[2] = {80*M_PI/180,
				   M_PI-2*(80*M_PI/180)};
	float qd_dot[2]  = {0,0};
	float qd_2dot[2] = {0,0};

	// set initial value

	float q[2] = {45*M_PI/180,
		          M_PI-2*(45*M_PI/180)};
	float q_dot[2] = {0,
			  		  0};
	float q_2dot[2]; 
	float u[2];

	// set finish time

	// float t_end = 2;

	// caluculation by eular

  	while (ros::ok()) { 

		// set dynamics

		float M[4] = {phi[0]+phi[1]+2*phi[2]*cos(q[1])+inertia[0]+inertia[1],
			      phi[0]+phi[2]*cos(q[1])+inertia[1],
			      phi[0]+phi[2]*cos(q[1])+inertia[1],
			      phi[0]+inertia[1]};

		float C[4] = {-2*phi[2]*sin(q[1]),
			      -phi[2]*q_dot[1]*sin(q[1]),
			      phi[2]*q_dot[0]*sin(q[1]),
			      0};
	
		float G[2] = {phi[4]*cos(q[0])+phi[3]*cos(q[0]+q[1]),
			      phi[3]*cos(q[0]+q[1])};
		
		float tem1[2];
		float tem2[2];
		float tem3[2];
		float tem4[2];		

		tem1[0] = (-C[0]*q_dot[0]-C[1]*q_dot[1]-G[0])*(alpha-1.0); 
		tem1[1] = (-C[2]*q_dot[0]-C[3]*q_dot[1]-G[1])*(alpha-1.0);

		tem2[0] = qd_2dot[0]-kp*(q[0]-qd[0])-kd*(q_dot[0]-qd_dot[0]);
		tem2[1] = qd_2dot[1]-kp*(q[1]-qd[1])-kd*(q_dot[1]-qd_dot[1]);

		tem3[0] = tem1[0]+M[0]*tem2[0]+M[1]*tem2[1];
		tem3[1] = tem1[1]+M[2]*tem2[0]+M[3]*tem2[1];

		q_2dot[0] = 1/(M[0]*M[3]-M[1]*M[2])*(M[3]*tem3[0]-M[1]*tem3[1]);
		q_2dot[1] = 1/(M[0]*M[3]-M[1]*M[2])*(-M[2]*tem3[0]+M[0]*tem3[1]);
		
		tem4[0] = (C[0]*q_dot[0]+C[1]*q_dot[1]+G[0])*alpha;
		tem4[1] = (C[2]*q_dot[0]+C[3]*q_dot[1]+G[1])*alpha;

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

	ros::spinOnce(); 
	rate.sleep();
	
  	}
  
  return 0;
}




