#include "ros/ros.h"  
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/LU"

using namespace std;
using namespace Eigen;

float arm1_position;
float arm1_velocity;
float arm2_pitch_position;
float arm2_pitch_velocity;

float mass_all;
float g;
MatrixXd iden(3,3),mass(3,1),length(3,1),length_g(3,1),inertia(2,1),phi(6,1),kp(3,3),kd(3,3),q(3,1),q_dot(3,1),q_2dot(3,1),u(3,1),qd(3,1),qd_dot(3,1),qd_2dot(3,1),F(3,1),Hb(3,3),Hbj(3,3),Hj(3,3),Cb(3,1),Cj(3,1),Gb(3,1),Gj(3,1),J(3,3);

void callback_joint_states(const sensor_msgs::JointState &msg);
void cal_dynamics(MatrixXd q,MatrixXd q_dot,MatrixXd F);



int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_robot"); 

  ros::NodeHandle nh;
  ros::Publisher pub_joint1, pub_pitch_joint2;
   
  // set sampling time

  float T = 0.01;
  	
  ros::Rate rate(1/T);
  
  std_msgs::Float64 target_joint1, target_joint2;

  pub_joint1 = nh.advertise<std_msgs::Float64>("/my_robot/arm1_joint_effort_controller/command", 10);
  pub_pitch_joint2 = nh.advertise<std_msgs::Float64>("/my_robot/arm2_pitch_joint_effort_controller/command", 10);

  ros::Subscriber sub_joint_states = nh.subscribe("my_robot/joint_states",10,callback_joint_states);

	// set identity matrix

	iden << 1,0,0,
			0,1,0,
			0,0,1;

    // set robot data

	mass << 6.0,
			0.5,
			0.4;

	length << 0.15,
			  0.15,
			  0.15;

	length_g << 0.075,
			    0.045,
			    0.022;

	inertia << 0.0026,
		       0.0009;

	mass_all = mass(0,0)+mass(1,0)+mass(2,0);
	g = 9.8;
	
	// set phi in order to easily

	phi << mass(1,0)*length_g(1,0)*length_g(1,0),
		   mass(2,0)*length(1,0)*length(1,0),
		   mass(2,0)*length(1,0)*length_g(2,0),
		   mass(1,0)*length_g(1,0),
		   mass(2,0)*length(1,0),
		   mass(2,0)*length_g(2,0);

	kp << 0,0,0,
		  0,200,0,
		  0,0,100;

	kd << 0,0,0,
		  0,1,0,
		  0,0,1;

	
	
	// set timer

	float t = 0;

	ros::spinOnce(); 

	// calculation by eular

	cout << "first position" << endl;

  	while (ros::ok()) 
	{ 
		q(1,0) = M_PI/2 - arm2_pitch_position;
		q(2,0) = -arm1_position;
		q_dot(1,0) = arm2_pitch_velocity;
		q_dot(2,0) = arm1_velocity;

		if(t>500)
		{
			qd << 0,
				  80*M_PI/180,
				  20*M_PI/180;

			qd_dot << 0,
					  0,
					  0;

			qd_2dot << 0,
					   0,
					   0;
					   
			F << -6,
				 0,
			 	 1.0*mass_all*g;

			cal_dynamics(q,q_dot,F);

		}
		else
		{
			// set first target trajectory 

			qd << 0,
				  40*M_PI/180,
				  100*M_PI/180;

			qd_dot << 0,
					  0,
					  0;

			qd_2dot << 0,
					   0,
					   0;

			F << -20,
				 0,
			 	 1.0*mass_all*g;

			cal_dynamics(q,q_dot,F);

		}	
		

		// publish data
		
		target_joint1.data = -u(2,0);
		target_joint2.data = -u(1,0);

		pub_joint1.publish(target_joint1);        // knee 
    	pub_pitch_joint2.publish(target_joint2);  // hip pitch

		cout << "------------------------------------------------" << endl;
		cout << "hip_pitch torque " << -u(1,0) << " Nm" << endl;
		cout << "knee torque " << -u(2,0) << " Nm" << endl;

		cout << "q" << q << endl;
		cout << "q_dot" << q_dot << endl;	
		
		t ++;

		rate.sleep();
		ros::spinOnce(); 

  	}

	return 0;
}

void callback_joint_states(const sensor_msgs::JointState &msg)
{
	  arm1_position = msg.position[0];
	  arm2_pitch_position = msg.position[1];
	  arm1_velocity = msg.velocity[0];
	  arm2_pitch_velocity = msg.velocity[1];
}

void cal_dynamics(MatrixXd q,MatrixXd q_dot,MatrixXd F)
{
	// set dynamics

	Hb << mass_all,0,0,
		  0,mass_all,0,
		  0,0,mass_all;
			      
	Hbj << 0,(phi(3,0)+phi(4,0))*sin(q(1,0))+phi(5,0)*sin(q(1,0)+q(2,0)),phi(5,0)*sin(q(1,0)+q(2,0)),
		   -(phi(3,0)+phi(4,0))*sin(q(0,0)),0,0,
		   (phi(3,0)+phi(4,0))*sin(q(0,0)),-(phi(3,0)+phi(4,0))*cos(q(1,0))-phi(5,0)*cos(q(1,0)+q(2,0)),-phi(5,0)*cos(q(1,0)+q(2,0));

	Hj << phi(0,0)+phi(1,0)+inertia(0,0)+inertia(1,0),-(phi(0,0)+phi(1,0))*sin(q(0,0))*cos(q(1,0))-phi(2,0)*sin(q(0,0))*cos(q(1,0)+q(2,0)),-phi(2,0)*sin(q(0,0))*cos(q(1,0)+q(2,0)),
		  -(phi(0,0)+phi(1,0))*sin(q(0,0))*cos(q(1,0))-phi(2,0)*sin(q(0,0))*cos(q(1,0)+q(2,0)),phi(0,0)+phi(1,0)+2*phi(2,0)*cos(q(2,0))+inertia(0,0)+inertia(1,0),phi(2,0)*cos(q(2,0))+inertia(1,0),
		  -phi(2,0)*sin(q(0,0))*cos(q(1,0)+q(2,0)),phi(2,0)*cos(q(2,0))+inertia(1,0),inertia(1,0);


	Cb << (phi(3,0)+phi(4,0))*q_dot(1,0)*q_dot(1,0)*cos(q(1,0))+phi(5,0)*(q_dot(1,0)+q_dot(2,0))*(q_dot(1,0)+q_dot(2,0))*cos(q(1,0)+q(2,0)),
		  (phi(3,0)+phi(4,0))*q_dot(0,0)*q_dot(0,0)*sin(q(0,0)),
		  (phi(3,0)+phi(4,0))*(q_dot(0,0)*q_dot(0,0)*cos(q(0,0))+q_dot(1,0)*q_dot(1,0)*sin(q(1,0)))+phi(5,0)*(q_dot(1,0)+q_dot(2,0))*(q_dot(1,0)+q_dot(2,0))*sin(q(1,0)+q(2,0));

	Cj << (phi(0,0)+phi(1,0))*q_dot(1,0)*q_dot(1,0)*sin(q(0,0))*sin(q(1,0))+phi(2,0)*(q_dot(1,0)+q_dot(2,0))*(q_dot(1,0)+q_dot(2,0))*sin(q(0,0))*sin(q(1,0)+q(2,0)),
		  -(phi(0,0)+phi(1,0))*q_dot(0,0)*q_dot(0,0)*cos(q(0,0))*cos(q(1,0))-phi(2,0)*(q_dot(0,0)*q_dot(0,0)*cos(q(0,0))*cos(q(1,0)+q(2,0)))+2*q_dot(1,0)*q_dot(2,0)*sin(q(2,0))+q_dot(2,0)*q_dot(2,0)*sin(q(2,0)),
		  phi(2,0)*(-q_dot(0,0)*q_dot(0,0)*cos(q(0,0))*cos(q(1,0)+q(2,0)))+q_dot(1,0)*q_dot(1,0)*sin(q(2,0));

	Gb << 0,
		  0,
		  mass_all*g;

	Gj << (phi(3,0)+phi(4,0))*g*sin(q(0,0)),
		  -(phi(3,0)+phi(4,0))*g*cos(q(1,0))-phi(5,0)*g*cos(q(1,0)+q(2,0)),
		  -phi(5,0)*g*cos(q(1,0)+q(2,0));

	J << 0,length(1,0)*sin(q(1,0))+length(2,0)*sin(q(1,0)+q(2,0)),length(2,0)*sin(q(1,0)+q(2,0)),
		 -length(1,0)*cos(q(0,0)),0,0,
		 length(1,0)*sin(q(0,0)),-length(1,0)*cos(q(1,0))-length(2,0)*cos(q(1,0)+q(2,0)),-length(2,0)*cos(q(1,0)+q(2,0));

	// calculate q_2dot and torque

	q_2dot = qd_2dot-kp*(q-qd)-kd*(q_dot-qd_dot);

	Eigen::PartialPivLU<Eigen::MatrixXd> lu(Hb);

	u = (Hj-Hbj.transpose()*lu.solve(Hbj))*q_2dot+Hbj.transpose()*lu.solve(iden*F-Cb-Gb)+Cj+Gj-J.transpose()*F;
		
	if(u(1,0)<-40 or u(1,0)>40 or u(2,0)<-40 or u(2,0)>40)
	{
		u << 0,0,0;
	}
}
