#include "ros/ros.h"
#include <stdlib.h> 
#include <iostream>
#include <string>
#include <vector>
#include <cmath> 

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

class UAVMatrix {
	public:
		// int swarm_size;
		vector<vector<double>> posMatrix = vector<vector<double>>(3,vector<double>(2));
		vector<vector<double>> velMatrix = vector<vector<double>>(3,vector<double>(2));
		vector<vector<vector<double>>> rhoMatrix  = vector<vector<vector<double>>>(3,vector<vector<double>>(2,vector<double>(3)));
		void get_pos_vel(const gazebo_msgs::ModelStates& msg);
		void get_uav_rho();
 };
 
void UAVMatrix::get_pos_vel(const gazebo_msgs::ModelStates& msg)
{
	int uav_No;
	for(int i(0); i<4; i++)
	{
		uav_No = 0;
		if (!msg.name[i].compare("model_uav1")) uav_No = 1;
		if (!msg.name[i].compare("model_uav2")) uav_No = 2;
		if (!msg.name[i].compare("model_uav3")) uav_No = 3;
		switch (uav_No) {
			case 1:{
				UAVMatrix::posMatrix[0][0] = msg.pose[i].position.x;
				UAVMatrix::posMatrix[0][1] = msg.pose[i].position.y;
				UAVMatrix::velMatrix[0][0] = msg.twist[i].linear.x;
				UAVMatrix::velMatrix[0][1] = msg.twist[i].linear.y;
				break;
			}
			case 2:{
				UAVMatrix::posMatrix[1][0] = msg.pose[i].position.x;
				UAVMatrix::posMatrix[1][1] = msg.pose[i].position.y;
				UAVMatrix::velMatrix[1][0] = msg.twist[i].linear.x;
				UAVMatrix::velMatrix[1][1] = msg.twist[i].linear.y;
				break;
			}
			case 3:{
				UAVMatrix::posMatrix[2][0] = msg.pose[i].position.x;
				UAVMatrix::posMatrix[2][1] = msg.pose[i].position.y;
				UAVMatrix::velMatrix[2][0] = msg.twist[i].linear.x;
				UAVMatrix::velMatrix[2][1] = msg.twist[i].linear.y;
				break;
			}
		}
	}
}

void UAVMatrix::get_uav_rho()
{
	int i,j;
	for (i=0;i<=2;i++)
	{
		for (j=0;j<=i-1;j++)
		{
			UAVMatrix::rhoMatrix[i][j][0] = UAVMatrix::posMatrix[i][0] - UAVMatrix::posMatrix[j][0];
			UAVMatrix::rhoMatrix[i][j][1] = UAVMatrix::posMatrix[i][1] - UAVMatrix::posMatrix[j][1];
			UAVMatrix::rhoMatrix[i][j][2] = sqrt(pow(UAVMatrix::rhoMatrix[i][j][0],2)+pow(UAVMatrix::rhoMatrix[i][j][1],2));
		}
		for (j=i+1;j<=2;j++)
		{
			UAVMatrix::rhoMatrix[i][j-1][0] = UAVMatrix::posMatrix[i][0] - UAVMatrix::posMatrix[j][0];
			UAVMatrix::rhoMatrix[i][j-1][1] = UAVMatrix::posMatrix[i][1] - UAVMatrix::posMatrix[j][1];
			UAVMatrix::rhoMatrix[i][j-1][2] = sqrt(pow(UAVMatrix::rhoMatrix[i][j-1][0],2)+pow(UAVMatrix::rhoMatrix[i][j-1][1],2));
		}
	}
}  

int main(int argc, char** argv)
{
	ros::init(argc,argv,"artificial_potential_field_node");
	UAVMatrix uavmatrix;
	
	ros::NodeHandle nh;

	ros::Subscriber uav_pos_vel_sub = nh.subscribe ("/gazebo/model_states", 1, &UAVMatrix::get_pos_vel, &uavmatrix);
	
	ros::Publisher pub_vel_0 = nh.advertise<geometry_msgs::TwistStamped>("/group1/mavros/setpoint_velocity/cmd_vel", 1);
	ros::Publisher pub_vel_1 = nh.advertise<geometry_msgs::TwistStamped>("/group2/mavros/setpoint_velocity/cmd_vel", 1);
	ros::Publisher pub_vel_2 = nh.advertise<geometry_msgs::TwistStamped>("/group3/mavros/setpoint_velocity/cmd_vel", 1);
	
	geometry_msgs::TwistStamped velmsg_0;
	geometry_msgs::TwistStamped velmsg_1;
	geometry_msgs::TwistStamped velmsg_2;
	
	ros::Publisher pub_rho = nh.advertise<std_msgs::Float64MultiArray>("/uavs_rho", 10);
	std_msgs::Float64MultiArray msg_rho;
	msg_rho.data = vector<double>(3);

	velmsg_0.header.frame_id = "world";
	velmsg_1.header.frame_id = "world";
	velmsg_2.header.frame_id = "world";
	
	double a =5;
	double b =1;
	double c =1;
	double k =0.0048685;
	double rho_min =0.5;
	double rho_max =100;
	
	double V_d[3][2] = {0,0,0,0,0,0}; 
	
	ros::Rate rate(10); 
	while(ros::ok())
	{
		
		ros::spinOnce();
		
		cout<<"----------"<<endl;
		cout<<"posMatrix:"<<endl;
		cout<<uavmatrix.posMatrix[0][0]<<"  "<<uavmatrix.posMatrix[0][1]<<endl;
		cout<<uavmatrix.posMatrix[1][0]<<"  "<<uavmatrix.posMatrix[1][1]<<endl;
		cout<<uavmatrix.posMatrix[2][0]<<"  "<<uavmatrix.posMatrix[2][1]<<endl;
		cout<<endl;

		uavmatrix.get_uav_rho();
		
		cout<<"uav0 rho:"<<endl;
		cout<<uavmatrix.rhoMatrix[0][0][0]<<"  "<<uavmatrix.rhoMatrix[0][0][1]<<"  "<<uavmatrix.rhoMatrix[0][0][2]<<endl;
		cout<<uavmatrix.rhoMatrix[0][1][0]<<"  "<<uavmatrix.rhoMatrix[0][1][1]<<"  "<<uavmatrix.rhoMatrix[0][1][2]<<endl;
		cout<<"uav1 rho:"<<endl;
		cout<<uavmatrix.rhoMatrix[1][0][0]<<"  "<<uavmatrix.rhoMatrix[1][0][1]<<"  "<<uavmatrix.rhoMatrix[1][0][2]<<endl;
		cout<<uavmatrix.rhoMatrix[1][1][0]<<"  "<<uavmatrix.rhoMatrix[1][1][1]<<"  "<<uavmatrix.rhoMatrix[1][1][2]<<endl;
		cout<<"uav2 rho:"<<endl;
		cout<<uavmatrix.rhoMatrix[2][0][0]<<"  "<<uavmatrix.rhoMatrix[2][0][1]<<"  "<<uavmatrix.rhoMatrix[2][0][2]<<endl;
		cout<<uavmatrix.rhoMatrix[2][1][0]<<"  "<<uavmatrix.rhoMatrix[2][1][1]<<"  "<<uavmatrix.rhoMatrix[2][1][2]<<endl;
		cout<<endl;

		msg_rho.data[0] = uavmatrix.rhoMatrix[0][0][2];
		msg_rho.data[1] = uavmatrix.rhoMatrix[0][1][2];
		msg_rho.data[2] = uavmatrix.rhoMatrix[1][1][2];
		pub_rho.publish(msg_rho);

		for (int i(0);i<3;i++)
		{
			for (int j(0); j<2;j++)
			{
				if (uavmatrix.rhoMatrix[i][j][2] <= rho_max)
				{
					V_d[i][0] = V_d[i][0] + a*(-k*uavmatrix.rhoMatrix[i][j][2] + 
					b/c*exp(uavmatrix.rhoMatrix[i][j][2]/c) / pow((exp(uavmatrix.rhoMatrix[i][j][2]/c)-exp(rho_min/c)), 2) ) 
					*uavmatrix.rhoMatrix[i][j][0]/uavmatrix.rhoMatrix[i][j][2];
					
					V_d[i][1] = V_d[i][1] + a*(-k*uavmatrix.rhoMatrix[i][j][2] + 
					b/c*exp(uavmatrix.rhoMatrix[i][j][2]/c) / pow((exp(uavmatrix.rhoMatrix[i][j][2]/c)-exp(rho_min/c)), 2) ) 
					*uavmatrix.rhoMatrix[i][j][1]/uavmatrix.rhoMatrix[i][j][2];
				} 
				/*else{
					V_d[i][0] = V_d[i][0] + 0.5*(-0.2*uavmatrix.rhoMatrix[i][j][2]+1.6)*uavmatrix.rhoMatrix[i][j][0]/uavmatrix.rhoMatrix[i][j][2];
					V_d[i][1] = V_d[i][1] + 0.5*(-0.2*uavmatrix.rhoMatrix[i][j][2]+1.6)*uavmatrix.rhoMatrix[i][j][1]/uavmatrix.rhoMatrix[i][j][2];
				};*/
			}
		}   
		
		cout<<"delta of velocity:"<<endl;
		cout<<V_d[0][0]<<"  "<<V_d[0][1]<<endl;
		cout<<V_d[1][0]<<"  "<<V_d[1][1]<<endl;
		cout<<V_d[2][0]<<"  "<<V_d[2][1]<<endl;
		cout<<endl;

		/* velmsg_0.header.stamp =ros::Time::now();
		velmsg_0.twist.linear.x = -( V_d[0][1] + uavmatrix.velMatrix[0][1] );
		velmsg_0.twist.linear.y = V_d[0][0] + uavmatrix.velMatrix[0][0];
		pub_vel_0.publish(velmsg_0);
		
		velmsg_1.header.stamp =ros::Time::now();
		velmsg_1.twist.linear.x = -( V_d[1][1] + uavmatrix.velMatrix[1][1] );
		velmsg_1.twist.linear.y = V_d[1][0] + uavmatrix.velMatrix[1][0];
		pub_vel_1.publish(velmsg_1);
		
		velmsg_2.header.stamp =ros::Time::now();
		velmsg_2.twist.linear.x = -( V_d[2][1] + uavmatrix.velMatrix[2][1] );
		velmsg_2.twist.linear.y = V_d[2][0] + uavmatrix.velMatrix[2][0];
		pub_vel_2.publish(velmsg_2); */
		
		velmsg_0.header.stamp =ros::Time::now();
		velmsg_0.twist.linear.x = -( V_d[0][1] + 0 );
		velmsg_0.twist.linear.y = V_d[0][0] + 0.5;
		pub_vel_0.publish(velmsg_0);
		
		velmsg_1.header.stamp =ros::Time::now();
		velmsg_1.twist.linear.x = -( V_d[1][1] + 0 );
		velmsg_1.twist.linear.y = V_d[1][0] + 0.5;
		pub_vel_1.publish(velmsg_1);
		
		velmsg_2.header.stamp =ros::Time::now();
		velmsg_2.twist.linear.x = -( V_d[2][1] + 0 );
		velmsg_2.twist.linear.y = V_d[2][0] + 0.5;
		pub_vel_2.publish(velmsg_2);

		cout<<"uav velocity published"<<endl;
		cout<<"uav0:"<<velmsg_0.twist.linear.x<<"  "<<velmsg_0.twist.linear.y<<endl;
		cout<<"uav1:"<<velmsg_1.twist.linear.x<<"  "<<velmsg_1.twist.linear.y<<endl;
		cout<<"uav2:"<<velmsg_2.twist.linear.x<<"  "<<velmsg_2.twist.linear.y<<endl;
		cout<<endl<<endl;    

		for(int i(0);i<3;i++)
		{
			V_d[i][0] = 0;
			V_d[i][1] = 0;
		}  

		rate.sleep(); 
	}
	
	return 0;
}

