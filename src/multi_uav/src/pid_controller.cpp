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

class UAVPos {
	public:
	vector<vector<double>> posMatrix = vector<vector<double>>(5,vector<double>(2));
	vector<vector<double>> velMatrix = vector<vector<double>>(5,vector<double>(2));
	void get_pos_vel(const gazebo_msgs::ModelStates& msg);
};

void UAVPos::get_pos_vel(const gazebo_msgs::ModelStates& msg)
{
	int uav_No;
	for(int i(0); i<6; i++)
	{
		uav_No = 0;
		if (!msg.name[i].compare("model_uav1")) uav_No = 1;
		if (!msg.name[i].compare("model_uav2")) uav_No = 2;
		if (!msg.name[i].compare("model_uav3")) uav_No = 3;
		if (!msg.name[i].compare("model_uav4")) uav_No = 4;
		if (!msg.name[i].compare("model_uav5")) uav_No = 5;
		switch (uav_No) {
			case 1:{
				UAVPos::posMatrix[0][0] = msg.pose[i].position.x;
				UAVPos::posMatrix[0][1] = msg.pose[i].position.y;
				UAVPos::velMatrix[0][0] = msg.twist[i].linear.x;
				UAVPos::velMatrix[0][1] = msg.twist[i].linear.y;
				break;
			}
			case 2:{
				UAVPos::posMatrix[1][0] = msg.pose[i].position.x;
				UAVPos::posMatrix[1][1] = msg.pose[i].position.y;
				UAVPos::velMatrix[1][0] = msg.twist[i].linear.x;
				UAVPos::velMatrix[1][1] = msg.twist[i].linear.y;
				break;
			}
			case 3:{
				UAVPos::posMatrix[2][0] = msg.pose[i].position.x;
				UAVPos::posMatrix[2][1] = msg.pose[i].position.y;
				UAVPos::velMatrix[2][0] = msg.twist[i].linear.x;
				UAVPos::velMatrix[2][1] = msg.twist[i].linear.y;
				break;
			}
			case 4:{
				UAVPos::posMatrix[3][0] = msg.pose[i].position.x;
				UAVPos::posMatrix[3][1] = msg.pose[i].position.y;
				UAVPos::velMatrix[3][0] = msg.twist[i].linear.x;
				UAVPos::velMatrix[3][1] = msg.twist[i].linear.y;
				break;
			}
			case 5:{
				UAVPos::posMatrix[4][0] = msg.pose[i].position.x;
				UAVPos::posMatrix[4][1] = msg.pose[i].position.y;
				UAVPos::velMatrix[4][0] = msg.twist[i].linear.x;
				UAVPos::velMatrix[4][1] = msg.twist[i].linear.y;
				break;
			}
		}
	}
}

int main(int argc, char** argv) 
{
	ros::init(argc,argv,"pid_controller_node");
	UAVPos uavpos;
	ros::NodeHandle nh;
	ros::Subscriber uav_posandvel_sub = nh.subscribe ("/gazebo/model_states", 1, &UAVPos::get_pos_vel, &uavpos);
	
	ros::Publisher pub_err = nh.advertise<std_msgs::Float64MultiArray>("/pos_err_x", 10);
	std_msgs::Float64MultiArray pos_err_x_array;
	pos_err_x_array.data = vector<double>(4);

	ros::Publisher pub_vel_l  = nh.advertise<geometry_msgs::TwistStamped>("/group1/mavros/setpoint_velocity/cmd_vel", 1);
	ros::Publisher pub_vel_f1 = nh.advertise<geometry_msgs::TwistStamped>("/group2/mavros/setpoint_velocity/cmd_vel", 1);
	ros::Publisher pub_vel_f2 = nh.advertise<geometry_msgs::TwistStamped>("/group3/mavros/setpoint_velocity/cmd_vel", 1);
	ros::Publisher pub_vel_f3 = nh.advertise<geometry_msgs::TwistStamped>("/group4/mavros/setpoint_velocity/cmd_vel", 1);
	ros::Publisher pub_vel_f4 = nh.advertise<geometry_msgs::TwistStamped>("/group5/mavros/setpoint_velocity/cmd_vel", 1);
	
	geometry_msgs::TwistStamped velmsg_l;
	geometry_msgs::TwistStamped velmsg_f1;
	geometry_msgs::TwistStamped velmsg_f2;
	geometry_msgs::TwistStamped velmsg_f3;
	geometry_msgs::TwistStamped velmsg_f4;
	
	int i;
	double f_target_pos[4][2] = {0,0,0,0,0,0,0,0};

	double f =10;
	double T =1/f;

	double Kp =0.2;
	double Ti =2.0;
	double Td =0.45;
	
	double Kp_x = Kp;
	double Ki_x = T*Kp/Ti;
	double Kd_x = Kp*Td;
	
	double Kp_y = Kp;
	double Ki_y = T*Kp/Ti;
	double Kd_y = Kp*Td;
	
	double err_sum[4][2] = {0,0,0,0,0,0,0,0};
	
	double err[4][2] = {0,0,0,0,0,0,0,0};
	double Vl_x = 0;
	double Vl_y = 0.5;
	
	velmsg_l.twist.linear.x = Vl_x;
	velmsg_l.twist.linear.y = Vl_y;
	
	ros::Rate rate(f);
	while(ros::ok())
	{
		ros::spinOnce();

		cout<<"----------"<<endl;
		cout<<"posMatrix"<<endl;
		cout<<uavpos.posMatrix[0][0]<<"  "<<uavpos.posMatrix[0][1]<<endl;
		cout<<uavpos.posMatrix[1][0]<<"  "<<uavpos.posMatrix[1][1]<<endl;
		cout<<uavpos.posMatrix[2][0]<<"  "<<uavpos.posMatrix[2][1]<<endl;
		cout<<endl;
		
		//publish leader's velocity!!
		velmsg_l.header.stamp = ros::Time::now();
		velmsg_l.header.frame_id = "world";
		pub_vel_l.publish(velmsg_l);
		
		// target postion of followers
		f_target_pos[0][0] = uavpos.posMatrix[0][0];
		f_target_pos[0][1] = uavpos.posMatrix[0][1] + 2;
		f_target_pos[1][0] = uavpos.posMatrix[0][0];
		f_target_pos[1][1] = uavpos.posMatrix[0][1] - 2;
		f_target_pos[2][0] = uavpos.posMatrix[0][0];
		f_target_pos[2][1] = uavpos.posMatrix[0][1] + 4;
		f_target_pos[3][0] = uavpos.posMatrix[0][0];
		f_target_pos[3][1] = uavpos.posMatrix[0][1] - 4;

		//cout<<"f1 target pos: "<<f1_target_pos[0]<<"  "<<f1_target_pos[1]<<endl;
		//cout<<"f2 target pos: "<<f2_target_pos[0]<<"  "<<f2_target_pos[1]<<endl;
		
		//position errors of f1,f2
		for(i=0; i<=3; i++)
		{
			err[i][0] = f_target_pos[i][0] - uavpos.posMatrix[i+1][0];
			err[i][1] = f_target_pos[i][1] - uavpos.posMatrix[i+1][1];
		}

		cout<<"f1 pos err:  "<<err[0][0]<<"  "<<err[0][1]<<endl;
		cout<<"f2 pos err:  "<<err[1][0]<<"  "<<err[1][1]<<endl;
		cout<<"f3 pos err:  "<<err[2][0]<<"  "<<err[2][1]<<endl;
		cout<<"f4 pos err:  "<<err[3][0]<<"  "<<err[3][1]<<endl;
		
		for(i=0; i<=3;i++)
		{
			pos_err_x_array.data[i] = err[i][0];
		};
		pub_err.publish(pos_err_x_array);

		// error integration
		for(i=0; i<=3; i++)
		{
			err_sum[i][0] = err_sum[i][0] + err[i][0];
			err_sum[i][1] = err_sum[i][1] + err[i][1];
		}

		//velocity of f1,f2
		velmsg_f1.twist.linear.x = -( Kp_y*err[0][1] + Ki_y*err_sum[0][1] + Kd_y*(Vl_y-uavpos.velMatrix[1][1]) );
		velmsg_f1.twist.linear.y = Kp_x*err[0][0] + Ki_x*err_sum[0][0] + Kd_x*(Vl_x-uavpos.velMatrix[1][0]);
		velmsg_f2.twist.linear.x = -( Kp_y*err[1][1] + Ki_y*err_sum[1][1] + Kd_y*(Vl_y-uavpos.velMatrix[2][1]) );
		velmsg_f2.twist.linear.y = Kp_x*err[1][0] + Ki_x*err_sum[1][0] + Kd_x*(Vl_x-uavpos.velMatrix[2][0]);
		velmsg_f3.twist.linear.x = -( Kp_y*err[2][1] + Ki_y*err_sum[2][1] + Kd_y*(Vl_y-uavpos.velMatrix[3][1]) );
		velmsg_f3.twist.linear.y = Kp_x*err[2][0] + Ki_x*err_sum[2][0] + Kd_x*(Vl_x-uavpos.velMatrix[3][0]);
		velmsg_f4.twist.linear.x = -( Kp_y*err[3][1] + Ki_y*err_sum[3][1] + Kd_y*(Vl_y-uavpos.velMatrix[4][1]) );
		velmsg_f4.twist.linear.y = Kp_x*err[3][0] + Ki_x*err_sum[3][0] + Kd_x*(Vl_x-uavpos.velMatrix[4][0]);

		cout<<"f1 target vel:  "<<velmsg_f1.twist.linear.x<<"  "<<velmsg_f1.twist.linear.y<<endl;
		cout<<"f2 target vel:  "<<velmsg_f2.twist.linear.x<<"  "<<velmsg_f2.twist.linear.y<<endl;
		
		//publish the velocity of f1
		velmsg_f1.header.stamp = ros::Time::now();
		velmsg_f1.header.frame_id = "world";
		pub_vel_f1.publish(velmsg_f1);
		
		//publish the velocity of f2
		velmsg_f2.header.stamp = ros::Time::now();
		velmsg_f2.header.frame_id = "world";
		pub_vel_f2.publish(velmsg_f2);

		velmsg_f3.header.stamp = ros::Time::now();
		velmsg_f3.header.frame_id = "world";
		pub_vel_f3.publish(velmsg_f3);

		velmsg_f4.header.stamp = ros::Time::now();
		velmsg_f4.header.frame_id = "world";
		pub_vel_f4.publish(velmsg_f4);
		
		rate.sleep();
		 
	}
	return 0;
}
