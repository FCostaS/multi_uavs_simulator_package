#include "ros/ros.h"
#include <stdlib.h> 
#include <iostream>
#include <string>
#include <vector>
#include <cmath> 

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

using namespace std;

class UAVPos 
{
public:
	vector<vector<double>> posMatrix = vector<vector<double>>(3,vector<double>(2));
	void get_uavs_position(const gazebo_msgs::ModelStates& msg);
};

void UAVPos::get_uavs_position (const gazebo_msgs::ModelStates& msg)
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
				UAVPos::posMatrix[0][0] = msg.pose[i].position.x;
				UAVPos::posMatrix[0][1] = msg.pose[i].position.y;
				break;
			}
			case 2:{
				UAVPos::posMatrix[1][0] = msg.pose[i].position.x;
				UAVPos::posMatrix[1][1] = msg.pose[i].position.y;
				break;
			}
			case 3:{
				UAVPos::posMatrix[2][0] = msg.pose[i].position.x;
				UAVPos::posMatrix[2][1] = msg.pose[i].position.y;
				break;
			}
		}
	}
}

int main(int argc, char** argv) 
{
	ros::init(argc,argv,"leader_follower_node");
	UAVPos uavpos;
	ros::NodeHandle nh;
	
	ros::Subscriber uav_pos_sub = nh.subscribe ("/gazebo/model_states", 1, &UAVPos::get_uavs_position, &uavpos);
	
	ros::Publisher pub_vel_l  = nh.advertise<geometry_msgs::TwistStamped>("/group1/mavros/setpoint_velocity/cmd_vel", 1);
	ros::Publisher pub_vel_f1 = nh.advertise<geometry_msgs::TwistStamped>("/group2/mavros/setpoint_velocity/cmd_vel", 1);
	ros::Publisher pub_vel_f2 = nh.advertise<geometry_msgs::TwistStamped>("/group3/mavros/setpoint_velocity/cmd_vel", 1);
	
	geometry_msgs::TwistStamped velmsg_l;
	geometry_msgs::TwistStamped velmsg_f1;
	geometry_msgs::TwistStamped velmsg_f2;
	
	////////
	ros::Publisher pub_distance_error_1 = nh.advertise<std_msgs::Float64>("/follower1/distance_error", 10);
	ros::Publisher pub_distance_error_2 = nh.advertise<std_msgs::Float64>("/follower2/distance_error", 10);
	std_msgs::Float64 distance_error1;
	std_msgs::Float64 distance_error2;

	ros::Publisher pub_vel_f1_y = nh.advertise<std_msgs::Float64>("/follower1_command/y_axis", 10);
	ros::Publisher pub_vel_f2_y = nh.advertise<std_msgs::Float64>("/follower2_command/y_axis", 10);
	std_msgs::Float64 msg_y1;
	std_msgs::Float64 msg_y2; 
	////////

	double V_l = 0.5;
	double R_d = 0.3;
	double R_c = 6;
	double V_max = 1.5;
	
	double v_f1 =0;
	double v_f2 =0;
	
	double f1_target_pos[2] = {0,0};
	double f2_target_pos[2] = {0,0};
	
	double theta1;
	double theta2;

	double distance1 = 0;
	double distance2 = 0;
	
	velmsg_l.twist.linear.x = 0;
	velmsg_l.twist.linear.y = V_l;
	
	ros::Rate rate(10);
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
		f1_target_pos[0] = uavpos.posMatrix[0][0] - 1;
		f1_target_pos[1] = uavpos.posMatrix[0][1] + sqrt(3);
		f2_target_pos[0] = uavpos.posMatrix[0][0] - 1;
		f2_target_pos[1] = uavpos.posMatrix[0][1] - sqrt(3);
		
		cout<<"f1 target pos: "<<f1_target_pos[0]<<"  "<<f1_target_pos[1]<<endl;
		cout<<"f2 target pos: "<<f2_target_pos[0]<<"  "<<f2_target_pos[1]<<endl;

		//target angle of followers
		theta1 = atan2((f1_target_pos[1]-uavpos.posMatrix[1][1]),(f1_target_pos[0]-uavpos.posMatrix[1][0])); // in rads
		theta2 = atan2((f2_target_pos[1]-uavpos.posMatrix[2][1]),(f2_target_pos[0]-uavpos.posMatrix[2][0])); // in rads
		
		cout<<"f1 f2 target angle: "<<theta1<<"  "<<theta2<<endl;

		//the distance between target pos and follower
		distance1 = sqrt(pow(f1_target_pos[1]-uavpos.posMatrix[1][1],2)+pow(f1_target_pos[0]-uavpos.posMatrix[1][0],2));
		distance2 = sqrt(pow(f2_target_pos[1]-uavpos.posMatrix[2][1],2)+pow(f2_target_pos[0]-uavpos.posMatrix[2][0],2));
		
		cout<<"f1 f2 distance: "<<distance1<<"  "<<distance2<<endl;

		distance_error1.data = distance1;
		pub_distance_error_1.publish(distance_error1);
		distance_error2.data = distance2;
		pub_distance_error_2.publish(distance_error2);

		//choose the vel of follower1 and publish vel
		if (distance1 >= R_c) {
			v_f1 = V_max;
			velmsg_f1.twist.linear.x = -v_f1*sin(theta1);
			velmsg_f1.twist.linear.y = v_f1*cos(theta1);
		}
		else if (distance1<R_c && distance1>=R_d)
		{
			v_f1 = distance1*(V_max-V_l)/(R_c-R_d) + (V_l*R_c-V_max*R_d)/(R_c-R_d);
			velmsg_f1.twist.linear.x = -v_f1*sin(theta1);
			velmsg_f1.twist.linear.y = v_f1*cos(theta1);
		} 
		else 
		{
			velmsg_f1.twist.linear.x = velmsg_l.twist.linear.x;
			velmsg_f1.twist.linear.y = velmsg_l.twist.linear.y;
		}
		velmsg_f1.header.stamp = ros::Time::now();
		velmsg_f1.header.frame_id = "world";
		pub_vel_f1.publish(velmsg_f1); 
		
		cout<<"v_f1: "<<v_f1<<endl;
		cout<<"f1 vel: "<<velmsg_f1.twist.linear.x<<"  "<<velmsg_f1.twist.linear.y<<endl;

		msg_y1.data = velmsg_f1.twist.linear.x;
		pub_vel_f1_y.publish(msg_y1);


		//choose the vel of follower2 and publish vel
		if (distance2 >= R_c) {
			v_f2 = V_max;
			velmsg_f2.twist.linear.x = -v_f2*sin(theta2);
			velmsg_f2.twist.linear.y = v_f2*cos(theta2);
		}
		else if (distance2<R_c && distance2>=R_d) 
		{
			v_f2 = distance2*(V_max-V_l)/(R_c-R_d) + (V_l*R_c-V_max*R_d)/(R_c-R_d);
			velmsg_f2.twist.linear.x = -v_f2*sin(theta2);
			velmsg_f2.twist.linear.y = v_f2*cos(theta2);
		}
		else
		{
			velmsg_f2.twist.linear.x = velmsg_l.twist.linear.x;
			velmsg_f2.twist.linear.y = velmsg_l.twist.linear.y;
		}
		velmsg_f2.header.stamp = ros::Time::now();
		velmsg_f2.header.frame_id = "world";
		pub_vel_f2.publish(velmsg_f2);
		
		cout<<"v_f2: "<<v_f2<<endl;
		cout<<"f2 vel: "<<velmsg_f2.twist.linear.x<<"  "<<velmsg_f2.twist.linear.y<<endl;
		cout<<endl<<endl;
		
		msg_y2.data = velmsg_f2.twist.linear.x;
		pub_vel_f2_y.publish(msg_y2);

		rate.sleep();
	}
	return 0;
}
