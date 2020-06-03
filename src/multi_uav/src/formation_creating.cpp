#include "ros/ros.h"
#include <stdlib.h> 
#include <iostream>
#include <string>
#include <vector>
#include <cmath> 


#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>



using namespace std;

class Formation
{
	public: 
		int columns = 3;
		vector<vector<double>> posMatrix = vector<vector<double>>(3,vector<double>(columns));
		vector<vector<double>> velMatrix = vector<vector<double>>(3,vector<double>(columns));
		vector<vector<double>> Xi = vector<vector<double>>(6,vector<double>(columns));
		
		void get_uav_pos_and_vel(const gazebo_msgs::ModelStates& msg);   
		//void Xi_creating();
		 
};

void Formation::get_uav_pos_and_vel(const gazebo_msgs::ModelStates& msg) 
{
	
	int uav_No;
	for(int i(0); i<=columns; i++)
	{
		uav_No = 0;
		if (!msg.name[i].compare("model_uav1")) uav_No = 1;
		if (!msg.name[i].compare("model_uav2")) uav_No = 2;
		if (!msg.name[i].compare("model_uav3")) uav_No = 3;
		switch (uav_No) {
			case 1:{
				Formation::posMatrix[0][0] = msg.pose[i].position.x;
				Formation::posMatrix[1][0] = msg.pose[i].position.y;
				Formation::posMatrix[2][0] = msg.pose[i].position.z;
				
				Formation::velMatrix[0][0] = msg.twist[i].linear.x;
				Formation::velMatrix[1][0] = msg.twist[i].linear.y;
				Formation::velMatrix[2][0] = msg.twist[i].linear.z;
				break;
			}
			case 2:{
				Formation::posMatrix[0][1] = msg.pose[i].position.x;
				Formation::posMatrix[1][1] = msg.pose[i].position.y;
				Formation::posMatrix[2][1] = msg.pose[i].position.z;
				
				Formation::velMatrix[0][1] = msg.twist[i].linear.x;
				Formation::velMatrix[1][1] = msg.twist[i].linear.y;
				Formation::velMatrix[2][1] = msg.twist[i].linear.z;
				break;
			}
			case 3:{
				Formation::posMatrix[0][2] = msg.pose[i].position.x;
				Formation::posMatrix[1][2] = msg.pose[i].position.y;
				Formation::posMatrix[2][2] = msg.pose[i].position.z;
				
				Formation::velMatrix[0][2] = msg.twist[i].linear.x;
				Formation::velMatrix[1][2] = msg.twist[i].linear.y;
				Formation::velMatrix[2][2] = msg.twist[i].linear.z;
				break;
			}
		}
	}
	Formation::Xi[0] = Formation::posMatrix[0];
	Formation::Xi[2] = Formation::posMatrix[1];
	Formation::Xi[4] = Formation::posMatrix[2];
		
	Formation::Xi[1] = Formation::velMatrix[0];
	Formation::Xi[3] = Formation::velMatrix[1];
	Formation::Xi[5] = Formation::velMatrix[2];
	
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"formation_creating_node");
	Formation formation;  
	ros::NodeHandle nh;
	
	ros::Subscriber uav_pos_and_vel_sub = nh.subscribe ("/gazebo/model_states", 1, &Formation::get_uav_pos_and_vel, &formation);
	
     
	string uav_id_string;
	ros::NodeHandle pn("~");
	pn.getParam("uav_id",uav_id_string); 
	int uav_id = atoi(uav_id_string.c_str()); 
	uav_id = uav_id - 1;
	double x_offset =strtod(argv[2],NULL);
	double y_offset =strtod(argv[4],NULL);
	double z_offset =strtod(argv[6],NULL);
	string group_name = ros::this_node::getNamespace();
	
	
	ros::Publisher pub_pos = nh.advertise<geometry_msgs::PoseStamped>("/" + group_name + "/mavros/setpoint_position/local", 10);
	ros::Publisher pub_vel = nh.advertise<geometry_msgs::TwistStamped>("/" + group_name + "/mavros/setpoint_velocity/cmd_vel", 10);
	
	
	
	geometry_msgs::PoseStamped posmsg;
	geometry_msgs::TwistStamped velmsg; 
	
 
	
	
	double sigma = 0.3;
	int w[3][3] = {0,1,1,1,0,0,1,0,0}; 
	double K[3][6] = {0.4, 0.4, 0, 0, 0, 0,
					  0, 0, 0.4, 0.4, 0, 0,
					  0, 0, 0, 0, 0.4, 0.4} ;  
	double f[3][6] = {{2*sqrt(3), 0, 0, 0, 0, 0}, {0, 0, 2, 0, 0, 0},{0, 0, -2, 0, 0, 0}};  
	double accel[3] = {0, 0, 0};  
	double middle_Matrix[6] = {0, 0, 0, 0, 0, 0};  
	
	ros::Rate rate(10);
	
	while(ros::ok())
	{
		ros::spinOnce();
		cout << endl;
		cout << "------------------" << endl;
		cout << "posMatrix:" << endl;
		cout << formation.posMatrix[0][0] << "  " << formation.posMatrix[0][1] << "  " << formation.posMatrix[0][2] << endl; 
		cout << formation.posMatrix[1][0] << "  " << formation.posMatrix[1][1] << "  " << formation.posMatrix[1][2] << endl;
		cout << formation.posMatrix[2][0] << "  " << formation.posMatrix[2][1] << "  " << formation.posMatrix[2][2] << endl;
		cout << endl;
		cout << "velMatrix:" << endl;
		cout << formation.velMatrix[0][0] << "  " << formation.velMatrix[0][1] << "  " << formation.velMatrix[0][2] << endl;
		cout << formation.velMatrix[1][0] << "  " << formation.velMatrix[1][1] << "  " << formation.velMatrix[1][2] << endl;
		cout << formation.velMatrix[2][0] << "  " << formation.velMatrix[2][1] << "  " << formation.velMatrix[2][2] << endl;
		cout << endl;
		cout << "XiMatrix:" << endl;
		cout << formation.Xi[0][0] << "  " << formation.Xi[0][1] << "  " << formation.Xi[0][2] <<endl;
		cout << formation.Xi[1][0] << "  " << formation.Xi[1][1] << "  " << formation.Xi[1][2] <<endl;
		cout << formation.Xi[2][0] << "  " << formation.Xi[2][1] << "  " << formation.Xi[2][2] <<endl;
		cout << formation.Xi[3][0] << "  " << formation.Xi[3][1] << "  " << formation.Xi[3][2] <<endl;
		cout << formation.Xi[4][0] << "  " << formation.Xi[4][1] << "  " << formation.Xi[4][2] <<endl;
		cout << formation.Xi[5][0] << "  " << formation.Xi[5][1] << "  " << formation.Xi[5][2] <<endl; 
		cout << endl;

		
		posmsg.pose.position.y = (formation.posMatrix[0][uav_id] + sigma*formation.velMatrix[0][uav_id]) - x_offset;
		posmsg.pose.position.x = y_offset - (formation.posMatrix[1][uav_id] + sigma*formation.velMatrix[1][uav_id]);
		posmsg.pose.position.z = (formation.posMatrix[2][uav_id] + sigma*formation.velMatrix[2][uav_id]) - z_offset;
        

		posmsg.header.stamp = ros::Time::now();
		posmsg.header.frame_id = "world";
		// pub_pos.publish(posmsg);
		
		
		
		for (int j(0); j<formation.columns; j++) 
		{
			for (int k(0); k<6; k++)
			{
				middle_Matrix[k] = w[uav_id][j] * ( formation.Xi[k][j] - formation.Xi[k][uav_id] - f[uav_id][k] + f[j][k] ) + middle_Matrix[k];
			}
			 
		};
		
		for (int i(0); i<6; i++)
		{
			accel[0] = accel[0] + K[0][i]*middle_Matrix[i];  
			accel[1] = accel[1] + K[1][i]*middle_Matrix[i];
			accel[2] = accel[2] + K[2][i]*middle_Matrix[i];
		};
		cout << "uav_id:" << uav_id <<endl;
		cout << "accel:  " << accel[0] << "  " << accel[1] << "  " << accel[2] << endl;
		cout << "middle_Matrix:  "<<middle_Matrix[0]<<"  "<<middle_Matrix[1]<<"  "<<middle_Matrix[2]<<"  "<<middle_Matrix[3]<<"  "<<middle_Matrix[4]<<"  "<<middle_Matrix[5]<<endl; 
		
		
		velmsg.twist.linear.y = formation.velMatrix[0][uav_id] + sigma*accel[0];        //
		velmsg.twist.linear.x = -( formation.velMatrix[1][uav_id] + sigma*accel[1] );   // 
		velmsg.twist.linear.z = formation.velMatrix[2][uav_id] + sigma*accel[2];        //
		velmsg.header.stamp = ros::Time::now();
		velmsg.header.frame_id = "world";
		pub_vel.publish(velmsg); 
		cout << "xyz_offset " << x_offset << "  " << y_offset << "  " << z_offset << endl;
		cout << "target_postion:" << posmsg.pose.position.x << "  " << posmsg.pose.position.y << "  " << posmsg.pose.position.z << endl;
		cout << "target_velocity:"  << velmsg.twist.linear.x << "  " << velmsg.twist.linear.y << "  " << velmsg.twist.linear.z << endl;
		
		for (int i(0); i<6; i++)
		{
			middle_Matrix[i] = 0;
		};
		for (int i(0); i<3; i++)
		{
			accel[i] = 0;
		};
		cout << "middle_Matrix after clearing:"<<middle_Matrix[0]<<"  "<<middle_Matrix[1]<<"  "<<middle_Matrix[2]<<"  "<<middle_Matrix[3]<<"  "<<middle_Matrix[4]<<"  "<<middle_Matrix[5]<<endl;
		cout << "accel vector after clearing:"<<accel[0]<<"  "<<accel[1]<<"  "<<accel[2]<<endl;
		cout << "------------------" << endl;
		cout << endl << endl << endl;
		// ros::spinOnce();
		rate.sleep();
		
	}
	return 0;

}
