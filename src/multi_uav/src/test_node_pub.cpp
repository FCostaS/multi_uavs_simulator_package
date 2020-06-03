#include "ros/ros.h"
#include <stdlib.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <gazebo_msgs/ModelStates.h>
using namespace std;

int main(int argc,char **argv)
{
  ros::init(argc,argv,"test_node");
  ros::NodeHandle nh;

  string group_name = ros::this_node::getNamespace();
  ros::Publisher publish_velocity = nh.advertise<geometry_msgs::TwistStamped>("/" + group_name + "/mavros/setpoint_velocity/cmd_vel",10);
  // ros::Subscriber suber = nh.subscribe("/gazebo/model_states", 10, &JSSJ::get_time, &computation);
 
  geometry_msgs::TwistStamped pubmsg;

  

  pubmsg.twist.linear.x = -0.1;
  pubmsg.twist.linear.y = 0;
  pubmsg.twist.linear.z = 0;
  pubmsg.header.frame_id = "world";
  

  ros::Rate rate(10); 
  int counter(0);
  while(ros::ok())
  {
    if(counter <= 50)
    {
      pubmsg.twist.linear.x = 0.6;
      pubmsg.twist.linear.y = -0.2;
      pubmsg.header.stamp = ros::Time::now();
      publish_velocity.publish(pubmsg);
    } else if (counter > 50)
    {
      pubmsg.twist.linear.x = 0.2;
      pubmsg.twist.linear.y = 0.6;
      pubmsg.header.stamp = ros::Time::now();
      publish_velocity.publish(pubmsg);
    }
    ros::spinOnce();
    rate.sleep();
    
    counter++;
  }

  return 0;
  
}

