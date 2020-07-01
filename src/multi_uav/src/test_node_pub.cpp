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

  // send acceleration setpoint   --------does not work
  ros::Publisher publish_accel = nh.advertise<geometry_msgs::Vector3Stamped>("/" + group_name + "/mavros/setpoint_accel/accel",10);
  geometry_msgs::Vector3Stamped accel_msg;
  accel_msg.header.frame_id = "world";
  accel_msg.vector.x = 1;
  accel_msg.vector.y = 0;
  accel_msg.vector.z = 0;

  // setpoint_raw/local : Local position, velocity and acceleration setpoint. ----------position(y),velocity(y),accel(n),yaw and yaw rate(n)
  ros::Publisher publish_setpointraw = nh.advertise<mavros_msgs::PositionTarget>("/" + group_name + "/mavros/setpoint_raw/target_local", 10);
  mavros_msgs::PositionTarget setpointraw_msg;
  setpointraw_msg.header.frame_id = "world";
  setpointraw_msg.coordinate_frame = 1;
  setpointraw_msg.type_mask = 2047;
  setpointraw_msg.position.x = 5;
  setpointraw_msg.position.y = 1;
  setpointraw_msg.position.z = 5;
  setpointraw_msg.acceleration_or_force.x = 1;
  setpointraw_msg.acceleration_or_force.y = 0;
  setpointraw_msg.acceleration_or_force.z = 0;
  setpointraw_msg.yaw_rate = 2;
  setpointraw_msg.yaw = 50;
  

  // send angular velocity   ------------does not work
  ros::Publisher publish_augular_velocity = nh.advertise<geometry_msgs::TwistStamped>("/" + group_name + "/mavros/setpoint_attitude/cmd_vel", 10);
  geometry_msgs::TwistStamped angular_velocity_msg;
  angular_velocity_msg.header.frame_id = "world";
  angular_velocity_msg.twist.linear.x = 0;
  angular_velocity_msg.twist.linear.y = 0;
  angular_velocity_msg.twist.linear.z = 1;
  angular_velocity_msg.twist.angular.x = 0;
  angular_velocity_msg.twist.angular.y = 0;
  angular_velocity_msg.twist.angular.z = 1;
  

  // send attitude setpoint   -----------does not work
  ros::Publisher publish_attitude = nh.advertise<geometry_msgs::PoseStamped>("/" + group_name + "/mavros/setpoint_attitude/attitude", 10);
  geometry_msgs::PoseStamped attitude_msg;
  attitude_msg.header.frame_id = "world";
  attitude_msg.pose.position.x = 1;
  attitude_msg.pose.position.y = 0;
  attitude_msg.pose.position.z = 0;
  attitude_msg.pose.orientation.x = 0;
  attitude_msg.pose.orientation.y = 0;
  attitude_msg.pose.orientation.z = 0.383;
  attitude_msg.pose.orientation.w = 0.924;
  

  // send velocity           -------------it works!!!
  ros::Publisher publish_velocity = nh.advertise<geometry_msgs::TwistStamped>("/" + group_name + "/mavros/setpoint_velocity/cmd_vel", 10);
  geometry_msgs::TwistStamped velocity_msg;
  velocity_msg.header.frame_id = "world";
  velocity_msg.twist.linear.x = 0;
  velocity_msg.twist.linear.y = 0;
  velocity_msg.twist.linear.z = 0;
  velocity_msg.twist.angular.x = 0;
  velocity_msg.twist.angular.y = 0;
  velocity_msg.twist.angular.z = 1;
  

  // send position        -----------it's wired, but it has a little effort
  ros::Publisher publish_position = nh.advertise<geometry_msgs::PoseStamped>("/" + group_name + "/mavros/setpoint_position/local", 10);
  geometry_msgs::PoseStamped position_msg;
  position_msg.header.frame_id = "world";
  position_msg.pose.position.x = -10;
  position_msg.pose.position.y = 10;
  position_msg.pose.position.z = 5;

  ros::Rate rate(10);
  while(ros::ok())
  {
  /*  // publish accelertion command
    accel_msg.header.stamp = ros::Time::now();
    publish_accel.publish(accel_msg);    
  

    // publish angular velocity
    angular_velocity_msg.header.stamp = ros::Time::now();
    publish_augular_velocity.publish(angular_velocity_msg);


    // publish attitude setpoint
    attitude_msg.header.stamp = ros::Time::now();
    publish_attitude.publish(attitude_msg);
  

    // publish velocity command
    velocity_msg.header.stamp = ros::Time::now();
    publish_velocity.publish(velocity_msg);
 

    // publish setpoint postion
    position_msg.header.stamp = ros::Time::now();
    publish_position.publish(position_msg);
  */

    // publish raw setpoint message
    setpointraw_msg.header.stamp = ros::Time::now();
    publish_setpointraw.publish(setpointraw_msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
  
}

