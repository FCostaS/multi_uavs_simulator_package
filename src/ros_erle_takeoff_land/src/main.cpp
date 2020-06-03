#include <cstdlib>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
//#include <string>  // ----additional
int main(int argc, char **argv)
{

    int rate = 10;

    ros::init(argc, argv, "mavros_takeoff");
    ros::NodeHandle n;
    std::string group_name = ros::this_node::getNamespace(); //----additional
    std::string service_name_buffer; //----additional
    ros::Rate r(rate);

    ////////////////////////////////////////////
    /////////////////GUIDED/////////////////////
    ////////////////////////////////////////////
    service_name_buffer = "/" + group_name + "/mavros/set_mode"; // ----additional
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>(service_name_buffer); //---changed
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    if(cl.call(srv_setMode)){
        ROS_INFO("setmode send ok %d value:", srv_setMode.response);//改过，原先response后面还有个.success
    }else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }

    ////////////////////////////////////////////
    ///////////////////ARM//////////////////////
    ////////////////////////////////////////////
    service_name_buffer = "/" + group_name + "/mavros/cmd/arming"; // ----additional
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>(service_name_buffer); //--changed
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_INFO("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }

    ////////////////////////////////////////////
    /////////////////TAKEOFF////////////////////
    ////////////////////////////////////////////
    service_name_buffer = "/" + group_name + "/mavros/cmd/takeoff"; // ----additional
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>(service_name_buffer); //--changed
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 5;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if(takeoff_cl.call(srv_takeoff)){
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
    }

    ////////////////////////////////////////////
    /////////////////DO STUFF///////////////////
    ////////////////////////////////////////////
    /* sleep(120);

    ////////////////////////////////////////////
    ///////////////////LAND/////////////////////
    ////////////////////////////////////////////
    service_name_buffer = "/" + group_name + "/mavros/cmd/land"; // ----additional
    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>(service_name_buffer);
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 10;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if(land_cl.call(srv_land)){
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }else{
        ROS_ERROR("Failed Land");
    } */

    while (n.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}

