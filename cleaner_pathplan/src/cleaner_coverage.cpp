

#include "ros/ros.h"  
#include "std_msgs/String.h"  
#include <sstream>  

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>


typedef struct
{
    double x;      //position
    double y;     
    double w;      //oritation
} MPose;

int main(int argc, char **argv)  
{  
    ros::init(argc,argv,"cleaner_coverage");  
    ros::NodeHandle n;  
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("cleaner_coverage_topic",1000);  
    ros::Rate loop_rate(10);  
    ROS_INFO("test_node start");

    ros::Publisher  goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",10);	
    MPose pose;
    pose.x = 3.6;
    pose.y = 12.5;
    pose.w = 0;
    while(ros::ok())  
    {  
        geometry_msgs::PoseStamped goalPose;
        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose.w);
        
       
        goalPose.header.frame_id = "/map";
        goalPose.pose.position.x = pose.x;
        goalPose.pose.position.y = pose.y;
        goalPose.pose.position.z = 0;
        goalPose.pose.orientation = goal_quat;	

        goal_pub.publish(goalPose);	
//
        std_msgs::String msg;  
        std::stringstream ss;  
        ss << "Here is an example!";  
        msg.data=ss.str();  
        chatter_pub.publish(msg);  
        ros::spinOnce();  
        loop_rate.sleep();  
    }  
    
    
    return 0;  
}  


