#include "node_wallFollowing.h"


#include <tf/transform_listener.h>

#define SUBSCRIBER_BUFFER_SIZE 10	//!<Size of buffer for subscriber.

#define PUBLISHER_BUFFER_SIZE 1000	//!<Size of buffer for publisher.

// #define PUBLISHER_TOPIC "/syros/baserostopic info /mobile_base/events/bumper _cmd_vel"

//#define PUBLISHER_TOPIC "/robot0/cmd_vel"
#define PUBLISHER_TOPIC "/cmd_vel_mux/input/teleop"
// #define SUBSCRIBER_TOPIC "/syros/laser_laser"

//#define SUBSCRIBER_TOPIC "/robot0/laser_0"
#define SUBSCRIBER_TOPIC "/scan"

#define SUBSCRIBER_BUMPER_TOPIC "/mobile_base/events/bumper"

/*! \brief Starting node for task: "Wall Following"
 * 
 * In main function is created Subscribing node, which transmits messages 
 * to NodeWallFollowing object. There are the messages processed and commands 
 * generated.
 */

int main(int argc, char **argv)
{
	//Initialization of node
	ros::init(argc, argv, "wallFollowing");
	ros::NodeHandle n;

	//Creating publisher

	ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);

	//Creating object, which stores data from sensors and has methods for
	//publishing and subscribing
    NodeWallFollowing *nodeWallFollowing = new NodeWallFollowing(pubMessage, 0.50, 1, -1, 0, 0, 0, 2);
	
			//Creating subscriber and publisher
    ros::Subscriber scan_sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeWallFollowing::messageCallback, nodeWallFollowing);

	 while(ros::ok())
	 {
		 ros::spinOnce();
		 if(true == nodeWallFollowing->findOriginalPosition())
		 {
			 nodeWallFollowing->findArea();
		 }
		 
	 }
    //ros::Subscriber bumper_sub = n.subscribe(SUBSCRIBER_BUMPER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeWallFollowing::bumperCallback, nodeWallFollowing);
	ros::spin();


	return 0;
}
