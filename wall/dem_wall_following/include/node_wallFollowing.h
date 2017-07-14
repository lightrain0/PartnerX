#ifndef SR_NODE_WALL_FOLLOWING
#define SR_NODE_WALL_FOLLOWING

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "dem_wall_following/psdtuneConfig.h"
//#define <ros/ros.h>
#include "kobuki_msgs/BumperEvent.h"
#include "dynamic_reconfigure/server.h"
/*! \brief Demonstration task: "Wall Following"
 * 
 * This class controls robot. Robot finds nearest wall and goes along it.
 */
 #include <costmap_2d/costmap_2d_ros.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/wall_timer.h>

#include <visualization_msgs/Marker.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

 class NodeWallFollowing
{
public:
    struct Node{
		enum NodeDir{
			X_N,
			Y_N,
			X_P,
			Y_P
		};
		geometry_msgs::Pose pose;
		NodeDir		dir;
	};
	class Area{
	public:
		bool IsOverLine(geometry_msgs::Pose pose)
		{
			geometry_msgs::Pose des_pos = joints_[cur_joint_index_];
			switch(cur_dir_)
			{
				case Node::X_N:
					return des_pos.position.x >= pose.position.x;
				break; 
				case Node::Y_N:
					return des_pos.position.y >= pose.position.y;
				break;
				case Node::X_P:
					return des_pos.position.x <= pose.position.x;
				break;
				case Node::Y_P:
					return des_pos.position.y <= pose.position.y;
				break;
			}
		}
		void GetNextDir(Node::NodeDir& dir)
		{
			switch(dir)
			{
				case Node::X_N:
					dir = Node::Y_N;
				break; 
				case Node::Y_N:
					dir = Node::X_P;
				break;
				case Node::X_P:
					dir = Node::Y_P;
				break;
				case Node::Y_P:
					dir = Node::X_N;
				break;
			}
		}
		bool IsOver()
		{
			return cur_joint_index_ > 4;
		}
		bool GetNextPose(geometry_msgs::Pose& pose)
		{
			cur_joint_index_ ++;
			if(IsOver())
				return false;
			if(cur_joint_index_ == 4)
				pose = joints_[0];
			else
				pose = joints_[cur_joint_index_];
			return true;
		}
		geometry_msgs::Pose joints_[4];
		int cur_joint_index_;
		geometry_msgs::PolygonStamped contour_;

		geometry_msgs::Pose start_pose_;
		Node::NodeDir  cur_dir_;
	};
	/*! \brief A constructor.
	 * 
	 * @param pub Publisher, which can send commands to robot.
	 * @param wallDist Desired distance from the wall.
	 * @param maxSp Maximum speed, that robot can go.
	 * @param dir 1 for wall on the right side of the robot (-1 for the left one).
	 * @param pr P constant for PSD regulator.
	 * @param su S constant for PSD regulator.
	 * @param di D constant for PSD regulator.
	 * @param an Angle coeficient for regulator.
	 */
	NodeWallFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double su, double di, double an);
	
	/*! \brief A destructor.
	 */
	~NodeWallFollowing();

	/*! \brief This method publishes commands for robot.
	 *
	 * Commands are generated from data, which are stored in variables
	 * (#angleMin, #angleMax, #distMin, #distMax). If the robot is far from 
	 * nearest obstacle (distMin is bigger than desired distance plus 
	 * constant), it turns to the obstacle and goes there. If it is near 
	 * the desired distance, it turns and goes along the obstacle. 
	 * The whole time, when robot is going along the obstacle (wall), it 
	 * tries to keep desired distance and angle from the wall. The higher
	 * is the diference between desired and actual values, the lower is 
	 * speed.
	 */
	
	void publishMessage();

	/*! \brief This method reads data from sensor and processes them to variables.
	 * 
	 * This method finds maximum and minimum distance in data from sensor
	 * and stores these values (with appropriate angles) into variables: 
	 * #angleMin, #angleMax, #distMin, #distMax.
	 * 
	 * @param msg Message, which came from robot and contains data from
	 * laser scan.
	 */
	
    void messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void bumperCallback(const kobuki_msgs::BumperEventConstPtr & msg);
    void reconfigureCB(dem_wall_following::psdtuneConfig &config, uint32_t level);

	bool findOriginalPosition();
	int MoveTo(const geometry_msgs::Pose& pose);
	int getCurrentPose(geometry_msgs::Pose& pose);
	bool IsFrontierObstacle();
	bool findArea();
//variables
	double wallDistance;//!<Desired distance from the wall.
	double r;			//!<Difference between desired distance from the wall and actual distance.
	double sumR;		//!<Sum of #r.
	double diffR;		//!<Estimated next r;
	double maxSpeed;	//!<Maximum speed of robot.
	double P;			//!<P constant for PSD regulator.
	double S;			//!<S constant for PSD regulator.
	double D; 			//!<D constant for PSD regulator.
	double angleCoef;	//!<Angle coeficient for regulator.
	int direction;		//!<1 for wall on the right side of the robot (-1 for the left one).
	double angleMin;	//!<Angle, at which was measured the shortest distance.
	double distMin;		//!<Minimum distance masured by ranger.
	double distFront;	//!<Distance, measured by ranger in front of robot.
	int go;				//!<If the obstacle is in front of robot, change to 0.
    ros::Publisher pubMessage;	//!<Object for publishing messages.
    dynamic_reconfigure::Server<dem_wall_following::psdtuneConfig> *server_;
    dynamic_reconfigure::Server<dem_wall_following::psdtuneConfig>::CallbackType f_;
    int bump_count_;
    bool bump_happen_;

	//
	      tf::TransformListener tf_listener_;
	 ros::NodeHandle nh_; 
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;
	double m_scan_min_dist;
	double m_scan_min_angle;
	ros::Publisher  goal_pub;
	double m_scan_front_dist;
	bool  m_turn_on_follow;

	geometry_msgs::Pose orig_pose_;


};

#endif
