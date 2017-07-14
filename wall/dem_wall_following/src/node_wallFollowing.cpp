#include "node_wallFollowing.h"
//#include <node_wallFollowing.h>

#include <math.h>
#define PI 3.141592		//!<Mathematical constant (default value: 3.141592).	
double diffdist;
double diffangle;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    template<typename T, typename S>
  double pointsDistance(const T &one, const S &two){
      return sqrt(pow(one.x-two.x,2.0) + pow(one.y-two.y,2.0) + pow(one.z-two.z,2.0));
  }
  
  template<typename T, typename S>
  bool pointsNearby(const T &one, const S &two, const double &proximity){
      return pointsDistance(one, two) <= proximity;
  }


//Constructor and destructor
NodeWallFollowing::NodeWallFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double su, double di, double an)
    :
           tf_listener_(ros::Duration(10.0)),
            nh_(),
            move_client_("move_base",true)
{
	wallDistance = wallDist;
	maxSpeed = maxSp;
	direction = dir;
	P = pr;
	S = su;
	D = di;
	angleCoef = an;
	r = 0;
	sumR = 0;
	distMin = 0;	//minimum distance masured by sensor
	angleMin = 0;	//angle, at which was measured the shortest distance
	go = 1;			//in case of obstacle, change to 0
	pubMessage = pub;
    server_ = new dynamic_reconfigure::Server<dem_wall_following::psdtuneConfig>;
    //dynamic_reconfigure::Server<psdtuneConfig>::CallbackType cb = boost::bind(&DWAPlannerROS::reconfigureCB, this, _1, _2);
    f_ = boost::bind(&NodeWallFollowing::reconfigureCB, this, _1, _2);
    server_->setCallback(f_);
    bump_count_=0;
    bump_happen_==false;

    //
    m_scan_min_dist = 100;
    m_scan_min_angle = 0;
    goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",10);	
}


void NodeWallFollowing::reconfigureCB(dem_wall_following::psdtuneConfig &config, uint32_t level)
{
   wallDistance = config.wDst;
   P=config.P;
   S=config.S;
   D=config.D;
   maxSpeed=config.maxSp;
   direction=config.wDir;
   angleCoef=config.an;
   ROS_INFO("Reconfigure Request: %f %f %f %f %f %d %f",
             config.wDst, config.P, config.S,
             config.D, config.maxSp, config.wDir,
             config.an);

}

NodeWallFollowing::~NodeWallFollowing()
{

}

int NodeWallFollowing::getCurrentPose(geometry_msgs::Pose& pose)
{

    double yaw, pitch, roll;
    std::string map_frame_  = "/map";
    std::string base_link_frame_ = "base_link";
	try
    {
    //    ROS_INFO("WAITING FOR TRANSFORM FROM %s TO %s",map_frame_.c_str(),base_link_frame_.c_str());
   		tf_listener_.waitForTransform(map_frame_, base_link_frame_, ros::Time(), ros::Duration(0.5));

        tf::StampedTransform transform;
        tf_listener_.lookupTransform(map_frame_, base_link_frame_, ros::Time(), transform);
        std::cout.precision(3);
        std::cout.setf(std::ios::fixed,std::ios::floatfield);

		poseTFToMsg(transform,pose);
    
        // tf::Vector3 v = transform.getOrigin();
	    // tf::Quaternion q = transform.getRotation();	
        // transform.getBasis().getRPY(roll, pitch, yaw);

        // pose.position.x = v.getX();
        // pose.position.y = v.getY();
        // pose.position.z = v.getZ();

        // tf::Quaternion quat = tf::createQuaternionFromYaw(yaw);

        // pose.orientation.x = quat.x();
        // pose.orientation.y = quat.y();
        // pose.orientation.z = quat.z();
        // pose.orientation.w = quat.w();

        return 0;

    }
    catch(tf::LookupException& ex)
      {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return -1;
      }
      
      catch(tf::ConnectivityException& ex) 
      {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return -1;
      }
      
      catch(tf::ExtrapolationException& ex) 
      {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        return -1;
      }
      catch(tf::TransformException& ex)
      {
          std::cout << "Failure at "<< ros::Time::now() << std::endl;
          std::cout << "Exception thrown:" << ex.what()<< std::endl;
          std::cout << "The current list of frames is:" <<std::endl;
          std::cout << tf_listener_.allFramesAsString()<<std::endl;    
          return -1;
      }
}
int NodeWallFollowing::MoveTo(const geometry_msgs::Pose& pose)
{

    if(!move_client_.waitForServer(ros::Duration(60)))
    {
        ROS_ERROR("Can't connected to move base server");
        return -1;		
    }
    ROS_INFO("Connected to move base server");

    move_base_msgs::MoveBaseGoal goal;
	
	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose =  pose;

	move_client_.sendGoal(goal);
 //    goal_pub.publish(goal.target_pose);	// Must hide , if using waitforresult , preempted due to confict with rviz command after receiving this topic .
	int time_count = 10 * 60;

    geometry_msgs::Point32 goalPoint;
    goalPoint.x = pose.position.x;
    goalPoint.y = pose.position.y;
    while(nh_.ok())
	{
        //  geometry_msgs::Pose curPose;
        // getCurrentPose(curPose);
        // geometry_msgs::Point32 curPoint;
        // curPoint.x = curPose.position.x;
        // curPoint.y = curPose.position.y;

        // if(pointsNearby(goalPoint,curPoint,0.1))
        //     break;
		// ros::spinOnce();
        // actionlib::SimpleClientGoalState curState = move_client_.getState();
        // std::string strState = curState.toString();
	 	// std::cout<<"Current state: "<<strState<<std::endl;

         if(move_client_.waitForResult(ros::Duration(30,0)))
        {

          ROS_INFO("MOVE_BASE DONE");
            break;
        }
        else 
            ROS_INFO("MOVE_BASE TIMEOUT");
    }
    
    return 0;
	// if(move_client_.waitForResult(ros::Duration(30,0)))
    // {
    //     ROS_INFO("MOVE_BASE DONE");
    //     return 0;
    // } 
    // ROS_INFO("MOVE_BASE PREEMPTED");
    // return -1;


	// while(nh_.ok())
	// {
	// 	ros::spinOnce();
	// 	actionlib::SimpleClientGoalState curState = move_client_.getState();
		
	// 	if(!move_client_.waitForResult(ros::Duration(0.1)))
	// 	{
	// 		if(time_count-- < 0)
	// 		{		
	// 			move_client_.cancelGoal();
	// 			ROS_WARN("Timed out achieving goal");
	// 			return -3;
	// 		}
	// 	}
		
	// 	if(curState == actionlib::SimpleClientGoalState::PENDING)
	// 	{
			
		
	// 	}	
	// 	else if(curState == actionlib::SimpleClientGoalState::ACTIVE)
	// 	{
			
		
	// 	}
	// 	else if(curState == actionlib::SimpleClientGoalState::SUCCEEDED)
	// 	{
	// 		ROS_INFO("Goal succeeded!");
	// 		return 0;
	// 	}
	// 	else if(curState == actionlib::SimpleClientGoalState::ABORTED)
	// 	{
	// 		ROS_WARN("ABORTED!");
	// 		return -2;
	// 	}
    //     else if(curState == actionlib::SimpleClientGoalState::RECALLED)
    //     {
    //         ROS_WARN("RECALLED!");
    //         return -4;
    //     }
	// 	else
	// 	{			
	// 		std::string strState = curState.toString();
	// 		std::cout<<"Current state: "<<strState<<std::endl;		
	// 		return -1;
	// 	}
	// }
}
bool NodeWallFollowing::findArea()
{
    std::vector<Node> nodes;
    int cur_node_index = 0;
    const int stride = 0.5;
    geometry_msgs::Pose curPose;
    getCurrentPose(curPose);
    Node node;
    node.pose = curPose;
    node.dir = Node::X_N;
    nodes.push_back(node);
    while(ros::ok() && cur_node_index < nodes.size())
    {
        Node t_node;
        t_node = nodes.at(cur_node_index++);
        //
        Area area;
        area.start_pose_ = t_node.pose;
        area.cur_dir_ = node.dir;
        if(node.dir == Node::X_N )
        {
            area.joints_[0] = t_node.pose;
            area.joints_[1] = area.joints_[0];
            area.joints_[1].position.x -= stride;
            area.joints_[2] = area.joints_[1];
            area.joints_[2].position.y -= stride;
            area.joints_[3] = area.joints_[2];
            area.joints_[3].position.x += stride;
        }
        //
        geometry_msgs::Pose jointPose;
        while(area.GetNextPose(jointPose))
        {
            MoveTo(jointPose);
        }
    }

}
bool NodeWallFollowing::findOriginalPosition()
{
    m_scan_min_dist = 100;

    // Firstly rotating 180 degree 
    geometry_msgs::Pose curPose;
    getCurrentPose(curPose);
    double curYaw = tf::getYaw(curPose.orientation);
    // tf::Pose pose;
    // tf::poseMsgToTF(curPose, pose);
    // double yaw_angle = tf::getYaw(pose.getRotation());
    double rotYaw = curYaw + M_PI;

    geometry_msgs::Pose rotPose = curPose;
    tf::Quaternion quat = tf::createQuaternionFromYaw(rotYaw);

    rotPose.orientation.x = quat.x();
    rotPose.orientation.y = quat.y();
    rotPose.orientation.z = quat.z();
    rotPose.orientation.w = quat.w();
    ROS_INFO("Current position (%.2f,%.2f,%.2f) , Current yaw %.2f, rot yaw %.2f",curPose.position.x,curPose.position.y,curPose.position.z,curYaw,rotYaw);
   
    // if(0 != MoveTo(rotPose))
    // {
    //  //   return false;
    // }
    rotYaw = curYaw;
    geometry_msgs::Twist msg;
    double delta = 0;
    while(delta < M_PI && ros::ok())
    {
        
        getCurrentPose(rotPose);
        rotYaw = tf::getYaw(rotPose.orientation);
        delta += fabs(rotYaw-curYaw);
        curYaw = rotYaw;

        msg.angular.z = 0.3;
        pubMessage.publish(msg);
        ros::spinOnce();
         ROS_INFO("delta : %.2f",delta);
    }

    const double offset = 0.1;
    if(m_scan_min_dist < offset)
    {
        ROS_INFO("Distance to original position is too short relative to %.2f",offset);
        return true;
    } 
    else if(m_scan_min_dist == 100)
    {
        ROS_WARN("The shortest distance not founded");
        return false;
    }
    //
    double orig_yaw = m_scan_min_angle - M_PI/2;
    geometry_msgs::Pose origPose = curPose;
    origPose.position.x = curPose.position.x + (m_scan_min_dist - offset)* cos(m_scan_min_angle);
    origPose.position.y = curPose.position.y + (m_scan_min_dist - offset)* sin(m_scan_min_angle);
    quat = tf::createQuaternionFromYaw(orig_yaw);
    origPose.orientation.x = quat.x();
    origPose.orientation.y = quat.y();
    origPose.orientation.z = quat.z();
    origPose.orientation.w = quat.w();
    ROS_INFO("Original position (%.2f,%.2f,%.2f) min_angle:%.2f min_distance:%.2f",origPose.position.x,origPose.position.y,orig_yaw,m_scan_min_angle,m_scan_min_dist);
    orig_pose_ = origPose;
    if(0 != MoveTo(origPose))
    {
        return false;
    }
    
 //   m_turn_on_follow = true;
    return true;
}
//Publisher
void NodeWallFollowing::publishMessage()
{

    //preparing messagekobuki_msgs/BumperEvent
    if(!bump_happen_)
    {
        geometry_msgs::Twist msg;
        
        msg.angular.z = direction*(P*r + S*sumR + D*diffR) + angleCoef * (angleMin - PI*direction/2);		//PI regulator
        diffdist = direction*(P*r + S*sumR + D*diffR);
        diffangle = angleCoef * (angleMin - PI*direction/2);
            ROS_INFO("Difference r: wallDistance=%f, distMin=%f, r=%f",wallDistance,distMin,r);
        ROS_INFO("diffdist=%f,diffangle=%f",diffdist,diffangle);
        if (msg.angular.z > 2.0){
            msg.angular.z = 1.0;
        }
        else if (msg.angular.z < -2.0){
            msg.angular.z = -1.0;
	    }
	//
	if (distFront < wallDistance){
		msg.linear.x = 0;
		//msg.angular.z = 0.0;
        ROS_INFO("distFront: %f",distFront);
	}
    else if (distFront < wallDistance * 2){//sending information about message to console

        msg.linear.x = 0.3*maxSpeed;
	//msg.angular.z = 0.0;
	}
    else if (fabs(angleMin)>1.75){
		msg.linear.x = 0.4*maxSpeed;
		//msg.angular.z = 0.0;
	}
	else {
		msg.linear.x = maxSpeed;
		//msg.angular.z = 0.0;
	}
    //sending information about message to console


	//sending information about message to console

	ROS_INFO("Sending msg: linear.x=%f, angular.z=%f",msg.linear.x,msg.angular.z);
	
	//publishing message
	pubMessage.publish(msg);
    }
}//sending information about message to console



//Subscriber
void NodeWallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{//sending information about message to console

	// //Calculation of array size from angle range and angle increment.
	// //int size = (int) ((msg->angle_max - msg->angle_min)/msg->angle_increment);
	 int size = msg->ranges.size();
    // ROS_INFO("array_length: [%d]", size);

	
    // //Variables whith index of highest and lowest value in array.//sending information about message to console

	 int minIndex = size*(direction+1)/4;
	
    if(m_turn_on_follow)
    {
        // //This cycle goes through array and finds minimum
        for(int i=(size*(direction+1)/4); i<(size*(direction+3)/4); i++)
        {
            if (msg->ranges[i] < msg->ranges[minIndex] && msg->ranges[i] > 0.01){

                minIndex = i;
            }
        }
        
        //Calculation of angles from indexes and storing data to class variables.
        angleMin = (minIndex-size/2)*msg->angle_increment;
        distMin = msg->ranges[minIndex];
        distFront = msg->ranges[size/2];
        diffR = 2*(distMin - wallDistance) - r;
        r = distMin - wallDistance;
        sumR += r;
        
        //Sending info about processed data
        ROS_INFO("min: angle=%f, distance=%f, front=%f", angleMin, distMin, distFront);
        //Invoking method for publishing message
    	publishMessage();
    }
    else 
    {
        //------------------------ ZGY
        for(int i=0; i<size; i++)
        {
            if (msg->ranges[i] < msg->ranges[minIndex] && msg->ranges[i] > 0.01){

                minIndex = i;
            }
        }
        m_scan_front_dist = msg->ranges[0];
        //ROS_INFO("Scan message callback executed");
        if(m_scan_min_dist > msg->ranges[minIndex])
        {
            m_scan_min_dist =  msg->ranges[minIndex];
            geometry_msgs::Pose curPose;
            getCurrentPose(curPose);
            double curYaw = tf::getYaw(curPose.orientation);
            m_scan_min_angle = (minIndex-size/2)*msg->angle_increment + curYaw;
            if(m_scan_min_angle > M_PI)
                m_scan_min_angle -= 2*M_PI;
            else if(m_scan_min_angle < -M_PI)
                m_scan_min_angle += 2*M_PI;
        // ROS_INFO("min: angle=%f, distance=%f, curYaw=%f", m_scan_min_angle, m_scan_min_dist, curYaw);
        }
    }
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(orig_pose_.position.x,orig_pose_.position.y, 0) );
    tf::Quaternion q;
    q.setRPY(0, 0, m_scan_min_angle- M_PI/2);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/map", "/base_map"));
}
bool NodeWallFollowing::IsFrontierObstacle()
{
    return m_scan_front_dist <= wallDistance;
}
void NodeWallFollowing::bumperCallback(const kobuki_msgs::BumperEventConstPtr& msg )
{
    if(msg->state==1)
    {

        bump_count_++;
        bump_happen_==true;
        ROS_INFO("*****************+++++++++++++++++++****************");
        ROS_INFO("*****************+++++++++++++++++++****************");
        ROS_INFO("Now bumper pressed already happened for %d times", bump_count_);
        ROS_INFO("*****************+++++++++++++++++++****************");
        ROS_INFO("*****************+++++++++++++++++++****************");
        if(direction==1)
        {
            geometry_msgs::Twist msg;
            msg.angular.z = -1.57;
            msg.linear.x=0.0;
            pubMessage.publish(msg);
        }
        else if (direction==-1)
        {
            geometry_msgs::Twist msg;
            msg.angular.z = 1.57;
            msg.linear.x=0.0;
            pubMessage.publish(msg);

        }
    }
    else if(msg->state==0)
    {
        bump_happen_==false;
    }


}

