#include "cleaner_pathplan/room_division.h"


namespace cleaner_pathplan{

    RoomDivision::RoomDivision(bool spin_thread)
    {
        init(nh_,spin_thread);
    }
    RoomDivision::RoomDivision(ros::NodeHandle& n,bool spin_thread)
    {
        init(n,spin_thread);
    }
    void RoomDivision::init(ros::NodeHandle& n,bool spin_thread)
    {
        // Create a thread to spin for this node 
        if (spin_thread)
        {
            need_to_terminate_ = false;
            n.setCallbackQueue(&callback_queue_);
            spin_thread_ = new boost::thread(boost::bind(&RoomDivision::circle_spin, this));
        }
        else 
        {
            spin_thread_ = NULL;
        }
        //
        current_state_ = RoomDivisionState::NONE;
        // Set up topics 
        n.subscribe(TOPIC_SCAN,10,&RoomDivision::cb_scan, this);
        //
        sm_thread_ = new boost::thread(boost::bind(&RoomDivision::circle_statemachine, this));
        // Create a timer to publish topics periodly
        n.createWallTimer(ros::WallDuration(PERIOD_ADVERTISING),boost::bind(&RoomDivision::circle_advertising,this));
     //   n.createTimer(ros::Duration(PERIOD_ADVERTISING),boost::bind(&RoomDivision::circle_advertising,this));
    }
    RoomDivision::~RoomDivision()
    {
        {
        boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
        need_to_terminate_ = true;
        }
        if (spin_thread_)
        {
            spin_thread_->join();
            delete spin_thread_;
        }
        if(sm_thread_)
        {
            sm_thread_->join();
            delete sm_thread_;
        }
    }
    void RoomDivision::circle_spin()
    {
        while (nh_.ok())
        {
            {
            boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
            if (need_to_terminate_)
                break;
            }
            callback_queue_.callAvailable(ros::WallDuration(0.1f));
        }
    }
    void RoomDivision::circle_advertising()
    {
        publishBaseMap();
    }
    void RoomDivision::circle_statemachine()
    {
        while (nh_.ok())
        {
            {
            boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
            if (need_to_terminate_)
                break;
            }
            //
            switch(getState().state_)
            {
                case RoomDivisionState::NONE:
                    TransmitState(RoomDivisionState::FINDING);
                break;
                case RoomDivisionState::FINDING: 
                    if(findOriginalPosition())
                        TransmitState(RoomDivisionState::FOUND);
                break;
                case RoomDivisionState::FOUND:
                    // Create area according to original position 

                    // Add area to room 

                    //
                    TransmitState(RoomDivisionState::EXPLORING);
                break;
                case RoomDivisionState::EXPLORING: 
                    // static bool islastCoveredOff = true ;
                    
                    // if(islastCoveredOff)
                        // Find active Node 
                        //...
                        // Create area with active node
                        //...
                        // GetNextLinePosition 
                        // ...
                        //If(IsRoomCovered())
                            //TransmitState(RoomDivisionState::EXPLORED);
                        
                    // Moving to position  
                    // islastCoveredOff = false;
                    // Turn on wall following 
                    // Is Obstacle Ob 
                    // else is occupied by another area ,Oc 
                    // else is Jo 
                        // If yes , check it is nearby the obstacle in front 
                             // If yes , add Ob property 
                    // else is Bo 
                    // else is Co 
                    //  if above conditions are satisfied  
                        // Add node to area
                        // if it is not Co
                            // Turn off wall following 
                            // GetNextLinePosition
                                // if true, moving to next position 
                                // if false, 
                                    // Add area to room 
                                    // islastCoveredOff = true;
                    // else continue moving forward 
                    
                            
                break;
                case RoomDivisionState::EXPLORED: 
                    ROS_INFO("ROOM EXPLORING FINISHED");
                    TransmitState(RoomDivisionState::DONE);
                break;
                case RoomDivisionState::DONE: 
                break;
                default: 
                break;
            }
        }
        
    }
    void RoomDivision::circle_listening()
    {
        updateCurrentPosition();
    }
    void RoomDivision::cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
    {

    }
    void RoomDivision::TransmitState(RoomDivisionState newState)
    {
        setState(newState);
        ROS_INFO_NAMED("room_division","State is transmitted to %s",getState().toString().c_str());
    }
    RoomDivisionState RoomDivision::getState()
    {
        return current_state_;
    }
    void RoomDivision::setState(RoomDivisionState state)
    {
        current_state_ = state;
    }
    bool RoomDivision::findOriginalPosition()
    {
        
        return true;
    }
    void RoomDivision::publishBaseMap()
    {
        if(getState() == RoomDivisionState::NONE || getState() == RoomDivisionState::FINDING)
        {
            ROS_WARN_NAMED("room_division","Not ready to publish base map , firstly to find out original position");
            return ;
        }
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(global_original_pose_.x, global_original_pose_.y, global_original_pose_.z) );
        tf::Quaternion q;
        q.setRPY(global_original_pose_.roll, global_original_pose_.pitch, global_original_pose_.yaw);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),FRAME_GLOBAL_MAP, FRAME_BASE_MAP));
    }

    int RoomDivision::updateCurrentPosition()
    {

        double yaw, pitch, roll;
        std::string map_frame_  = FRAME_GLOBAL_MAP;
        std::string base_link_frame_ = FRAME_BASE_LINK;
        geometry_msgs::Pose pose;
        try
        {
        //    ROS_INFO("WAITING FOR TRANSFORM FROM %s TO %s",map_frame_.c_str(),base_link_frame_.c_str());
            tf_listener_.waitForTransform(map_frame_, base_link_frame_, ros::Time(), ros::Duration(0.5));

            tf::StampedTransform transform;
            tf_listener_.lookupTransform(map_frame_, base_link_frame_, ros::Time(), transform);
            std::cout.precision(3);
            std::cout.setf(std::ios::fixed,std::ios::floatfield);

            poseTFToMsg(transform,pose);

            GlobalPose gl_pose;
            gl_pose.x = pose.position.x;
            gl_pose.y = pose.position.y;
            gl_pose.z = pose.position.z;
            gl_pose.yaw = tf::getYaw(pose.orientation);
            global_current_pose_ = gl_pose;

            if(getState() == RoomDivisionState::NONE || getState() == RoomDivisionState::FINDING)
            {
                ROS_WARN_NAMED("room_division","Not ready to update local current position , firstly to find out original positio ");
                return -3;
            }
            else
            {
                GlobalPose& orig_pose = global_original_pose_;
                LocalPose lc_pose;
                lc_pose.x = gl_pose.x - orig_pose.x;
                lc_pose.y = gl_pose.y - orig_pose.y;
                lc_pose.z = gl_pose.z - orig_pose.z;
                lc_pose.yaw = gl_pose.yaw;
                local_current_pose_ = lc_pose;
                return 0;
            }
        }
        catch(tf::LookupException& ex)
        {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return -1;
        }
        catch(tf::TransformException& ex)
        {
            std::cout << "Failure at "<< ros::Time::now() << std::endl;
            std::cout << "Exception thrown:" << ex.what()<< std::endl;
            std::cout << "The current list of frames is:" <<std::endl;
            std::cout << tf_listener_.allFramesAsString()<<std::endl;    
            return -2;
        }
    }
}