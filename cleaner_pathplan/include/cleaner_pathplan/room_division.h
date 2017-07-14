#ifndef ROOM_DIVISION_H_
#define ROOM_DIVISION_H_

#include "cleaner_pathplan/area_tool.h"
using namespace space_type;
using namespace area_tool;

#include "ros/ros.h"
#include "ros/callback_queue.h"
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

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/concept_check.hpp>

#include <sensor_msgs/LaserScan.h>

namespace cleaner_pathplan{
    #define PERIOD_ADVERTISING          0.1  // second
    #define FRAME_GLOBAL_MAP           "/map"
    #define FRAME_BASE_MAP             "/base_map"
    #define FRAME_BASE_LINK             "/base_link"
    #define TOPIC_SCAN                  "/scan"
    
    class RoomDivisionState
    {
        public:
        //! \brief Defines the various states the RoomDivision can be in
        enum StateEnum{
            NONE,
            FINDING,
            FOUND,
            EXPLORING,
            EXPLORED,
            DONE
        };
        RoomDivisionState(){state_ = NONE;};
        RoomDivisionState(const StateEnum& state) : state_(state) { }

        inline bool operator==(const RoomDivisionState& rhs) const
        {
            return (state_ == rhs.state_) ;
        }

        inline bool operator==(const RoomDivisionState::StateEnum& rhs) const
        {
            return (state_ == rhs);
        }

        inline bool operator!=(const RoomDivisionState::StateEnum& rhs) const
        {
            return !(*this == rhs);
        }

        inline bool operator!=(const RoomDivisionState& rhs) const
        {
            return !(*this == rhs);
        }

        std::string toString() const
        {
            switch(state_)
            {
            case NONE:
                return "NOT WORKING";
            case FINDING:
                return "FINDING";
            case DONE:
                return "DONE";
            default:
                ROS_ERROR_NAMED("room_division", "BUG: Unhandled RoomDivisionState: %u", state_);
                break;
            }
            return "BUG-UNKNOWN";
        }

        StateEnum state_;
        
         
    };
    class RoomDivision
    {
    
        public:
        
        //! \brief Define the struct of position based on the global_map frame id 
        typedef TPose   GlobalPose; 
        //! \brief Define the struct of position based on the base_map frame id 
        typedef TPose   LocalPose;
        
        /**
        * \brief Simple constructor
        *
        * Constructs a RoomDivision and sets up the necessary ros topics 
        * \param spin_thread If true, spins up a thread to service this class's subscriptions. If false,
        *                    then the user has to call ros::spin() themselves. Defaults to True
        */
            RoomDivision(bool spin_thread = true);
        /**
        * \brief Constructor with namespacing options
        *
        * Constructs a RoomDivision object and sets up the necessary ros topics and namespaces them according the a specified NodeHandle
        * \param n The node handle on top of which we want to namespace our RoomDivision
        * \param spin_thread If true, spins up a thread to service this class's subscriptions. If false,
        *                    then the user has to call ros::spin() themselves. Defaults to True
        */
            RoomDivision(ros::NodeHandle& n,bool spin_thread = true);
            ~RoomDivision();
            
            RoomDivisionState getState();
            void setState(RoomDivisionState state);
        protected:
            void init(ros::NodeHandle& n,bool spin_thread);
            void createBaseMap();
            void publishBaseMap();
            int updateCurrentPosition();
            bool findOriginalPosition();
            void TransmitState(RoomDivisionState newState);

            void circle_spin();
            void circle_advertising();
            void circle_statemachine();
            void circle_listening();

            // Callbacks
            void cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg);

       private:
            tf::TransformListener  tf_listener_;
            ros::NodeHandle        nh_;

            // Spin Thread Stuff
            boost::mutex terminate_mutex_;
            bool need_to_terminate_;
            boost::thread* spin_thread_;
            boost::thread* sm_thread_;
            ros::CallbackQueue callback_queue_;

            //
            GlobalPose       global_original_pose_;
            GlobalPose       global_current_pose_;
            LocalPose        local_current_pose_;

            //
            RoomDivisionState       current_state_;
    };
}

#endif
