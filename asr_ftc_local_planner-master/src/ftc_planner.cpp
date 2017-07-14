/**

Copyright (c) 2016, Marek Felix
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ros/ros.h>

#include <ftc_local_planner/ftc_planner.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::FTCPlanner, nav_core::BaseLocalPlanner)

namespace ftc_local_planner
{

    FTCPlanner::FTCPlanner()
    {
    }

    void FTCPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);
        local_plan_publisher_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
        costmap_ros_ = costmap_ros;
        tf_ = tf;
        first_setPlan_ = true;
        rotate_to_global_plan_ = false;
        goal_reached_ = false;
        stand_at_goal_ = false;
        cmd_vel_angular_z_rotate_ = 0;
        cmd_vel_linear_x_ = 0;
        cmd_vel_angular_z_ = 0;

        //Parameter for dynamic reconfigure
        dsrv_ = new dynamic_reconfigure::Server<FTCPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<FTCPlannerConfig>::CallbackType cb = boost::bind(&FTCPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        joinCostmap_ = new JoinCostmap();

        ROS_INFO("FTCPlanner: Init.");
    }

    void FTCPlanner::reconfigureCB(FTCPlannerConfig &config, uint32_t level)
    {
        if (config.restore_defaults)
        {
            config = default_config_;
            config.restore_defaults = false;
        }
        config_ = config;
    }

    bool FTCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        global_plan_ = plan;

        //First start of the local plan. First global plan.
        bool first_use = false;
        if(first_setPlan_)
        {
            //init joincostmap with local an global costmap.
            joinCostmap_->initialize(costmap_ros_, global_costmap_ros_);

            first_setPlan_ = false;
            ftc_local_planner::getXPose(*tf_,global_plan_, costmap_ros_->getGlobalFrameID(),old_goal_pose_,global_plan_.size()-1);
            first_use = true;
        }
        tf::Stamped<tf::Pose> new_goal_pose;
        ftc_local_planner::getXPose(*tf_,global_plan_, costmap_ros_->getGlobalFrameID(),new_goal_pose,global_plan_.size()-1);
        //Have the new global plan an new goal, reset. Else dont reset.
        if(std::abs(std::abs(old_goal_pose_.getOrigin().getX())-std::abs(new_goal_pose.getOrigin().getX())) <= config_.position_accuracy &&
                std::abs(std::abs(old_goal_pose_.getOrigin().getY())-std::abs(new_goal_pose.getOrigin().getY())) <= config_.position_accuracy && !first_use
                && std::abs(angles::shortest_angular_distance(tf::getYaw(old_goal_pose_.getRotation()), tf::getYaw(new_goal_pose.getRotation()))) <= config_.rotation_accuracy)
        {
            ROS_DEBUG("FTCPlanner: Old Goal == new Goal.");
        }
        else
        {
            //Rotate to first global plan point.
            rotate_to_global_plan_ = true;
            goal_reached_ = false;
            stand_at_goal_ = false;
            ROS_INFO("FTCPlanner: New Goal. Start new routine.");
        }

        old_goal_pose_ = new_goal_pose;

        return true;
    }

    bool FTCPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        static double last_x_vel = 0;
        tf::Stamped<tf::Pose> current_pose;
        costmap_ros_->getRobotPose(current_pose);

        //Join the actual global an local costmap in the global costmap.
        if(config_.join_obstacle){
            joinCostmap_->joinMaps();
        }
        int max_point = 0;
        //First part of the routine. Rotatio to the first global plan orientation.
        if(rotate_to_global_plan_)
        {
            double angle_to_global_plan = calculateGlobalPlanAngle(current_pose, global_plan_, checkMaxDistance(current_pose));
            rotate_to_global_plan_ = rotateToOrientation(angle_to_global_plan, cmd_vel, 0.05);
            ROS_INFO("--FTCPlanner-- ***** firstly rotate to global plan with bias %.2f return %d ***** x_vel %.2f rot_vel %.2f",
            angle_to_global_plan,rotate_to_global_plan_, cmd_vel.linear.x,cmd_vel.angular.z);
        }
        //Second part of the routine. Drive along the global plan.
        else
        {
            tf::Stamped<tf::Pose> x_pose;

            ftc_local_planner::getXPose(*tf_,global_plan_, costmap_ros_->getGlobalFrameID(),x_pose,global_plan_.size()-1);
            double distance = sqrt(pow((x_pose.getOrigin().getX()-current_pose.getOrigin().getX()),2)+pow((x_pose.getOrigin().getY()-current_pose.getOrigin().getY()),2));

            //Check if robot near enough to global goal.
            if(distance > config_.position_accuracy && !stand_at_goal_)
            {

                if(fabs(calculateGlobalPlanAngle(current_pose, global_plan_, checkMaxDistance(current_pose)) > 1.2))
                {
                    ROS_INFO("FTCPlanner: Excessive deviation from global plan orientation. Start routine new.============================");
                    rotate_to_global_plan_ = true;
                }

                max_point = driveToward(current_pose, cmd_vel);

                if(!checkCollision(max_point))
                {
                    return false;
                }
            }
            //Third part of the routine. Rotate at goal to goal orientation.
            else
            {
                if(!stand_at_goal_)
                {
                    ROS_INFO("FTCPlanner: Stand at goal. Rotate to goal orientation.");
                }
                stand_at_goal_ = true;
                tf::Stamped<tf::Pose> x_pose;

                //Get the goal orientation.
                ftc_local_planner::getXPose(*tf_,global_plan_, costmap_ros_->getGlobalFrameID(),x_pose,global_plan_.size()-1);
                double angle_to_global_plan = angles::shortest_angular_distance(tf::getYaw(current_pose.getRotation()), tf::getYaw(x_pose.getRotation()));
                //Rotate until goalorientation is reached.
                if(!rotateToOrientation(angle_to_global_plan, cmd_vel, config_.rotation_accuracy))
                {
                    goal_reached_ = true;
                }
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
            }
        }

        publishPlan(max_point);
        return true;
    }

    int FTCPlanner::checkMaxDistance(tf::Stamped<tf::Pose> current_pose)
    {
        int max_point = 0;
        tf::Stamped<tf::Pose> x_pose;

        for (unsigned int i = 0; i < global_plan_.size(); i++)
        {
            ftc_local_planner::getXPose(*tf_,global_plan_, costmap_ros_->getGlobalFrameID(),x_pose,i);
            double distance = sqrt(pow((x_pose.getOrigin().getX()-current_pose.getOrigin().getX()),2)+pow((x_pose.getOrigin().getY()-current_pose.getOrigin().getY()),2));

          //  max_point = i-1;
            max_point = i; //ZGY ADD
            //If distance higher than maximal moveable distance in sim_time.
            if(distance > (config_.max_x_vel*config_.sim_time))
            {
                ROS_INFO("--FTCPlanner-- max distance  %.2f bigger than sim-distance %.2f on the point %d",distance,config_.max_x_vel*config_.sim_time,i);
                break;
            }
        }
        if(max_point < 0)
        {
            max_point = 0;
        }
        return max_point;
    }

    int FTCPlanner::checkMaxAngle(int points, tf::Stamped<tf::Pose> current_pose)
    {
        int max_point = points;
        double angle = 0;
        for(int i = max_point; i >= 0; i--)
        {
            angle = calculateGlobalPlanAngle(current_pose, global_plan_, i);

            max_point = i;
            //check if the angle is moveable
            if(fabs(angle) < config_.max_rotation_vel*config_.sim_time)
            {
                break;
            }
        }
        return max_point;
    }

    double FTCPlanner::calculateGlobalPlanAngle(tf::Stamped<tf::Pose> current_pose, const std::vector<geometry_msgs::PoseStamped>& plan, int point)
    {
        if(point >= (int)plan.size())
        {
            point = plan.size()-1;
        }
        double angle = 0;
        double current_th = tf::getYaw(current_pose.getRotation());
        for(int i = 0; i <= point; i++)
        {
            tf::Stamped<tf::Pose> x_pose;
            ftc_local_planner::getXPose(*tf_,global_plan_, costmap_ros_->getGlobalFrameID(),x_pose,point);


            //Calculate the angles between robotpose and global plan point pose
            double angle_to_goal = atan2(x_pose.getOrigin().getY() - current_pose.getOrigin().getY(),
                                         x_pose.getOrigin().getX() - current_pose.getOrigin().getX());
            angle += angle_to_goal;
        }

        //average
        angle = angle/(point+1);

        return angles::shortest_angular_distance(current_th, angle);
    }

    bool FTCPlanner::rotateToOrientation(double angle, geometry_msgs::Twist& cmd_vel, double accuracy)
    {
        if(fabs(angle) > accuracy)
        {
            //Slow down
            if(config_.max_rotation_vel >= fabs(angle) * (config_.acceleration_z+config_.slow_down_factor))
            {
                ROS_DEBUG("FTCPlanner: Slow down.");
                ROS_INFO("FTCPlanner: Slow down.");
                if(angle < 0)
                {
                    if(cmd_vel_angular_z_rotate_ >= -config_.min_rotation_vel)
                    {
                        cmd_vel_angular_z_rotate_ = - config_.min_rotation_vel;
                        cmd_vel.angular.z = cmd_vel_angular_z_rotate_;

                    }
                    else
                    {
                        cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_rotate_ + config_.acceleration_z/config_.local_planner_frequence;
                        cmd_vel.angular.z = cmd_vel_angular_z_rotate_;
                    }
                }
                if(angle > 0)
                {
                    if(cmd_vel_angular_z_rotate_  <= config_.min_rotation_vel)
                    {
                        cmd_vel_angular_z_rotate_ =  config_.min_rotation_vel;
                        cmd_vel.angular.z = cmd_vel_angular_z_rotate_;

                    }
                    else
                    {
                        cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_rotate_ - config_.acceleration_z/config_.local_planner_frequence;
                        cmd_vel.angular.z = cmd_vel_angular_z_rotate_;
                    }
                }
            }
            else
            {
                //Speed up
                if(fabs(cmd_vel_angular_z_rotate_) < config_.max_rotation_vel)
                {
                    ROS_DEBUG("FTCPlanner: Speeding up");
                    ROS_INFO("FTCPlanner: Speeding up");
                    if(angle < 0)
                    {
                        cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_rotate_ - config_.acceleration_z/config_.local_planner_frequence;

                        if(fabs(cmd_vel_angular_z_rotate_) > config_.max_rotation_vel)
                        {
                            cmd_vel_angular_z_rotate_ = - config_.max_rotation_vel;
                        }
                        cmd_vel.angular.z = cmd_vel_angular_z_rotate_;
                    }
                    if(angle > 0)
                    {
                        cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_rotate_ + config_.acceleration_z/config_.local_planner_frequence;

                        if(fabs(cmd_vel_angular_z_rotate_) > config_.max_rotation_vel)
                        {
                            cmd_vel_angular_z_rotate_ = config_.max_rotation_vel;
                        }

                        cmd_vel.angular.z = cmd_vel_angular_z_rotate_;
                    }
                }
                else
                {
                    cmd_vel.angular.z = cmd_vel_angular_z_rotate_;
                }
            }
            ROS_DEBUG("FTCPlanner: cmd_vel.z: %f, angle: %f", cmd_vel.angular.z, angle);
            return true;
        }
        else
        {
            cmd_vel_angular_z_rotate_ = 0;
            cmd_vel.angular.z = 0;
            return false;
        }
    }

    int FTCPlanner::driveToward(tf::Stamped<tf::Pose> current_pose, geometry_msgs::Twist& cmd_vel)
    {
        double distance = 0;
        double angle = 0;
        int max_point = 0;
         ROS_INFO(" --------------------------DriveToward -----------------------------");
        //Search for max achievable point on global plan.
        max_point = checkMaxDistance(current_pose);
    //    ROS_INFO("--FTCPlanner-- max distance point %d",max_point);
        max_point = checkMaxAngle(max_point, current_pose);
    //    ROS_INFO("--FTCPlanner-- max angle point %d",max_point);

        double cmd_vel_linear_x_old = cmd_vel_linear_x_;
        double cmd_vel_angular_z_old = cmd_vel_angular_z_;

        tf::Stamped<tf::Pose> x_pose;
        ftc_local_planner::getXPose(*tf_,global_plan_, costmap_ros_->getGlobalFrameID(),x_pose,max_point);
        distance = sqrt(pow((x_pose.getOrigin().getX()-current_pose.getOrigin().getX()),2)+pow((x_pose.getOrigin().getY()-current_pose.getOrigin().getY()),2));
        angle = calculateGlobalPlanAngle(current_pose, global_plan_, max_point);

        //check if max velocity is exceeded
        if((distance/config_.sim_time) > config_.max_x_vel)
        {
            cmd_vel_linear_x_ = config_.max_x_vel;
        }
        else
        {
            cmd_vel_linear_x_ = (distance/config_.sim_time);
        }

        //check if max rotation velocity is exceeded
        if(fabs(angle/config_.sim_time)>config_.max_rotation_vel)
        {
            cmd_vel_angular_z_ = config_.max_rotation_vel;
        }
        else
        {
            cmd_vel_angular_z_ = (angle/config_.sim_time);
        }

        //Calculate new velocity with max acceleration
        if(cmd_vel_linear_x_ > cmd_vel_linear_x_old+config_.acceleration_x/config_.local_planner_frequence)
        {
            cmd_vel_linear_x_ = cmd_vel_linear_x_old+config_.acceleration_x/config_.local_planner_frequence;
        }
        else
        {
            if(cmd_vel_linear_x_ < cmd_vel_linear_x_old-config_.acceleration_x/config_.local_planner_frequence)
            {
                cmd_vel_linear_x_ = cmd_vel_linear_x_old-config_.acceleration_x/config_.local_planner_frequence;
            }
            else
            {
                cmd_vel_linear_x_ = cmd_vel_linear_x_old;
            }
        }

        //Calculate new velocity with max acceleration
        if(fabs(cmd_vel_angular_z_) > fabs(cmd_vel_angular_z_old)+fabs(config_.acceleration_z/config_.local_planner_frequence))
        {
            if(cmd_vel_angular_z_ < 0)
            {
                cmd_vel_angular_z_ = cmd_vel_angular_z_old-config_.acceleration_z/config_.local_planner_frequence;
            }
            else
            {
                cmd_vel_angular_z_ = cmd_vel_angular_z_old+config_.acceleration_z/config_.local_planner_frequence;
            }
        }

        if(cmd_vel_angular_z_ < 0 && cmd_vel_angular_z_old > 0)
        {
            if( fabs(cmd_vel_angular_z_ - cmd_vel_angular_z_old) > fabs(config_.acceleration_z/config_.local_planner_frequence))
            {
                cmd_vel_angular_z_ = cmd_vel_angular_z_old - config_.acceleration_z/config_.local_planner_frequence;
            }
        }

        if(cmd_vel_angular_z_ > 0 && cmd_vel_angular_z_old < 0)
        {
            if( fabs(cmd_vel_angular_z_ - cmd_vel_angular_z_old) > fabs(config_.acceleration_z/config_.local_planner_frequence))
            {
                cmd_vel_angular_z_ = cmd_vel_angular_z_old + config_.acceleration_z/config_.local_planner_frequence;
            }
        }

        //Check at last if velocity is to high.
        if(cmd_vel_angular_z_ > config_.max_rotation_vel)
        {
            cmd_vel_angular_z_ = config_.max_rotation_vel;
        }
        if(cmd_vel_angular_z_ < -config_.max_rotation_vel)
        {
            cmd_vel_angular_z_ = (- config_.max_rotation_vel);
        }
        if(cmd_vel_linear_x_ >  config_.max_x_vel)
        {
            cmd_vel_linear_x_ = config_.max_x_vel;
        }
        //Push velocity to cmd_vel for driving.
        cmd_vel.linear.x = cmd_vel_linear_x_;
        cmd_vel.angular.z = cmd_vel_angular_z_;

        ROS_DEBUG("FTCPlanner: max_point: %d, distance: %f, x_vel: %f, rot_vel: %f, angle: %f", max_point, distance, cmd_vel.linear.x, cmd_vel.angular.z, angle);
        ROS_INFO("FTCPlanner: max_point: %d, distance: %f, x_vel: %f, rot_vel: %f, angle: %f", max_point, distance, cmd_vel.linear.x, cmd_vel.angular.z, angle);
        ROS_INFO(" -------------------------- END -----------------------------");
        return max_point;
    }


    bool FTCPlanner::isGoalReached()
    {
        if(goal_reached_)
        {
            ROS_INFO("FTCPlanner: Goal reached.");
        }
        return goal_reached_;
    }

    bool FTCPlanner::checkCollision(int max_points)
    {
        //maximal costs
        unsigned char previous_cost = 255;

        for (int i = 0; i <= max_points; i++)
        {
            tf::Stamped<tf::Pose> x_pose;
            ftc_local_planner::getXPose(*tf_,global_plan_, costmap_ros_->getGlobalFrameID(),x_pose,i);

            unsigned int x;
            unsigned int y;
            costmap_ros_->getCostmap()->worldToMap(x_pose.getOrigin().getX(), x_pose.getOrigin().getY(), x, y);
            unsigned char costs = costmap_ros_->getCostmap()->getCost(x, y);
            //Near at obstacel
            if(costs > 0)
            {
                if(!rotate_to_global_plan_)
                {
                    ROS_INFO("FTCPlanner: Obstacel detected. Start routine new.");
                }
                rotate_to_global_plan_ = true;

                //Possible collision
                if(costs > 127 && costs > previous_cost)
                {
                    ROS_WARN("FTCPlanner: Possible collision. Stop local planner.");
                    return false;
                }
            }
            previous_cost = costs;
        }
        return true;
    }

    void FTCPlanner::publishPlan(int max_point)
    {
        std::vector<geometry_msgs::PoseStamped> path;

        //Get all points of the global plan which are used and transform them
        for(int i = 0; i <= max_point; i++)
        {
            tf::Stamped<tf::Pose> x_pose;
            ftc_local_planner::getXPose(*tf_,global_plan_, costmap_ros_->getGlobalFrameID(),x_pose,i);
            tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(x_pose,
                                      ros::Time::now(),
                                      costmap_ros_->getGlobalFrameID());
            geometry_msgs::PoseStamped pose;
            tf::poseStampedTFToMsg(p, pose);
            path.push_back(pose);
        }

        //given an empty path we won't do anything
        if(path.empty())
            return;

        //create a path message
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for(unsigned int i=0; i < path.size(); i++)
        {
            gui_path.poses[i] = path[i];
        }

        local_plan_publisher_.publish(gui_path);
    }

    FTCPlanner::~FTCPlanner()
    {
    }
}
