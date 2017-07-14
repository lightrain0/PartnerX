#!/usr/bin/env python

import rospy
import tf
from sim_platform.msg import movePathAction,movePathGoal,movePathResult,movePathFeedback
from actionlib import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler as qfe
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PolygonStamped, Point, PoseStamped

class MovePathNode(object):
    """
    This is a ROS node that is responsible for moveing the points planned in the path using move_base package
    """
    def __init__(self):
        # Setup ROS node
        rospy.init_node('move_path')

        self._sas = SimpleActionServer("server_move_path",movePathAction,execute_cb=self.execute_cb)
        self.current_destination = None
        self.previous_destination = None
         # Create the actionlib service
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
  
        # pos1 = PoseStamped()
        # pos2 = PoseStamped()
        # poly = Path()
        # poly.poses.append(pos1)
        # poly.poses.append(pos2)
        # self.move_path(poly,0,1)
        
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()
    def distance(self, p1, p2):
        """Returns the distance between two points."""
        from math import sqrt
        dx = p2.target_pose.pose.position.x - p1.target_pose.pose.position.x
        dy = p2.target_pose.pose.position.y - p1.target_pose.pose.position.y
        return sqrt(dx**2 + dy**2)
    def move_path(self,path,start_number,end_number):
        
        if(start_number >= len(path.poses) or end_number >= len(path.poses)):
            rospy.logwarn("Wrong number of start %d or end %d position , len %d" % (start_number,end_number,len(goal_msg.path.poses)))
            return -1
        

        if(end_number <= 0):
            end_number = len(path.poses) - 1
        if(start_number < 0):
            start_number = 0
        cur_number = start_number
        feedback = movePathFeedback()
        result = movePathResult()
        for index, waypoint in enumerate(path.poses):
            if(self._sas.is_preempt_requested() or rospy.is_shutdown()):
                self._sas.set_preempted()
                rospy.logwarn("move_path received preempt_request")
                return -2
            if(index < start_number):
                continue
            if(index > end_number):
                break

            cur_number = index
            # Push the goal to the actionlib server
            destination = MoveBaseGoal()
            destination.target_pose.header.frame_id = waypoint.header.frame_id
            destination.target_pose.header.stamp = rospy.Time.now()
            # Set the target location
            destination.target_pose.pose.position.x = waypoint.pose.position.x
            destination.target_pose.pose.position.y = waypoint.pose.position.y
            rotate = waypoint.pose.orientation.w
            
            
            # Calculate the distance
            if self.previous_destination == None:
                self.current_distance = 5.0
            else:
                self.current_distance = self.distance(self.previous_destination, destination)
                from math import atan2
                rotate = atan2(destination.target_pose.pose.position.y-self.previous_destination.target_pose.pose.position.y,
                                    destination.target_pose.pose.position.x - self.previous_destination.target_pose.pose.position.x)
                
            # Set the heading
            quat = qfe(0, 0, rotate) 
            destination.target_pose.pose.orientation.x = quat[0]
            destination.target_pose.pose.orientation.y = quat[1]
            destination.target_pose.pose.orientation.z = quat[2]
            destination.target_pose.pose.orientation.w = quat[3]
            # Send the desired destination to the actionlib server
            rospy.loginfo("Sending waypoint %s" % waypoint)
            
            ###
            r_dest = MoveBaseGoal()
            if self.previous_destination != None:
                r_dest = self.previous_destination
                r_dest.target_pose.pose.orientation = destination.target_pose.pose.orientation
            else:
                r_dest = destination
            self.move_base_client.send_goal(r_dest)
            if(self.move_base_client.wait_for_result() == True):
                rospy.loginfo("Sending rotation ")
            else:
                self._sas.set_aborted()
                rospy.logwarn("move_path wait for result failed")
                return -1
            rospy.loginfo("Sending goal orientaion angle %f while current orientation %f ",destination.target_pose.pose.orientation.w,r_dest.target_pose.pose.orientation.w)
            ###

            self.current_destination = destination
            self.move_base_client.send_goal(destination)
            self.previous_destination = destination
            if(self.move_base_client.wait_for_result() == True):
                rospy.loginfo("Reaching the %d of %d position  "% (cur_number,end_number))
                feedback.cur_number = cur_number
                self._sas.publish_feedback(feedback)
            else:
                self._sas.set_aborted()
                rospy.logwarn("move_path wait for result failed")
                return -1
            return cur_number
    def execute_cb(self,goal_msg):
        connected_to_move_base = False
        dur = rospy.Duration(1.0)
        while not connected_to_move_base:
            # Wait for the server for a while
            connected_to_move_base = self.move_base_client.wait_for_server(dur)
            # Check to make sure ROS is ok still
            if rospy.is_shutdown(): return 
            # Update the user on the status of this process
            msg = "Path Planner: waiting on move_base."
            rospy.loginfo(msg)

        start_number = goal_msg.start_number
        end_number = goal_msg.end_number
        
        if(start_number >= len(goal_msg.path.poses) or end_number >= len(goal_msg.path.poses)):
            self._sas.set_aborted()
            rospy.logwarn("Wrong number of start %d or end %d position , len %d" % (start_number,end_number,len(goal_msg.path.poses)))
            return 
        
        if(end_number <= 0):
            end_number = len(goal_msg.path.poses) - 1
        if(start_number < 0):
            start_number = 0
        cur_number = start_number
        feedback = movePathFeedback()
        result = movePathResult()
        for index, waypoint in enumerate(goal_msg.path.poses):
            if(self._sas.is_preempt_requested() or rospy.is_shutdown()):
                self._sas.set_preempted()
                self.move_base_client.cancel_all_goals()
                rospy.logwarn("move_path received preempt_request")
                return 
            if(index < start_number):
                continue
            if(index > end_number):
                break

            cur_number = index
            # Push the goal to the actionlib server
            destination = MoveBaseGoal()
            destination.target_pose.header.frame_id = waypoint.header.frame_id
            destination.target_pose.header.stamp = rospy.Time.now()
            # Set the target location
            destination.target_pose.pose.position.x = waypoint.pose.position.x
            destination.target_pose.pose.position.y = waypoint.pose.position.y
            rotate = waypoint.pose.orientation.w
            
            
            # Calculate the distance
            if self.previous_destination == None:
                self.current_distance = 5.0
            else:
                self.current_distance = self.distance(self.previous_destination, destination)
                from math import atan2
                rotate = atan2(destination.target_pose.pose.position.y-self.previous_destination.target_pose.pose.position.y,
                                    destination.target_pose.pose.position.x - self.previous_destination.target_pose.pose.position.x)
                
            # Set the heading
            quat = qfe(0, 0, rotate) 
            destination.target_pose.pose.orientation.x = quat[0]
            destination.target_pose.pose.orientation.y = quat[1]
            destination.target_pose.pose.orientation.z = quat[2]
            destination.target_pose.pose.orientation.w = quat[3]
            # Send the desired destination to the actionlib server
            rospy.loginfo("Sending waypoint %s" % waypoint)
            
            ###
            r_dest = MoveBaseGoal()
            if self.previous_destination != None:
                r_dest = self.previous_destination
                r_dest.target_pose.pose.orientation = destination.target_pose.pose.orientation
            else:
                r_dest = destination
            self.move_base_client.send_goal(r_dest)
            rospy.loginfo("Sending rotation %s",r_dest)
            if(self.move_base_client.wait_for_result() == True):
                rospy.loginfo("Sending rotation success ")
            else:
                self._sas.set_aborted()
                rospy.logwarn("Sending rotation goal failed")
                return 
            rospy.loginfo("Sending goal orientaion angle %f while current orientation %f ",destination.target_pose.pose.orientation.w,r_dest.target_pose.pose.orientation.w)
            ###

            self.current_destination = destination
            self.move_base_client.send_goal(destination)
            self.previous_destination = destination
            rospy.loginfo("Sending goal  %s",destination)

            if(self.move_base_client.wait_for_result() == True):
                rospy.loginfo("Reaching the %d of %d position  "% (cur_number,end_number))
                feedback.cur_number = cur_number
                self._sas.publish_feedback(feedback)
            else:
                self._sas.set_aborted()
                return 
        result.cur_number = cur_number   
        self._sas.set_succeeded(result)
        rospy.loginfo("Reaching all positions with last number %d" % cur_number)

if __name__ == '__main__':
    ppn = MovePathNode()
    