#!/usr/bin/env python
import rospy, math
from teb_local_planner.msg import ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32


def publish_obstacle_msg():
  # Adjust the topic for navigation or remap
  pub = rospy.Publisher('/test_optim_node/obstacles', ObstacleMsg, queue_size=1)

  rospy.init_node("publish_obstacles")

  # Create empty obstacle message that will be filled in afterwards
  obstacle_msg = ObstacleMsg() 
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "odom" # CHANGE HERE: odom/map
  
  # Add point obstacle
  obstacle_msg.obstacles.append(PolygonStamped())
  obstacle_msg.obstacles[0].polygon.points = [Point32()]
  obstacle_msg.obstacles[0].polygon.points[0].x = 1.5
  obstacle_msg.obstacles[0].polygon.points[0].y = 0
  obstacle_msg.obstacles[0].polygon.points[0].z = 0

  # Add line obstacle
  obstacle_msg.obstacles.append(PolygonStamped())
  line_start = Point32()
  line_start.x = -2.5
  line_start.y = 0.5
  line_end = Point32()
  line_end.x = -2.5
  line_end.y = 2
  obstacle_msg.obstacles[1].polygon.points = [line_start, line_end]
  
  # Add polygon obstacle
  obstacle_msg.obstacles.append(PolygonStamped())
  v1 = Point32()
  v1.x = -1
  v1.y = -1
  v2 = Point32()
  v2.x = -0.5
  v2.y = -1.5
  v3 = Point32()
  v3.x = 0
  v3.y = -1
  obstacle_msg.obstacles[2].polygon.points = [v1, v2, v3]
  

  r = rospy.Rate(10) # 10hz
  t = 0.0
  while not rospy.is_shutdown():
    
    # Here we want to continuously update the point obstacle:
    # vary y-component of the point obstacle according to a sine
    obstacle_msg.obstacles[0].polygon.points[0].y = 1*math.sin(t)
    t = t + 0.1
    
    # Publish the message
    pub.publish(obstacle_msg)
    
    r.sleep()


if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass