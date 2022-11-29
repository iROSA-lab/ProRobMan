#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Point, Vector3
from control_msgs.msg import PointHeadActionGoal

rospy.init_node('move_head', anonymous=True)

head_movement = rospy.Publisher('/head_controller/point_head_action', PointHeadActionGoal, queue_size=1)
def send_goal():
    message=PointHeadActionGoal()
    # geometry_msgs/PointStamped target
    message.goal.target.header.stamp = rospy.Time.now()
    message.goal.target.header.frame_id = "map"
    message.goal.target.point = Point( 0.5, 0, 0.5)
    # geometry_msgs/Vector3 pointing_axis
    message.goal.pointing_axis.z = 1.0
    message.goal.pointing_frame = "map"
    message.goal.max_velocity = 1.0
    message.goal.min_duration = rospy.Duration(0.15)
    head_movement.publish(message)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    send_goal()
    r.sleep()