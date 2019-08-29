#!/usr/bin/env python  
import roslib
import rospy
from nav_msgs.msg import Odometry
import tf
import turtlesim.msg

def broadcast_odometry(msg):
	br = tf.TransformBroadcaster()

	br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     rospy.get_namespace(),
                     "world")

if __name__ == '__main__':
	rospy.init_node('bot_broadcaster')
	rospy.Subscriber('odom',Odometry,broadcast_odometry)
	rospy.spin()
