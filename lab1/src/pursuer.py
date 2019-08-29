#!/usr/bin/env python

import rospy
import roslib
import tf
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
	

	
def listen():
	rospy.init_node('pursuer', anonymous=True)
	listener = tf.TransformListener()
	pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/robot_1', '/robot_0', rospy.Time(0))
        	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            		continue

		angular = 4 * math.atan2(trans[1], trans[0])
        	linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        	cmd = Twist()
        	cmd.linear.x = linear
        	cmd.angular.z = angular
        	pub.publish(cmd)

        	rate.sleep()


if __name__ == '__main__':
    listen()
