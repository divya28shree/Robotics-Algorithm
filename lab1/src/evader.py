#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan



class Evader:
	rangeList = []

	def callback(self,data):
		self.rangeList=data.ranges
		return

	def evade(self):
		rospy.init_node('evader', anonymous=True)
		rospy.Subscriber("base_scan",LaserScan,self.callback)
		pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			move = 0
			if len(self.rangeList) > 0:	
				for i in range(0,180):
					if self.rangeList[i] < 0.5:
						move = 1
						break

			
			twist = Twist()
			if move == 0:
				twist.linear.x = 2.0;
				twist.linear.y = 0.0;
				twist.linear.z = 0.0;
				twist.angular.x = 0.0;
				twist.angular.y = 0.0;
				twist.angular.z = 0.0;
			else:

				twist.linear.x = 0.0;
				twist.linear.y = 0.0;
				twist.linear.z = 0.0;
				twist.angular.x = 0.0;
				twist.angular.y = 0.0;
				twist.angular.z = .5;
			pub.publish(twist)
			rate.sleep()	

	

def main():
	evader = Evader()
	evader.evade()

if __name__=="__main__":
	main()
		
