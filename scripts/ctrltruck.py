#!/usr/bin/env python

import rospy
from hw_api_ackermann.msg import AckermannDrive

def publisher():
	pub = rospy.Publisher('steeringAngle', AckermannDrive, queue_size=10)
	rospy.init_node('truck', anonymous=True)
	rate = rospy.Rate(2)
	angle = 50
	dir = 1
	while not rospy.is_shutdown():
		angle += dir
		if angle > 90:
			dir = -1.55
		if angle < 15:
			dir = 1.55
		rospy.loginfo("sending msg %s", angle)
		msg = AckermannDrive()
		msg.steering_angle = angle
		msg.speed = 46
		pub.publish(msg)
		rate.sleep()
		
if __name__ == '__main__':
	publisher()
