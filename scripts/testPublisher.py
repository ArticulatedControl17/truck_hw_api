#!/usr/bin/env python

import rospy
from hw_api_ackermann.msg import AckermannDrive

def publisher():
	pub = rospy.Publisher('truck_cmd', AckermannDrive, queue_size=10)
	rospy.init_node('testPublisher', anonymous=True)
	rate = rospy.Rate(30)
	angle = 0
	speed = 0
	dir1 = 1
	dir2 = 1
	rate1 = 0.5
	rate2 = 0.01
	while not rospy.is_shutdown():
		angle += dir1 * rate1
		speed += dir2 * rate2

		if angle > 16:
			dir1 = -1
		if angle < -21:
			dir1 = 1

                if speed < -1:
                        dir2 = 1
                if speed > 1:
                        dir2 = -1
                
		rospy.loginfo("sending msg: angle %s, speed %s", angle, speed)
		msg = AckermannDrive()
		msg.steering_angle = angle
		msg.speed = speed
		pub.publish(msg)
		rate.sleep()
		
if __name__ == '__main__':
	publisher()
