#!/usr/bin/env python

import rospy
from driving import Truck
from hw_api_ackermann.msg import AckermannDrive

truck = Truck()



def callback(data):
	#rospy.loginfo(rospy.get_caller_id() + "steering: %s, speed: %s", data.steering_angle, data.speed)
	truck.setSteering(data.steering_angle)
	truck.setSpeed(data.speed)
	truck.update()

def listener():
	rospy.init_node('truck', anonymous=True)
	truck.reset()
	truck.update()
	rospy.Subscriber("steeringAngle", AckermannDrive, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
