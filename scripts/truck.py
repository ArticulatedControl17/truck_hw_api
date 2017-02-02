#!/usr/bin/env python

import rospy
from driving import Truck
from hw_api_ackermann.msg import AckermannDrive

truck = Truck()

def callback(data):
        #i= 1
	#rospy.loginfo(rospy.get_caller_id() + "steering: %s, speed: %s", data.steering_angle, data.speed)
        
	truck.SetSteering(data.steering_angle)
	truck.SetSpeed(data.speed)
	truck.Update()

def listener():
	rospy.init_node('truck', anonymous=True)
	truck.Reset()
	truck.Update()
	rospy.Subscriber("steeringAngle", AckermannDrive, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
