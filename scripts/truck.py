#!/usr/bin/env python

#import rospy
from driving import Truck
#from hw_api_ackermann.msg import AckermannDrive

truck = Truck()

angle_dict = {}
ad_precision = 1
MAX_ANGLE = 0
MIN_ANGLE = 0

speed_forward_dict = {}
sfd_precision = 2

MAX_SPEED = 0
MIN_SPEED = 0

speed_backward_dict = {}
sbd_precision = 2


def readDictionaries():
    with open('angle_dict.txt', 'r') as ad, \
         open('backward_speed_dict.txt','r') as bd, \
         open('forward_speed_dict.txt','r') as fd:
        

        global angle_dict, speed_backward_dict, speed_forward_dict, MAX_SPEED, MAX_ANGLE, MIN_SPEED, MIN_ANGLE

        angle_dict = eval(ad.read())
        speed_backward_dict = eval(bd.read())
        speed_forward_dict = eval(fd.read())


        a_keys = angle_dict.keys()
        MAX_ANGLE = max(a_keys)
        MIN_ANGLE = min(a_keys)

        MIN_SPEED = min(speed_backward_dict.keys())
        MAX_SPEED = max(speed_forward_dict.keys())




def callback(data):
	#rospy.loginfo(rospy.get_caller_id() + "steering: %s, speed: %s", data.steering_angle, data.speed)
	

	phi = data.steering_angle
	v = data.speed

	if phi > MAX_ANGLE :
		phi = MAX_ANGLE
	elif phi < MIN_ANGLE:
		phi = MIN_SPEED

	if v > MAX_SPEED:
		v = MAX_SPEED
	elif v < MIN_SPEED:
		v = MIN_SPEED



	steering_cmd = angle_dict[round(phi, ad_precision)]


	if v >= 0:
		speed_cmd = speed_forward_dict[round(v, sfd_precision)]
	else:
		speed_cmd = speed_backward_dict[round(v, sbd_precision)]


	truck.setSteering(steering_cmd)
	truck.setSpeed(speed_cmd)
	truck.update()

def listener():
	
	
	readDictionaries()

	rospy.init_node('truck', anonymous=True)
	truck.reset()
	truck.update()
	rospy.Subscriber("steeringAngle", AckermannDrive, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()


