#!/usr/bin/env python

import rospy
import signal
import atexit
import time
import os
from driving import Truck
from hw_api_ackermann.msg import AckermannDrive


class TruckNode:
    def __init__(self):
        self.angle_dict = {}
        self.ad_precision = 1
        self.MAX_ANGLE = 0
        self.MIN_ANGLE = 0

        self.speed_forward_dict = {}
        self.sfd_precision = 2

        self.MAX_SPEED = 0
        self.MIN_SPEED = 0

        self.speed_backward_dict = {}
        self.sbd_precision = 2

        
        self.truck = Truck()
    


    def readDictionaries(self):
        dirpath = os.path.dirname(os.path.abspath(__file__))

        
        with open(os.path.join(dirpath, 'dicts/angle_dict.txt'), 'r') as ad, \
             open(os.path.join(dirpath, 'dicts/backward_speed_dict.txt'),'r') as bd, \
             open(os.path.join(dirpath, 'dicts/forward_speed_dict.txt'),'r') as fd:
            
            self.angle_dict = eval(ad.read())
            self.speed_backward_dict = eval(bd.read())
            self.speed_forward_dict = eval(fd.read())


            a_keys = self.angle_dict.keys()
            self.MAX_ANGLE = max(a_keys)
            self.MIN_ANGLE = min(a_keys)

            self.MIN_SPEED = min(self.speed_backward_dict.keys())
            self.MAX_SPEED = max(self.speed_forward_dict.keys())





    def callback(self, data):
            rospy.loginfo(rospy.get_caller_id() + "steering: %s, speed: %s", data.steering_angle, data.speed)
            

            phi = data.steering_angle
            v = data.speed

            if phi > self.MAX_ANGLE :
                    phi = self.MAX_ANGLE
            elif phi < self.MIN_ANGLE:
                    phi = self.MIN_ANGLE

            if v > self.MAX_SPEED:
                    v = self.MAX_SPEED
            elif v < self.MIN_SPEED:
                    v = self.MIN_SPEED


            print "phi " + str(phi)

            steering_cmd = self.angle_dict[round(phi, self.ad_precision)]


            if v >= 0:
                    speed_cmd = self.speed_forward_dict[round(v, self.sfd_precision)]
            else:
                    speed_cmd = self.speed_backward_dict[round(v, self.sbd_precision)]

            print "angle : " + str(steering_cmd)
            print "speed : " + str(speed_cmd)
            
            self.truck.setSteering(steering_cmd)
            self.truck.setSpeed(speed_cmd)
            self.truck.update()

    def listener(self):
            
            
            self.readDictionaries()

            rospy.init_node('truck_cmd_node', anonymous=True)
            self.truck.reset()
            self.truck.update()
            rospy.Subscriber("truck_cmd", AckermannDrive, self.callback)
            rospy.spin()


def interruptHandler(sig, frame):
	signal.signal(signal.SIGINT, signal.SIG_IGN)
	print("Interrupted, reset servo...")
	# Return to neutral
	truck = TruckNode()
	truck.truck.reset()
	truck.truck.update()
	time.sleep(0.4)
	exit(0)

def exit_handler():
        truck = TruckNode()
        truck.truck.reset()
        truck.truck.update()
        exit(0)

signal.signal(signal.SIGINT, interruptHandler)
atexit.register(exit_handler)

if __name__ == '__main__':
	TruckNode().listener()


