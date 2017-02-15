#!/usr/bin/env python
import signal
import interpolate
import atexit
import time
import rospy
from truck_drive import Truck
from ackermann_msgs.msg import AckermannDrive


class CommandNode:
    def __init__(self):

        interpolate.generateDictionaries()
        interpolate.setRosParams()

        self.last_message_time = 0
        self.last_speed = 0
		
        self.truck = Truck()
        self.truck.reset()
        self.truck.update()

        rospy.init_node('cmd_node', anonymous=True)
        rospy.Subscriber('master_drive', AckermannDrive, self.callback)



    def callback(self, data):
        phi = data.steering_angle
        v = data.speed
        print phi

        self.last_message_time = rospy.get_time()
        self.last_speed = v

        # look up what command to send to truck
        steering_cmd = interpolate.getSteeringCmd(phi)
        speed_cmd = interpolate.getSpeedCmd(v)

        print steering_cmd, speed_cmd

        self.truck.setSteering(steering_cmd)
        self.truck.setSpeed(speed_cmd)
        self.truck.update()
			

    def spin(self): 

        #rospy.spin()    
        while not rospy.is_shutdown():
            #if truck is moving and last message was a long time ago, stop truck
            if self.last_speed >= 0:
                if rospy.get_time() - self.last_message_time > 0.2:
                    print "didnt receive a message in 0.4 sec, resetting"
                    self.truck.reset()
                    self.truck.update()
            
            rospy.sleep(0.1)

# on ctrl c
def interruptHandler(sig, frame):
	signal.signal(signal.SIGINT, signal.SIG_IGN)
	print("Interrupted, reset servo...")
	# Return to neutral
	cn = CommandNode()
	cn.truck.reset()
	cn.truck.update()
	time.sleep(0.4)
	exit(0)

#if program crashes
def exit_handler():
    cn = CommandNode()
    cn.truck.reset()
    cn.truck.update()
    exit(0)

signal.signal(signal.SIGINT, interruptHandler)
atexit.register(exit_handler)

if __name__ == '__main__':
    cn = CommandNode()
    cn.spin()


