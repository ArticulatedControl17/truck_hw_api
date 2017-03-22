#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

import time
import interpolate
import Adafruit_ADS1x15

PIN = 0

class TrailerSensorNode:
    def __init__(self):
        
       
        interpolate.generateDictionaries()

        self.adc = Adafruit_ADS1x15.ADS1115()
        self.GAIN = 1
        rospy.init_node('trailersensor', anonymous=True)
        self.rate = rospy.Rate(50)
        self.pub = rospy.Publisher('trailer_sensor', Float32, queue_size=10)


    def spin(self): 
        while not rospy.is_shutdown():
            self.rate.sleep()
            value = self.adc.read_adc(PIN, gain=self.GAIN)
            print value
            angle = interpolate.getTrailerAngle(value)
            self.pub.publish(Float32(angle))


if __name__ == '__main__':
    cn = TrailerSensorNode()
    cn.spin()

