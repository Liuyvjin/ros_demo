#!/usr/bin/python


import rospy
from topic_demo.msg import gps

import math
import numpy as np

def chatterCallback( msg ):
	distance = math.sqrt(pow(msg.x, 2) + pow(msg.y, 2))
	rospy.loginfo("Listener: Distance to origin is %f, state: %d, %d" % (distance, msg.x, msg.y))


if __name__ == '__main__':
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('chatter', gps, chatterCallback)
	rospy.spin()


