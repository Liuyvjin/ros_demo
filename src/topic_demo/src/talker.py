#!/usr/bin/python


import rospy
from topic_demo.msg import gps
import numpy as np

if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    gps_pub = rospy.Publisher('chatter', gps, queue_size=10)
    rate = rospy.Rate(10)

    count = 0
    while not rospy.is_shutdown():
        msg = gps(np.random.randint(100), np.random.randint(100), '')
        gps_pub.publish(msg)
        rospy.loginfo("Talker: x = %d, y = %d" % (msg.x, msg.y))
        rate.sleep()
        count += 1

