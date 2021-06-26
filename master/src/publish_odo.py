#!/usr/bin/env python
# license removed for brevity

import rospy
from nav_msgs.msg import Odometry
from integration.msg import sync_info
from time import sleep

#pub = rospy.Publisher('/pub_odo', Odometry, queue_size = 1)
pub = rospy.Publisher('/sync_node', sync_info, queue_size = 1)
rospy.init_node('pub_node')
rospy.loginfo("-------a start!-------")

while True:
    sleep(0.1)
    msg = sync_info()
    msg.gps_x = 12.2
    msg.sign = 'abcd'
    print(msg.gps_x)
    print(msg.sign)
    pub.publish(msg)
