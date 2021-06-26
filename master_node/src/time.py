#!/usr/bin/env python

from std_msgs.msg import Time
import rospy


pub = rospy.Publisher('/timer', Time, queue_size = 1)
rospy.init_node("time",anonymous=True)
rate = rospy.Rate(10) #hz
test = Time()
while not rospy.is_shutdown():
    test.data = rospy.Time.now()
    pub.publish(test)
    rate.sleep()

