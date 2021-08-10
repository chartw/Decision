#!/usr/bin/env python

import rospy
import time
from nav_msgs.msg import Odometry

def test():

    rospy.init_node('simul_gps_test',anonymous=True)
    pub=rospy.Publisher('/sim_gps',Odometry,queue_size=10)
    r=rospy.Rate(0.5)
    # for message
    odom=Odometry()
    odom.header.frame_id= 'world'
    odom.header.stamp= rospy.Time.now()
    odom.pose.pose.position.x= 126.773156667
    odom.pose.pose.position.y= 37.239231667
    odom.twist.twist.angular.z= 15.4

    while not rospy.is_shutdown():
        odom.pose.pose.position.x += 0.000001
        odom.pose.pose.position.y += 0.000002
        print('sending_test_gps:',odom.pose.pose.position.x,odom.pose.pose.position.y)
        print(odom.header.stamp)
        pub.publish(odom)
        r.sleep()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass