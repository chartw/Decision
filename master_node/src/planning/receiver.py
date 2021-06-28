#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point

class Receiver():
    def __init__(self, planner):
        self.data = planner.planning_data
        rospy.init_node('Receiver', anonymous=False)

    
    def positionCallback(self,msg):
        self.position_msg=msg
        #msg 뜯어서 data안에 차곡차곡 채우기.

    def obstacleCallback(self,msg):
        self.obstacle_msg=msg

    def objectCallback(self,msg):
        self.object_msg=msg



    
    def run(self):
        
        rospy.Subscriber("/obstacles", Obstacles, self.obstacleCallback)
        rospy.Subscriber("/pose", Odometry, self.positionCallback)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.objectCallback)
        
        rospy.spin()