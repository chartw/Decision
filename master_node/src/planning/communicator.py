#!/usr/bin/env python3
import serial

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Time
from geometry_msgs.msg import Point

class Communicator():
    def __init__(self, planner):
        self.data = planner.planning_data
        self.data = planner.control_data
        self.mission = planner.planning_data['mission']
        self.pub = rospy.Publisher('/mission', String, queue_size = 1)
        self.msg = String()

        # self.ser = serial.Serial('/dev/ttyUSB0',115200)

    
    def positionCallback(self,msg):
        self.position_msg=msg
        #msg 뜯어서 data안에 차곡차곡 채우기.

    def obstacleCallback(self,msg):
        self.obstacle_msg=msg

    def objectCallback(self,msg):
        self.object_msg=msg


    def publish_mission(self):
        self.msg = self.data['mission']
        self.pub.publish(self.msg)


    def serialWrite(self):
        steering = self.data['steering']
        break_val = self.data['break_val']
        cnt=0x00
        result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, \
                              int(self.data['speed']), \
                              self.data['steering'], \
                              self.data['break_val'], \
                              cnt, \
                              0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
        
        self.ser.write(result)


    
    def run(self):
        
        # rospy.Subscriber("/obstacles", Obstacles, self.obstacleCallback)
        # rospy.Subscriber("/pose", Odometry, self.positionCallback)
        # rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.objectCallback)


        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.publish_mission()
            self.serialWrite()
            rate.sleep()

        