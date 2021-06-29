#!/usr/bin/env python3
import serial

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Time
from geometry_msgs.msg import Point

class Communicator():
    def __init__(self, planner):
        self.planning_data = planner.planning_data
        self.control_data = planner.control_data
        self.mission = planner.planning_data['mission']
        self.mission_pub = rospy.Publisher('/mission', String, queue_size = 1)
        self.mission_msg = String()
        self.serial_msg = ERP_Serial() # 개발할 메세지 타입
        self.ser = serial.Serial('/dev/ttyUSB0',115200)

    
    def positionCallback(self,msg):
        position=msg.pose.pose.position
        angular=msg.twist.twist.angular
        self.planning_data['cur_x']=position.x
        self.planning_data['cur_y']=position.y
        self.planning_data['cur_yaw']=angular.z

        #msg 뜯어서 data안에 차곡차곡 채우기.

    def obstacleCallback(self,msg):
        self.obstacle_msg=msg

    def objectCallback(self,msg):
        self.object_msg=msg


    def publish_mission(self):
        self.mission_msg = self.planning_data['mission']
        self.mission_pub.publish(self.mission_msg)
    
    def publish_serial(self):
        self.serial_msg = self.planning_data['mission']
        self.mission_pub.publish(self.serial_msg)
    
    def serialRead(self):
        # serial_data를 가공하여 어디에 저장할지가 관건?
        cnt=0x00
        serial_input = self.ser.readline()
        self.ser.flushInput()
        # print(serial_input)
        # print(serial_input[0])

        if (serial_input[0] is 0x53 and serial_input[1] is 0x54 and serial_input[2] is 0x58):
            serial_data = []

            while True:
                for i in range(len(serial_input)):
                    if serial_input[i] is 0x0A and i is not 17:
                        # print("### 0x0A Found!", i, "th data")
                        serial_data.append(0x0B)
                    else :
                        serial_data.append(serial_input[i])

                if len(serial_data) < 18:
                    serial_input = self.ser.readline()
                else:
                    break

            cnt = int(serial_data[15])
            self.V_veh = int(serial_data[6])

    def serialWrite(self):
        steering = self.control_data['steering']
        break_val = self.control_data['break_val']
        cnt=0x00
        result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, \
                              int(self.control_data['speed']), \
                              self.control_data['steering'], \
                              self.control_data['break_val'], \
                              cnt, \
                              0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
        
        self.ser.write(result)


    
    def run(self):
        
        # rospy.Subscriber("/obstacles", Obstacles, self.obstacleCallback)
        # rospy.Subscriber("/pose", Odometry, self.positionCallback)
        # rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.objectCallback)


        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.serialRead()
            self.publish_serial()
            self.publish_mission()
            self.serialWrite()
            rate.sleep()

        