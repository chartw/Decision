#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String

class Transmitter():
    def __init__(self, planner):
        rospy.init_node('Transmitter', anonymous=False)

        self.data = planner.control_data
        self.mission = planner.planning_data['mission']
        self.pub = rospy.Publisher('/mission', String, queue_size = 1)
        self.msg = String()

        self.ser = serial.Serial('/dev/ttyUSB0',115200)


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

    def timerCallback(self, msg):
        self.publish_mission()
        self.serialWrite()

    
    def run(self):
        rospy.Subscriber("/timer", Time, self.timerCallback)
        rospy.spin()