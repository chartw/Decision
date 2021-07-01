#!/usr/bin/env python
# -*- coding:utf-8 -*-

import serial
import rospy
import struct

from master_node.msg import Serial_Info


class Serial_Node:
    def __init__(self):
        # Serial Connect
        self.ser = serial.Serial("/dev/ttyUSB0", 115200)
        
        # ROS Publish
        rospy.init_node("Serial", anonymous=False)
        serial_pub = rospy.Publisher("/serial", Serial_Info, queue_size=1)
                    
        # ROS Subscribe        
        def controlCallback(self, msg): self.control_input = msg
        rospy.Subscriber("/control", Serial_Info, controlCallback)

        # Messages/Data
        self.serial_msg = Serial_Info() # Message to publish
        self.control_input = Serial_Info()
        self.serial_data = []
        self.alive = 0
        
        
        # Main Loop
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.serialRead()
            # self.serialCal()
            serial_pub.publish(self.serial_msg)
            # self.serialWrite()
            
            rate.sleep()

    def serialRead(self):
        while True :
            packet = self.ser.readline()
            if len(packet)==18:
                header= packet[0:3].decode()
                
                if header == "STX":
                    self.is_data=True
                    self.serial_msg.auto_manual,
                    self.serial_msg.emergency_stop,
                    self.serial_msg.gear = struct.unpack('BBB',packet[3:6])
                    
                    self.serial_msg.speed,
                    self.serial_msg.steer = struct.unpack('2h',packet[6:10])
                    
                    self.serial_msg.brake = struct.unpack('B',packet[10:11])
                    
                    self.serial_msg.encoder = struct.unpack('f',packet[11:15])
                    self.alive=struct.unpack('B',packet[15:16])
                    print(self.speed,self.steer)
                    
                    
    def serialCal(self):
        self.serial_msg.auto_manual =  int(self.serial_data[3])
        self.serial_msg.emergency_stop = int(self.serial_data[4])
        self.serial_msg.gear = int(self.serial_data[5])
        self.serial_msg.speed = int(self.serial_data[6] / 10) #km/h

        self.serial_msg.steer = int(
                                    self.serial_data[8]
                                    + 256*self.seria_data[7]      
                                    )
        self.serial_msg.brake = int(self.serial_data[10])
        self.serial_msg.encoder = float(
                                        self.serial_data[11]
                                        + 256*self.serial_data[12]
                                        + 65536*self.serial_data[13]
                                        + 16777216*self.serial_data[14]
                                        )

    def serialWrite(self):
        steering = self.control_data["steering"]
        break_val = self.control_data["break_val"]
        cnt = 0x00
        result = struct.pack(
            "!BBBBBBHhBBBB",
            0x53,
            0x54,
            0x58,
            0x01,
            self.control_input.emergency_stop,
            self.control_input.gear,
            self.control_input.speed,
            self.control_input.steer,
            self.control_input.brake,
            cnt,
            0x0D,
            0x0A,
        )  # big endian 방식으로 타입에 맞춰서 pack

        self.ser.write(result)
        
        
        
erp = Serial_Node()

# while True:
#     erp.serialRead()