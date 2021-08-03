#!/usr/bin/env python
# -*- coding:utf-8 -*-

import serial
import rospy
import struct
import threading

from master_node.msg import Serial_Info


class Serial_Node:
    def __init__(self):
        # Serial Connect
        self.ser = serial.Serial("/dev/ttyUSB0", 115200)


        # ROS Publish
        rospy.init_node("Serial", anonymous=False)
        self.serial_pub = rospy.Publisher("/serial", Serial_Info, queue_size=1)

        

        rospy.Subscriber("/control", Serial_Info, self.controlCallback)

        # Messages/Data
        self.serial_msg = Serial_Info()  # Message to publish
        self.control_input = Serial_Info()
        self.serial_data = []
        self.alive = 0


        # Serial Read Thread
        th_serialRead = threading.Thread(target=self.serialRead)
        th_serialRead.daemon = True
        th_serialRead.start()

        # Main Loop
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # print("----------loop!")
            # self.serialRead()
            # print(self.control_input)
            self.serial_pub.publish(self.serial_msg)
            self.serialWrite()
            rate.sleep()

    def serialRead(self):
        while True:
            packet = self.ser.readline()
            if len(packet) == 18:
                header = packet[0:3].decode()

                if header == "STX":
                    self.is_data = True

                    tmp1, tmp2, tmp3 = struct.unpack("BBB", packet[3:6])
                    self.serial_msg.auto_manual = tmp1
                    self.serial_msg.emergency_stop = tmp2
                    self.serial_msg.gear = tmp3

                    tmp1, tmp2 = struct.unpack("2h", packet[6:10])
                    self.serial_msg.speed = tmp1 / 10  # km/h
                    self.serial_msg.steer = -tmp2 / 71  # degree
                    # print("speed", tmp1, "steer", tmp2)

                    tmp3 = struct.unpack("B", packet[10:11])
                    self.serial_msg.brake = tmp3[0]
                    # print("brake", tmp3[0])

                    tmp1 = struct.unpack("f", packet[11:15])
                    self.serial_msg.encoder = tmp1[0]
                    # print("encoder", tmp1[0])

                    self.alive = struct.unpack("B", packet[15:16])[0]
                    # print(self.alive)
            print(self.serial_msg)
            self.serial_pub.publish(self.serial_msg)

    def serialWrite(self):
        if self.control_input.speed > 20:
            self.control_input.speed = 20
            
        if self.control_input.brake > 200:
            self.control_input.brake = 200

        if self.control_input.steer > 27.7:
            self.control_input.steer = 27.7
        elif self.control_input.steer < -27.7:
            self.control_input.steer = -27.7


        print(self.control_input.auto_manual)
        print("#######################")
        print(self.control_input)
        # a = input()
        result = struct.pack(
            ">BBBBBBHhBBBB",
            0x53,
            0x54,
            0x58,
            self.control_input.auto_manual,
            self.control_input.emergency_stop,
            self.control_input.gear,
            int(self.control_input.speed * 10),
            int(self.control_input.steer * 71),
            self.control_input.brake,
            self.alive,
            0x0D,
            0x0A
            
        )  # big endian 방식으로 타입에 맞춰서 pack
        # tail = '\r\n'.encode()
        self.ser.write(result)
        # print('alive', self.alive)

        # if self.alive < 255:
        #     self.alive += 1
        # else:
        #     self.alive = 0
    # ROS Subscribe
    def controlCallback(self, msg):
        self.control_input = msg


erp = Serial_Node()