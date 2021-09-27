#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
import numpy as np
from master_node.msg import Obstacles, PangPang, Planning_Info, Path, Local, Serial_Info
from nav_msgs.msg import Odometry

from sensor_msgs.msg import PointCloud, NavSatFix
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32, Time, String, Int16, Int32

from ublox_msgs.msg import NavPVT
from sensor_msgs.msg import NavSatFix

from lib.planner_utils.sig_int_handler import SigIntHandler


class Monitor:
    def __init__(self):
        self.planning_msg = Planning_Info()
        self.serial_msg = Serial_Info() # Serial Read
        self.control_msg = Serial_Info() # serial Write
        self.parking_msg = Int32()
        self.global_path = Path()
        self.local = Local()
        self.navpvt_msg = NavPVT()
        self.gps_status_list = ["GPS only", 'RTK floating', "RTK fixed!"]
        self.gps_status = -1
        rospy.init_node("Monitor", anonymous=False)

        rospy.Subscriber("/control", Serial_Info, self.controlCallback)
        rospy.Subscriber("/serial", Serial_Info, self.serialCallback)
        rospy.Subscriber("/pose", Odometry, self.localCallback)

        rospy.Subscriber('/gps_data/navpvt',NavPVT, self.navpvtCallback)
        rospy.Subscriber('/gps_data/fix',NavSatFix, self.navsatfixCallback)

        rospy.Subscriber("/planner", Planning_Info, self.planningCallback)

    def localCallback(self, msg):
        self.local.x = msg.pose.pose.position.x
        self.local.y = msg.pose.pose.position.y
        self.local.heading = msg.twist.twist.angular.z

    def serialCallback(self, msg):
        self.serial_msg = msg

    def controlCallback(self, msg): 
        self.control_msg= msg
        
    def navpvtCallback(self, msg):
        self.navpvt_msg = msg
    def navsatfixCallback(self, msg):
        self.gps_status = self.gps_status_list[msg.status.status]
       
    def planningCallback(self, msg):
        self.planning_msg = msg

    def print_everything(self):
        print(f"-----------------------------------------------------------\n\
                    ==Serial Read==             ==Serial Write== \n\
    auto_manual:    {self.serial_msg.auto_manual}                           {self.control_msg.auto_manual} \n\
    emergency_stop: {self.serial_msg.emergency_stop}                           {self.control_msg.emergency_stop} \n\
    gear:           {self.serial_msg.gear}                           {self.control_msg.gear} \n\
    speed:          {self.serial_msg.speed:2f}                    {self.control_msg.speed:2f} \n\
    steer:          {self.serial_msg.steer:5f}                    {self.control_msg.steer:5f} \n\
    brake:          {self.serial_msg.brake:3d}                         {self.serial_msg.brake:3d} \n\
    \n\
    ==Controller==\n\
    path_steer: {self.control_msg.path_steer:5f}   ready: {self.control_msg.ready} \n\
    \n\
    ==Planner==\n\
    mode: {self.planning_msg.mode}      index: {self.planning_msg.cur_index} \n\
    mapinfo: TODOTODOTODO\
    \n\n\
    ==GPS==\n\
    status: {self.gps_status}\n\
    horizontalAcc: {self.navpvt_msg.hAcc:10d}    verticalAcc: {self.navpvt_msg.vAcc:10d} \n\
    headingAcc:    {self.navpvt_msg.headAcc:10d}       speedAcc: {self.navpvt_msg.sAcc:10d} \n\
    ")
    
if __name__ == "__main__":
    SI = SigIntHandler()
    SI.run()
    
    monitor = Monitor()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        monitor.print_everything()
        rate.sleep()