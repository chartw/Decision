#!/usr/bin/env python
import serial #Serial USB
import socket #UDP LAN
import rospy
import pymap3d
from nav_msgs.msg import Odometry

import time

class Localization():
	def __init__(self):
		rospy.init_node('Localization', anonymous=False)
		self.pub = rospy.Publisher('/pose', Odometry, queue_size = 1)
		self.msg = Odometry()
		self.heading = 0
		self.lat = 0
		self.lon = 0

		# print('==========Localization node activated===========')

	def connect(self):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP LAN
		recv_address = ('127.0.0.1', 9092)
		self.sock.bind(recv_address)

		print("Connect!")

	def calc_heading(self, data):
		input = data[49:54]
		heading_str = str(input)[2:5]
		heading = int(heading_str)
		self.heading = (360 - heading - 90) % 360

		# print("yaw:", self.heading)

	def calc_position(self, data):
		lat_str = str(data[16:25])
		lon_str = str(data[28:38])
		lat_int = 37 + float(lat_str[4:11])/60
		lon_int = 126 + float(lon_str[5:12])/60
		self.lon = round(lon_int, 9)
		self.lat = round(lat_int, 9)

	def get_xy(self,  lat,  lon,  alt):
		e, n, u = pymap3d.geodetic2enu(lat, lon, alt, 37.239231667, 126.773156667, 15.400)
		# 'base_lat': 37.383784,  'base_lon': 126.654310,  'base_alt': 15.400,          # songdo base (98th node)
        # 'base_lat': 37.239231667,  'base_lon': 126.773156667,  'base_alt': 15.400,    # kcity base

		return e, n

		# print(self.lon, self.lat)

	def publish_msg(self):
		# for controller
		self.msg.twist.twist.angular.z = self.heading
		# self.msg.pose.pose.position.x = self.lon
		# self.msg.pose.pose.position.y = self.lat
		# print(self.msg.pose.pose.position.x,self.msg.pose.pose.position.y,self.msg.twist.twist.angular.z)
		# for lidar
		x, y = self.get_xy(self.lat, self.lon, 15.400)
		print(x,y,self.msg.twist.twist.angular.z )
		self.msg.pose.pose.position.x = x
		self.msg.pose.pose.position.y = y
		self.pub.publish(self.msg)
		
		# print("Publish!")


	def main(self):
		data, sender = self.sock.recvfrom(1024)
  
		if str(data[1:6]) == "b'GPRMC'":
			self.calc_heading(data)
			print('0000')

		if str(data[1:6]) == "b'GPGGA'":
			self.calc_position(data)
			print('1111')

		self.publish_msg()


if __name__ == '__main__':
    
	loc = Localization()
	rate = rospy.Rate(100)
	# old_time = 0
	# new_time = 0
 
	loc.connect()
 
	while not rospy.is_shutdown():
		loc.main()
		rate.sleep()
		# old_time = new_time
		# new_time = time.time()
		# print(new_time - old_time)
