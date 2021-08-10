#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from adas_map import AdasMap
import csv
import matplotlib.pyplot as plt
# test.py : adas_map 활용해서 lane_.csv 쭉 새로 만들어줌.(kcity.shp 정밀지도 geopandas 사용해서 가져옴)


# rospy.init_node('map_server', anonymous=False) # 왠지 몰라도 켜줘야 실행됨.

# waypoints = []
# curvature = []

# id_list = ['21','149','26','162','24','130','31','117','42','44','79','48','51','54','58','113','62','5','72','82','40','121','30','132','23','151','20']
# id_list = []

# for id in id_list:
# 	wp = adas_map.lanelet[id]['waypoints']
# 	curve = adas_map.lanelet[id]['k']
# 	waypoints.extend(wp)
# 	curvature.extend(curve)
adas_map = AdasMap() 

for i in range(0,164):
	path = "/home/ming/Downloads/JH/jh/kcity_lanes/" + str(i) + ".csv" # file (will be made)
	f = open(path,'w')
	csv_writer = csv.writer(f)
	csv_writer.writerow(["x", "y", "yaw", "k", "s", "max_v"]) # top row.

	# for idx in adas_map.lanelet[str(i)]['waypoints']:
	# 	csv_writer.writerow( [ float(idx[1]), float(idx[0]) ] )
	for idx in range(len(adas_map.lanelet[str(i)]['waypoints'])): # 한 lane안의 점 개수
		tmp_lane = adas_map.lanelet[str(i)]
		print('waypoint{}of lane{}:{}'.format(idx,i,tmp_lane))
		csv_writer.writerow([ tmp_lane['waypoints'][idx][0] , tmp_lane['waypoints'][idx][1] , tmp_lane['yaw'][idx], tmp_lane['k'][idx], tmp_lane['s'][idx], tmp_lane['max_v'][idx] ])

	# for idx in adas_map.lanelet.keys():
	# 	# print(idx)
	# 	for j in range(len(adas_map.lanelet[idx]['yaw'])):
	# 		tmp_lane = adas_map.lanelet[idx]
	# 		csv_writer.writerow( [ tmp_lane['waypoints'][1], tmp_lane['waypoints'][0], tmp_lane['yaw'], tmp_lane['k'], tmp_lane['s'], tmp_lane['max_v'] ] )
		
		# csv_writer.writerow( [ tmp_lane['waypoints'][1], tmp_lane['waypoints'][0], tmp_lane['yaw'], tmp_lane['k'], tmp_lane['s'], tmp_lane['max_v'] ] )


print ('lane.csv files have made.')

# rospy.spin()




# print curvature

# print(waypoints[-1][0])

# f = open("/home/junhyeok/Desktop/junhyeok/test_map.csv", 'w')
# csv_writer = csv.writer(f)
# csv_writer.writerow(["X", "Y"])

# for idx in waypoints:
# 	csv_writer.writerow( [ float(idx[0]), float(idx[1]) ] )


# with open('/home/junhyeok/Desktop/junhyeok/test_map.csv', mode='r') as csv_file:	
#     csv_reader = csv.DictReader(csv_file)
#     for next_r in csv_reader:
#         ax.append(str(next_r['X']) )
        # ay.append(str(next_r['Y']) )
    

# pt.close()
# plt.grid(True)

# for idx in waypoints:
# 	ax.append(idx[0])
# fig = plt.figure()
# 	ay.append(idx[1])
# plt.plot(ax,ay)
# plt.scatter(a,ay, maker='.')
# plt.show(