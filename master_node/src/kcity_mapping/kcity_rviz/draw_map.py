#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import csv
# matplot 으로 그냥 전체 함수 다 그려주는 파일.

# waypoints = []
# curvature = []

# plt.close()
# fig = plt.figure()
# plt.grid(True)

# for i in range(0,10):
#     path = "/home/junhyeok/Desktop/junhyeok/lane_" + str(i) + ".csv"
#     with open(path, mode='r') as csv_file:
#         csv_reader = csv.DictReader(csv_file)
#         globals()['lane'+str(i)+'x'] =[]
#         globals()['lane'+str(i)+'y'] =[]
#         for next_r in csv_reader:
#             globals()['lane'+str(i)+'x'].append(str(next_r['X']) )
#             globals()['lane'+str(i)+'y'].append(str(next_r['Y']) )
#         plt.plot(globals()['lane'+str(i)+'x'], globals()['lane'+str(i)+'y'], marker ='.')
#         # print(globals()['lane'+str(i)+'x'])
# plt.show()

ax, ay = [], []
lane_list = ['21', '163', '38','144','36','140', '32','136','31']
# for idx in lane_list:
for idx in range(1,164):
    path = "/home/ming/Downloads/JH/jh/lanes/lane_" + str(idx) + ".csv"
    with open(path, mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)

        globals()['lane'+str(idx)+'x'] =[]
        globals()['lane'+str(idx)+'y'] =[]

        for next_r in csv_reader:
            ax.append(float(next_r['x']) )
            ay.append(float(next_r['y']) )
            globals()['lane'+str(idx)+'x'].append(float(next_r['x']) )
            globals()['lane'+str(idx)+'y'].append(float(next_r['y']) )
        plt.scatter(globals()['lane'+str(idx)+'x'], globals()['lane'+str(idx)+'y'], marker='.')

        plt.text(globals()['lane'+str(idx)+'x'][len(globals()['lane'+str(idx)+'x'])//2], globals()['lane'+str(idx)+'y'][len(globals()['lane'+str(idx)+'y'])//2], "lane_{}".format(idx))
    csv_file.close()    

# plt.scatter(ax,ay, marker='.')
plt.show()

# ax,ay 쭉 담은 entire_path.csv 파일 만들어줌.
path2 = "/home/ming/Downloads/JH/jh/entire_path.csv"
file = open(path2,'w') #있는걸 open하는거 아님. 걍 새로 열어서 써주는거.
cs = csv.writer(file)
cs.writerow(["X", "Y"])

for i in range(len(ax)):
    cs.writerow( [ round(float(ax[i]),3), round(float(ay[i]),3) ] ) 

