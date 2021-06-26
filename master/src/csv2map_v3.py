#!/usr/bin/python2


'''
K-City Map (EPSG:4326) Parse out 126.77/37.2 10000-2484/1000000-379547
to GOD JI UNG
--FIX--
Line 25 -> (all_lane.csv) for all lane // (out_lane.csv) for contour lane
Line 65 for Lane Thickness
Line 69 for Resolution change

'''
import cv2
import csv
import numpy as np
x_list=[]
y_list=[]
#x_list.append('X')
#y_list.append('Y')
cnt=0
# full_path = "/home/nvidia/" + raw_input("file name: ") + ".csv"
full_path = "/home/wego/catkin_ws/src/master/src/waypoint/csv2map_v3.csv"
csv_writer = csv.writer(open(full_path, mode='w'))
csv_writer.writerow(["X", "Y"])

def draw_circle(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(dst, (x, y), 3, (0, 255, 0), -1)
        dx = int(x) * 2
        dy = (900 - int(y)) * 2
        #x_list.append(dx)
       # y_list.append(dy)
        print(dx, dy)
        csv_writer.writerow([dx, dy])
    elif event == cv2.EVENT_RBUTTONDOWN:
        #x_list.pop()
        #y_list.pop()
        print('save csv')
        cv2.destroyAllWindows()
        exit(0)
'''
img = cv2.imread('songdo_map.jpg')
dst = cv2.resize(img,dsize=(1750,900),interpolation=cv2.INTER_LINEAR_EXACT)
cv2.flip(dst,0,dst)
#Height,Width=img.shape
#print(Height,Width)
cv2.namedWindow('image')
cv2.moveWindow('image',40,30)
cv2.setMouseCallback('image', draw_circle)
csv_writer = csv.writer(open('1.csv', mode='w'))
csv_writer.writerow(["X", "Y"])
csv_writer.writerow([dx, dy])
'''

if __name__ == "__main__":
    img = cv2.imread("/home/wego/catkin_ws/src/master/src/map/songdo_map_.jpg")
    dst = cv2.resize(img,dsize=(1750,900),interpolation=cv2.INTER_LINEAR_EXACT)
    
    cv2.flip(dst,0,dst)
#Height,Width=img.shape
#print(Height,Width)
    cv2.namedWindow('image')
    cv2.moveWindow('image',40,30)
    cv2.setMouseCallback('image', draw_circle)
    while(1):
        cv2.imshow('image',dst)
        cv2.waitKey(1)
