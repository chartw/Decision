#-*- coding:utf-8 -*-
import glob
import matplotlib.pyplot as plt
import statistics
import csv
import sys
sys.path.append("/home/ming/Downloads/JH/jh")

try:
    import cubic_spline_planner
except:
    raise

ax = []
ay = []
tmp_lane_type = []
lane_type = []
idx_list = []
cx, cy, cyaw, ck, s  = [],[],[],[],[]
# path_cand= open("C:/Users/junma/OneDrive/바탕 화면/course.txt")   # C:/Users/cvlab/Desktop/test/test_track.txt lab: "C:/erp/examples/simulation_example/path.txt",'r' "C:/Users/cvlab/Desktop/test/path.txt",'r'
# lines = path_cand.readlines()
# with open('C:/Users/junma/OneDrive/바탕 화면/csv2map_v3.csv', mode='r') as csv_file:
with open('test_map.csv', mode='r') as csv_file:	
    csv_reader = csv.DictReader(csv_file)
    for next_r in csv_reader:
        ax.append(float(next_r['X']) )
        ay.append(float(next_r['Y']) )
        if next_r['info'] != '':
        	print(next_r['info'])
        	tmp_lane_type.append([int(next_r['X']),int(next_r['Y']),int(next_r['info'])])
        	# tmp_lane_type.append(int(next_r['info']))


# print('ax is ', ax)
# print('ay is ', ay)
print('tmp_lane_type', tmp_lane_type)
print(len(tmp_lane_type))
print('ax range: ', range(len(ax)-1) ) 
print(len(ax))

for i in range(len(ax)-1):
	dis = ((ax[i+1]-ax[i])**2 + (ay[i+1]-ay[i])**2)*0.5
	print('distance', i, ':', dis)
	if dis > 10000:
		idx_list.append(i)
		idx_list.append(i+1)
		
if len(ax)-1 != idx_list[-1]:
	idx_list.append(len(ax)-1)

print('index list : ', idx_list)
print('range: ', range(len(idx_list)-1))
# j = 0
# print('test:',ax[idx_list[j]:idx_list[j+2]] )

for j in range(len(idx_list)-1):
	bx, by, byaw, bk, bs = cubic_spline_planner.calc_spline_course(ax[idx_list[j]:idx_list[j+1]+1], 
		ay[idx_list[j]:idx_list[j+1]+1], ds=5)
	cx = cx + bx
	cy = cy + by
	cyaw = cyaw + byaw
	ck = ck + bk
	for idx in range(len(bs)):
		if len(s) > 0:
			bs[idx] = bs[idx] + s[-1] + 5
	s = s + bs

# for coord in tmp_lane_type:
# 	while range(len(cx)):
# 		dis = ((cx[i]-coord[0])**2 + (cy[i]-coord[1])**2)**0.5
# 		if (dis < 1) :
# 			lane_type.append(coord[2])
			
# 		else:
# 			lane_type.append(0)

# while len(lane_type) < len(cx):
# 	tmp_lane_type[i][0] - cx[j]


# print(lane_type)
# print('len cx', len(cx), 'len lane_type', len(lane_type))



#겹치는거 지워줌
# idx = 2
# while idx < len(cx)-1:
# 	tmp_dis = ((cx[idx]-cx[idx-1])**2 + (cy[idx]-cy[idx-1])**2)**0.5
# 	print(idx)
# 	if tmp_dis < 15:
# 		del cx[idx]
# 		del cy[idx]
# 		del cyaw[idx]
# 		del ck[idx]
# 	idx += 1 

# print('cx is :', cx)


# print(ck)

fig = plt.figure()
plt.grid(True)

#좌표 수정용
# for i in range(len(cx)):
# 	plt.text(cx[i], cy[i], ck[i])

# full_path = "C:/Users/junma/OneDrive/바탕 화면/k-city_test.csv"
f = open("C:/Users/junma/OneDrive/바탕 화면/k-city_test.csv", 'w', newline='')
csv_writer = csv.writer(f)
csv_writer.writerow(["X", "Y", "yaw", "K", "s", "type"])

for i in range(len(cx)):
	csv_writer.writerow( [cx[i], cy[i], cyaw[i], ck[i], s[i], lane_type[i]] )
	# f.write(str(cx[i])+"\t"+str(cy[i])+"\n")
	# f.write(str(cx[i])+"\t"+str(cy[i])+"\t"+str(cyaw[i])+"\t"+str(ck[i])+"\t"+str(s[i])+"\n")

# f.close()

plt.plot(cx,cy)
plt.scatter(cx,cy, marker='.')
plt.show()



# for line in lines:
#     x_cand = line.split('\t')[0]
#     y_cand = line.split('\t')[1]
#     # x_cand = (x_cand-126.653281) * 100000
#     # y_cand = (y_cand-37.3837028) * 100000

#     ax.append(float(x_cand))
#     ay.append(float(y_cand))

# path_cand.close()


# cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.5)
# print()
# f = open('path.txt','w')
# for i in range(len(cx)):
# 	f.write(str(cx[i])+"\t"+str(cy[i])+"\n")

# f.close()



# plt.plot(cx,cy)
# plt.show()
# print('cx is', cx)
# print('cy is', cy)

# print(len(cx))

# a = ((cx[3]-cx[2])**2 + (cy[3]-cy[2])**2)**0.5
# print(a)
