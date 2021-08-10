#-*- coding:utf-8 -*-

import sys,os,csv,serial,struct,time,pymap3d,rospy

from math import radians,degrees,sin,cos,hypot,atan2
import numpy as np

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from master_node.msg import Obstacles 
from master_node.msg import PangPang
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
# from std_msgs.msg import int32
# from hybrid_a_star import path_plan 

# sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/map1/")
# try:
#     import makeTarget
# except:
#     raise

# 전역변수
LocalPath = 0
GlobalPath = 1

class Controller:
    def __init__(self,  master): #  초기 변수.
        self.control_data = master.control_data
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        self.lpath_x = []
        self.lpath_y = []
        self.cur_idx = 0
        #------------------------------------------------------
        self.gpaths=PointCloud()
        self.gpaths.header.frame_id='world'
        self.lpaths=PointCloud()
        self.lpaths.header.frame_id='world'
        self.cur_points=PointCloud()
        self.cur_points.header.frame_id = 'world'
        self.pub_gp=rospy.Publisher('/gpath',PointCloud,queue_size=1)
        self.pub_lp=rospy.Publisher('/lpath',PointCloud,queue_size=1)
        self.pub_c=rospy.Publisher('/cur_xy',PointCloud,queue_size=1)
        #------------------------------------------------------

        self.goal_x = 0
        self.goal_y = 0
        self.gx = 0
        self.gy = 0
        self.obx = []
        self.oby = []
        self.gtidx =0 # gpp target index
        self.ltidx = 0 # lpp target index
        self.gidx = 0
        self.ob_num = 0

        self.center = []
        self.goal_point = '9999'

        # self.next = False
        self.start_idx = 0
        self.ser = serial.Serial('/dev/ttyUSB0',115200) # USB 권한 주
        self.break_once = False
        self.person_detect = 0
        self.animation_cnt = 0
        self.old_lpath_x = []
        self.old_lpath_y = []
        self.mode_sw = GlobalPath   
        self.LPP_is_done = 0
        self.GPP_is_done = 0
        self.old_obstacle = 0

        # UTurn
        self.mission_mode = "Driving"
        self.first_uturn = True
        self.old_yaw = 0

        # Parking
        self.parking_first = 0
        self.parking_node = 2
        self.base1_x = 22.760400877965
        self.base1_y = 41.7303388307402
        self.base2_x = 17.978170358155626
        self.base2_y = 34.84945192598553

        self.parking_time_count=0

        # Displacement - encoder
        self.msg = Float32()
        self.pub_dis = rospy.Publisher('/Displacement', Float32, queue_size=1)
        self.pre_add = 0 #엔코더에서 사용하는 저장변수
        self.now_add = 0 #엔코더에서 사용하는 저장변수
        self.enc_flag = 0 #엔코더에서 사용하는 flag


        # self.bring_tmp_map()
    
    ## 잡 ## 
    def get_xy(self,  lat,  lon,  alt): # lat,lon ->> x,y
        e, n, u = pymap3d.geodetic2enu(lat, lon, alt, self.control_data['base_lat'],  self.control_data['base_lon'],  self.control_data['base_alt'])
        return e,  n 

    def calc_dis(self, nx, ny): # 두점과 현위치 사이 거리.
        distance = ((nx - self.control_data['cur_x'])**2 +  (ny - self.control_data['cur_y'])**2)**0.5
        return distance


    ## GPP 관련 #######
    def GPP(self): # GPP 경로 생성 함수 
        # [self.path_x],[self.path_y],[self.path_yaw] 얻음.
        self.control_data['target_idx'] = 0
        self.path_x, self.path_y = [], []
        self.first = False

        csv_file=str(os.path.dirname(os.path.abspath(__file__)))+'/kcity_map/kcity_yes.csv' # 예선
        # csv_file=str(os.path.dirname(os.path.abspath(__file__)))+'/kcity_map/kcity_bon.csv' # 본선

        with open(csv_file,'r') as P_path: # 해당 경로의 파일을 책으로 열람
            lines = csv.reader(P_path) # 책을 줄글로 바꿈 (줄마다 list form)
            for line in lines:
                self.path_x.append(float(line[0]))
                self.path_y.append(float(line[1]))



        # self.start_idx = self.select_start_node(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw']) # GPP 시작 노드 설정
        # self.path_x, self.path_y, self.path_yaw = makeTarget.path_connect(str(self.start_idx), '34') # 출발노드, 도착노드

        self.goal_x, self.goal_y = self.path_x[-1], self.path_y[-1]

        #--------------------------------------------------------------------------------
        for i in range(len(self.path_x)):
            gpath=Point32()
            gpath.x=self.path_x[i]
            gpath.y=self.path_y[i]
            self.gpaths.points.append(gpath)
        self.gpaths.header.stamp=rospy.Time.now()
        # print(self.gpaths.points)
        # print('gpaths published.')
        #-------------------------------------------------------------

    def select_start_node(self, cx, cy, cyaw): # GPP 에서 사용
        # 작으면 계속 저장하면서 쭉 훑어 >> nodelist 중 최단거리 노드 번호찾음.
        nodelist = []
        nodelist = makeTarget.node_set()

        min_dis = 99999
        min_idx = 10000
        temp_idx = 10000
        temp_dis = 9999

        for node in nodelist:
            temp_dis = self.calc_dis(nodelist[node].x, nodelist[node].y)
            if temp_dis < min_dis:
                min_dis = temp_dis
                min_idx = node
        return min_idx # 노드번호임.
    ##################
    def calc_cur_idx(self):
        min_dis=1000000
        min_idx=100000
        for idx in range(self.gtidx-100,self.gtidx):
            dis = hypot(self.path_x[idx]-self.control_data['cur_x'] , self.path_y[idx] -self.control_data['cur_y'])
            if dis< min_dis:
                min_dis=dis
                min_idx=idx
        self.cur_idx = min_idx

        print('cur_idx, target_idx ',self.cur_idx,self.gtidx) 

            

        

    ### delta 계산 3set ###

    def cal_steering(self, cur_x, cur_y, cur_yaw, look_ahead): # delta return
        # gpp,lpp 따라서 P.P 에 맞는 "delta" return. 
        # print("cur ::", cur_x, cur_y)
        self.gtidx = self.select_target(cur_x, cur_y, self.gtidx, self.path_x, self.path_y)
        # print('what??', self.gtidx)
        # 세번째 변수 gtidx 부터 탐색한거는 어차피 지난번 계산 한 변수니까 조금이라도 탐색 범위 줄인거.
        if self.mode_sw is GlobalPath:
            target_x = self.path_x[self.gtidx]
            target_y = self.path_y[self.gtidx]
            self.control_data['look_ahead'] = 4
        elif self.mode_sw is LocalPath:
            self.ltidx = self.select_target(cur_x, cur_y, self.ltidx, self.lpath_x, self.lpath_y)
            target_x = self.lpath_x[self.ltidx]
            target_y = self.lpath_y[self.ltidx]
            self.control_data['look_ahead'] =1
            # target_x = self.lpath_x[10]
            # target_y = self.lpath_y[10]
        

        self.calc_cur_idx()


        self.control_data['target_idx'] = self.gtidx

        return self.steering_angle(cur_x, cur_y, cur_yaw, target_x, target_y)

    def select_target(self, cx, cy, cidx, path_x, path_y):
        # 현재 index부터 경로 끝 ind 까지 보면서 ld에 해당하는 target index return.
        valid_idx_list = []

        for i in range(cidx, len(path_x)):
            dis = ((path_x[i]-cx)**2 +(path_y[i]-cy)**2)**0.5

            if dis <= self.control_data['look_ahead']:
                valid_idx_list.append(i)
            if len(valid_idx_list) != 0 and dis > self.control_data['look_ahead']:
                break
        # print("len :", len(valid_idx_list))
        if len(valid_idx_list) == 0:
            return 0
        else:
            return valid_idx_list[len(valid_idx_list) - 1]
     
    def steering_angle(self,  cur_x, cur_y, cur_yaw, target_x, target_y): # cal_steering에서 사용
        # target_x,target_y 를 각각 selct target 으로 부터 index 받아서 계산.해둠. 
        # pure pursuit 계산되는 부분 
        tmp_th = degrees(atan2((target_y - cur_y), (target_x - cur_x)))
        tmp_th = tmp_th%360
        alpha =  cur_yaw - tmp_th

        if abs(alpha)>180:
            if (alpha < 0) :
                alpha += 360
            else :
                alpha -= 360
        alpha = max(alpha,  -90)
        alpha = min(alpha,  90)
        delta = degrees(atan2(2*self.control_data['WB']*sin(radians(alpha))/self.control_data['look_ahead'],1))

        if abs(delta)>180:
            if (delta < 0) :
                delta += 360
            else :
                delta -= 360
        if abs(delta)>30:
            if delta > 0:
                return 1999
            else :
                return -1999
        else :
            delta = 71*delta
            # print('del:',  delta)
            return int(delta)



    ############### LPP ############################################

    def getObstacleMsg(self, msg): # self.lpath_x, self.lpath_y 구해줌
        # 첫번재가 아니고, 주차 모드 아닐 시 => 나중엔 주차 대신 mission_mode가 default 일 시로 바꿔야 할듯(미정)

        if self.first == False and self.mission_mode is not "Parking":
            # lpp 처음이고, parking 표지판으로도 안바뀌었을 때.
            self.ob_num = msg.circle_number
            diff = self.ob_num - self.old_obstacle
            self.old_obstacle = self.ob_num

            self.obx, self.oby = [], []
            for pt in msg.points:
                self.obx.append(pt.x)
                self.oby.append(pt.y)

            if diff > 0: # 장애물 개수 차이 있으면
                if self.LPP_is_done is 0:
                    # self.control_data['target_idx']
                    self.ltidx = 0
                    self.gidx, ox, oy = self.makeObstacleMap(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
                    self.lpath_x, self.lpath_y = path_plan(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'], self.path_x[self.gidx], self.path_y[self.gidx], self.path_yaw[self.gidx], ox, oy, self.animation_cnt)
                    self.mode_sw = LocalPath
                    if len(self.lpath_x) is not 0:
                        self.LPP_is_done = 1

                elif self.mode_sw is LocalPath:
                    get_test = []
                    for i in range(int(self.ob_num)):
                        get_test.append(self.calc_dis(self.obx[i], self.oby[i]))
                    ox, oy = [], []

                    if min(get_test) > 2:
                        self.gidx, ox, oy = self.makeObstacleMap(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
                        self.lpath_x, self.lpath_y = path_plan(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'], self.path_x[self.gidx], self.path_y[self.gidx], self.path_yaw[self.gidx], ox, oy, self.animation_cnt)
                
                
                #------------- # rviz LPP # --------------------------------------------------------------------------------
                for i in range(len(self.lpath_x)):
                    lpath=Point32() # 메시지로 정의.
                    lpath.x=self.lpath_x[i]
                    lpath.y=self.lpath_y[i]
                    self.lpaths.points.append(lpath)
                self.lpaths.header.stamp=rospy.Time.now()
                self.pub_lp.publish(self.lpaths)
                # print('lpaths published.')
                self.lpaths.points = []
                #--------------------------------------------------------------------------------

    def makeObstacleMap(self, c_x, c_y, c_yaw): # hybrid A* : parking 이랑 getoObstacleMsg 에서 maketarget.pathplan과 함께 사용
        # 
        ox, oy = [], [] # lidar, vision에서 받기

        # print('makeObstacleMap')
        # for i in range(100):
        #     ox.append(0.6)
        #     oy.append(i)

        # for i in range(100):
        #     ox.append(0.6)
        #     oy.append(i)

        ox.append(c_x-10)
        oy.append(c_y-1.0)
        ox.append(c_x+10)
        oy.append(c_y-1.0)

        ox.append(c_x-15)
        oy.append(c_y+26.0)
        ox.append(c_x+15)
        oy.append(c_y+26.0)
        

        for i in range(100):
            ox.append(c_x-10 + 0.2* i)
            oy.append(c_y - 1.0)


        for i in range(len(ox)):
            ox[i] -= c_x
            oy[i] -= c_y

        ox_rot, oy_rot = [], []
        rotate_yaw = c_yaw-90 % 360
        for i in range(len(ox)):
            ox_rot.append(ox[i]*cos(np.deg2rad(rotate_yaw)) + oy[i]*-sin(np.deg2rad(rotate_yaw)))
            oy_rot.append(ox[i]*sin(np.deg2rad(rotate_yaw)) + oy[i]*cos(np.deg2rad(rotate_yaw)))

        for i in range(len(ox)):
            ox_rot[i] += c_x
            oy_rot[i] += c_y

        for i in range(len(self.obx)):
            ox_rot.append(self.obx[i])
            oy_rot.append(self.oby[i])

        # print("#gidx: ", self.control_data['target_idx'] + 200, self.select_goal(c_x, c_y, c_yaw, ox, oy))
        gidx = self.control_data['target_idx'] + 150
        # print("##############################################################", gidx)
        return gidx, ox_rot, oy_rot

    ###################################################################





    ####################### Parking ######################
    def getParkingLot(self, msg): # /Parking_num callback
        # print("Parking Mode ON")
        self.mission_mode = "Parking"
        self.parking_node = msg.data # 1,2,3,4,5 중.


    def parking(self): # base point 도달시 parking LPP 경로 생성 하고, parking_node 도달하면 'PCOM'으로 mission_mode 변경
        # print("parking")
        self.mission_mode = 'Parkinging'
        self.gx, self.gy ,num = self.calc_gp() # 주차공간의 좌표,번호
        
        P_path_x, P_path_y = [],[]

        csv_file=str(os.path.dirname(os.path.abspath(__file__)))+'/Parking_songdo/'+str(num) +'.csv' 
        with open(csv_file,'r') as P_path: # 해당 경로의 파일을 책으로 열람
            lines = csv.reader(P_path) # 책을 줄글로 바꿈 (줄마다 list form)
            for line in lines:
                P_path_x.append(float(line[0]))
                P_path_y.append(float(line[1]))
        
        if self.parking_first != 2: # parking 한번 완료하면 무조건 2 , first_base_point 에서 parking_node==0이면(주차공간 없으면)  p.f=1

            self.mode_sw = LocalPath # 바꿔줘야 lpath_x,y 유효해짐.
            self.control_data['target_speed'] = 50
            self.lpath_x, self.lpath_y = P_path_x, P_path_y
            self.parking_first = 2

            
            # return 0

    def calc_gp(self): #################송도 좌표로 CHANGE_##########
        if self.parking_node == 1:
            gx,gy= 17.623907356361915,41.175622253568505
        elif self.parking_node == 2:
            gx,gy= 15.85266480396189,38.844924089730185
        elif self.parking_node == 3:
            gx,gy= 14.16998329088652,36.736197374027405
        elif self.parking_node == 4:
            gx,gy= 12.398738836681156,34.405499957494584
        elif self.parking_node == 5:
            gx,gy= 10.716055698028432,32.18578849457638

        return 14.16998329088652,36.736197374027405 , 3  # 2번 주차칸 중앙점으로 일단 ㄱ 
        # return gx,gy ,self.parking_node  # LiDAR에서 아직 번호 받으면 이걸록 ㄱ. # CHANGE####

    def parking_process(self, break_val, speed, steering, gear):
        

        if self.mission_mode == "P_ready":  # P_ready는 멈춰서 주차공간 확인하는 모드.
            break_val = 200
            speed = 0x00


        # 주차 완료_break 조져 >> serial 주고 parking_next_process 에서 2초 쉬고 Backward 로 바꿔줌.
        if self.mission_mode == "PCOM": 
            break_val = 200
            speed = 0x00

        # 후진 > 이 serial줄건데 주고 또 6초 동안 뒤로 감( 송도에선 좀 줄이자_ 중앙선 조심)
        if self.mission_mode == "Backward":
            speed = 0x50
            steering = 400
            gear = 0x02
            break_val = 0
            print('제발제발제발제발')
            
        # 후진 완료 _ 다시 break 조져 (gear 바꾸려면 멈춰야함)
        if self.mission_mode == "BCOM":
            break_val = 200
            speed = 0x00

        # 전진
        if self.mission_mode == "Driving":
            break_val = 0
            speed = 0x30
            steering = 0
            gear = 0x00

            print("GPP start")
            self.mission_mode = ""
            self.GPP() # 어차피 다시 실행
            self.mode_sw = GlobalPath
            #self.GPP_is_done = 1
            self.LPP_is_done = 1 ###### ???? 언제 1 되는거?? LPP 가 끝날때가 맞아?? LPP 경로 받을때가 아니라? 
        
        return break_val,speed,steering,gear 



    def parking_next_process(self): # parking_process > serial 보내 > parking_next_process
        # parking_process 에서 미션 완료 후 mission_mode 바뀌었으면 , serial 한번 명령 쏘자마자
        # 바로 time.sleep(*) 때려버림. >> 그 전 명령이 계속 유지 > 단순 명령이라 가능.

        if self.mission_mode == "P_ready":
            print("P_ready")
            time.sleep(4)
            break_val = 0
       
        if self.mission_mode == "BCOM":
            time.sleep(2)
            self.mission_mode = "Driving"

        if self.mission_mode == "Backward":
            print('@@@@@@@@@백월드@@@@@@@@@')
            parking_time_end=time.time() + 6.5
            while time.time() < parking_time_end:
                
                cnt=0x00
                result = self.ser.readline() 
                self.ser.flushInput()
                # print(result)
                # print(result[0])
                if (result[0] is 0x53 and result[1] is 0x54 and result[2] is 0x58):
                    res_arr = []
                    res_idx = 0
                    # print('okokok')

                    while True:
                        for i in range(len(result)):
                            if result[i] is 0x0A and i is not 17:
                                # print("### 0x0A Found!", i, "th data")
                                res_arr.append(0x0B)
                            else :
                                res_arr.append(result[i])

                        if len(res_arr) < 18:
                            result = self.ser.readline()
                        else:
                            break
                    
                    #################SHSHSSH############################3

                    # print('shsh_raw', res_arr[6], res_arr[7],res_arr[8], res_arr[9])
                    # print('shsh_modify', int(res_arr[6]))
                    ###############################################

                    
                    cnt = res_arr[15]

                result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x02, 80,
                    1200, 0, cnt, 0x0D, 0x0A)    # big endian 방식으로 타입에 맞춰서 pack  
                self.ser.write(result)
            self.mission_mode = "BCOM"

        if self.mission_mode == "PCOM":
            print("PCOMing")
            time.sleep(2)
            self.mission_mode = "Backward"



            



    def getOdoMsg(self,  msg):
       
        self.control_data['cur_yaw']  = msg.twist.twist.angular.z #imu 정북 기준으로 시계로 돌아갈때(시뮬)
        self.control_data['cur_x'], self.control_data['cur_y']  = msg.pose.pose.position.x, msg.pose.pose.position.y 

        ## rviz_cur_* #------------------------------------------------------
        self.cur_points = PointCloud()
        self.cur_points.header.frame_id = 'world'
        cur_point = Point32()
        cur_point.x = self.control_data['cur_x']
        cur_point.y = self.control_data['cur_y']
        self.cur_points.points.append(cur_point)
        self.cur_points.header.stamp = rospy.Time.now()
        self.pub_c.publish(self.cur_points)
        #--------------------------------------------------------
        # UTURN #
        #--------------------------------------------------------
        # if abs(self.control_data['cur_x'] - 17.3) < 0.8 and abs(self.control_data['cur_y'] - 36.4) < 0.8 and self.first_uturn == True and self.mission_mode is not "UTURN":
        #     print("UTURN ON")
        #     self.mission_mode = "UTURN"
        #     self.first_uturn = False
        #     self.old_yaw = self.control_data['cur_yaw']
        # #--------------------------------------------------------

        if self.first == False: 
            if self.GPP_is_done is 0: # GPP 가 경로 아직 안받은거 (맨 처음)
                print("GPP start")
                self.GPP()
                self.mode_sw = GlobalPath
                self.GPP_is_done = 1 # GPP 로 전역경로 받아왔다.
            self.pub_gp.publish(self.gpaths)

            ################## 이거 없어도 되지 않나??@##########################

            # LPP 골포인트 근처도달시 GPP 전환. 
            if self.calc_dis(self.path_x[self.gidx], self.path_y[self.gidx]) < 4 and self.mode_sw is LocalPath:
                self.mode_sw = GlobalPath
                self.LPP_is_done = 0 # LPP switch off
                self.old_obstacle = 0
            # print("first_check", self.control_data['first_check'])

            #################################################################



            # GPP or LPP 진행중이면 ,target speed
            if (self.mode_sw is GlobalPath and self.GPP_is_done is 1) or (self.mode_sw is LocalPath and self.LPP_is_done is 1):
                # print("ls", self.ls, "gs", self.gs)
                # if len(self.lpath_x) is not 0:
                if self.mode_sw is GlobalPath:
                    # print("GlobalPath")
                    self.control_data['target_speed'] = 200
                elif self.mode_sw is LocalPath:
                    # print("LocalPath")
                    self.control_data['target_speed'] = 50                     

            

            
            #---------------------- Parking in getOdoMsg. ------------------------# 
            # print(self.mission_mode)


            if self.mission_mode == 'P_ready':
                if self.parking_node != 0:
                    self.parking() # 보내버리면 신경쓸거 없음.
                else:
                    self.parking_first = 1 # 2nd 로 갈거에요.
                    self.mission_mode = 'Parking' # 움직이는 주차대기상태에요. only.

            if self.mission_mode == 'Parkinging':
                if self.calc_dis(self.gx, self.gy) < 1:
                    self.mission_mode = "PCOM"

            # if self.parking_first ==0 and self.calc_dis(self.base1_x, self.base1_y) < 2:
            #     self.mission_mode = "P_ready"
            
            # if self.parking_first ==1 and self.calc_dis(self.base2_x, self.base2_y) < 1:
            #     self.mission_mode = "P_ready" 
           
            # = base point 도달시 parking LPP 경로 생성 하고, parking_node 도달하면 'PCOM'으로 mission_mode 변경

            # parking 중인데, yaw 가 저 범위 안쪽이면 직진, 아니면 CHANGE############

            if self.mission_mode == "Parkinging" and self.control_data['cur_yaw'] < 170 and self.control_data['cur_yaw'] > 164:
                self.control_data['steering'] = 0
            else: # 그게 아닌 일반적인 모든 경우가 여기. (p.p)
                self.control_data['steering']  = self.cal_steering(self.control_data['cur_x'], self.control_data['cur_y'],  self.control_data['cur_yaw'],  self.control_data['look_ahead'])




            # Serial
            #----------------------------------------------        
            cnt=0x00
            result = self.ser.readline() 
            self.ser.flushInput()
            # print(result)
            # print(result[0])
            if (result[0] is 0x53 and result[1] is 0x54 and result[2] is 0x58):
                res_arr = []
                res_idx = 0
                # print('okokok')

                while True:
                    for i in range(len(result)):
                        if result[i] is 0x0A and i is not 17:
                            # print("### 0x0A Found!", i, "th data")
                            res_arr.append(0x0B)
                        else :
                            res_arr.append(result[i])

                    if len(res_arr) < 18:
                        result = self.ser.readline()
                    else:
                        break
                
                #################SHSHSSH############################3

                # print('shsh_raw', res_arr[6], res_arr[7],res_arr[8], res_arr[9])
                # print('shsh_modify', int(res_arr[6]))
                ###############################################

                
                cnt = res_arr[15]
                # print("res_arr is", res_arr[6])
                # print(self.mission_mode)
                self.serWrite(int(self.control_data['target_speed']), int(self.control_data['steering']), cnt)
               
                a = res_arr[11]
                b = res_arr[12]
                c = res_arr[13]
                d = res_arr[14]

                # print("I'm here" , a)
                add = (float(a + 256*b + 256**2*c + 256**3*d))
                # if self.enc_flag == 0:
                #     self.pre_add = add
                #     self.now_add = add
                #     self.enc_flag = 1
                # else:
                #     self.pre_add = self.now_add
                #     self.now_add = add

                # difference = self.now_add - self.pre_add
                
            

                self.msg = add

                # if difference > 500:
                #     self.msg = self.pre_add


                self.pub_dis.publish(self.msg)




    def serWrite(self, speed, steering, cnt):
        input_speed = speed
        break_val = 0x00
        gear = 0x00
        # steering 값 2000 넘길 시 2000으로 설정
        if abs(steering)>=2000:
            if steering>0:
                steering = 1999
            else :
                steering =-1999
        # if abs(steering) < 300 :
        #     steering = int(0.5*steering)

        # 기어 기본값 0: 전진, 1:중립 2: 후진 인듯 ?? 
        goal_dis = hypot(self.goal_x - self.control_data['cur_x'], self.goal_y - self.control_data['cur_y']) # 하이팟 대신에 x, y 따로 비교를 할까...

        if goal_dis > 3:
            speed = speed
        else:
            break_val = 0x0B #
            speed = 0x00
            # self.next = 
        get_test = []
        for i in range(int(self.ob_num)):
            get_test.append(self.calc_dis(self.obx[i], self.oby[i]))

        if self.person_detect == 1:
            print("emergency stop")
            break_val = 200

        if self.mode_sw == LocalPath:
            print("LPP ing")
        
        break_val,speed,steering,gear = self.parking_process(break_val, speed, steering, gear)

        # print(gear,speed,break_val)

        result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, gear, int(speed),
                    steering, break_val, cnt, 0x0D, 0x0A)    # big endian 방식으로 타입에 맞춰서 pack  

        self.ser.write(result)

        # if self.mission_mode == "UTing":
        #     print("###ing")
        #     time.sleep(3)
        #     self.mission_mode = ""
        #     self.first_uturn = False

        self.parking_next_process()


        # BASE POINT 지나는 순간   self.mission_mode= 'Parking' (  그냥 speed 2 로 서행하면서 parking() 대기상태. )



        # if self.mission_mode == "Driving":
        #     time.sleep(1)
        # print(result)

    #----------------------------------------------



    # def vision(self,msg):
    #     self.person_detect = 0
    #     for os in msg.bounding_boxes:
    #         if(os.Class == "PERSON" and self.ob_num > 0):
    #             self.person_detect = 1
    #             # self.control_data['target_speed'] = 0x00
    #             print("person is detected")



    ############### Uturn ########################

    # def uTurn(self, msg):
    #     if msg.data == "UTURN" and self.first_uturn is True:
    #         self.mission_mode = "UTURN"
    #         self.first_uturn = False
    #         self.old_yaw = self.control_data['cur_yaw']


    # def Uturn_process(self, steering):
        
    #     # 유턴
    #     if self.mission_mode == "UTURN":
    #         print("###Uturn ON")
    #         if self.first_uturn is False:
    #             print("okok", self.control_data['cur_yaw'], self.old_yaw)
    #             steering = -1999
    #             self.mission_mode = "UTing"

    #         else:
    #             pass

    #############################################





    def run(self):
        print("Controller ON")
        self.first = False
        
        # rospy.Subscriber('/uturn', uuu, self.uTurn)
        # rospy.Subscriber('/Parking_num', Int32, self.getParkingLot) # LiDAR 주차공간
        # rospy.Subscriber('/pangpang', PangPang, self.getObstacleMsg) # LiDAR 장애물 정보
        rospy.Subscriber("/pose", Odometry, self.getOdoMsg) # GPS/IMU
        # rospy.Subscriber('/darknet_ros/bounding_boxes_c',BoundingBoxes, self.vision) # Vision


