#!/usr/bin/env python
#-*- coding:utf-8 -*-
from master_node.msg import Serial_Info  # 개발할 메세지 타입

from math import degrees, atan2, sin,cos, radians, sqrt
import time
import mpc_supporter as sfc


class General_MPC:
    def __init__(self, control):
        # 깊은 복사 수행
        self.cur = control.local # local 좌표
        self.path = control.global_path # global_path
        self.GeneralLookahead = control.lookahead #직진 주행시 lookahead
        # self.serial_info = Serial_Info()
        self.serial_info = control.serial_info
        self.past_mode = control.past_mode

        self.lookahead = 4
        self.WB = 1.04
        self.target_index = 0

        self.t_start = 0
        self.t_delta = 0
        self.t_old = 0
        self.t_new = 0
        self.t = 0


        ##MPC변수
        # Create an object for the self.support functions.

        #전체 path
        self.path.x
        self.path.y
        #10개만큼 가져온 path
        self.path.y[self.cur_idx:self.cur_idx+10]
        self.path.heading[self.cur_idx:self.cur_idx+10]

        self.support=sfc.SupportFilesCar()
        self.constants=self.support.constants

        # Load the constant values needed in the main file
        self.Ts=self.constants[6]
        self.outputs=self.constants[10] # number of outputs (self.psi, Y)
        self.hz = self.constants[11] # horizon prediction period
        self.x_dot=self.constants[12] # constant longitudinal velocity
        self.time_length=self.constants[15] # duration of the manoeuvre

        # Generate the refence signals
        self.t=np.arange(0,self.time_length+Ts,self.Ts) # time from 0 to 10 seconds, sample time (self.Ts=0.1 second)
        self.r=self.constants[13]
        self.f=self.constants[14]
        self.psi_ref,self.X_ref,self.Y_ref=self.support.trajectory_generator(self.t,self.r,self.f)
        self.sim_length=len(self.t) # Number of control loop iterations
        
        self.k = 0

        # self.refSignals=np.zeros(len(self.X_ref)*self.outputs)  굴비 잘가 
        # Build up the reference signal vector:
        # refSignal = [psi_ref_0, Y_ref_0, psi_ref_1, Y_ref_1, psi_ref_2, Y_ref_2, ... etc.]

        # -굴비- # 
        # for i in range(0,len(self.refSignals),self.outputs):
        #     self.refSignals[i]=self.psi_ref[self.k]
        #     self.refSignals[i+1]=self.Y_ref[self.k]
        #     self.k=self.k+1


        # Load the initial self.states
        self.y_dot=0
        self.psi=0
        self.psi_dot=0
        self.Y=self.Y_ref[0]+10 #변경

        self.states=np.array([self.y_dot,self.psi,self.psi_dot,self.Y])
        self.statesTotal=np.zeros((len(self.t),len(self.states))) # It will keep track of all your self.states during the entire manoeuvre
        self.statesTotal[0][0:len(self.states)]=self.states
        self.psi_opt_total=np.zeros((len(self.t),self.hz))
        self.Y_opt_total=np.zeros((len(self.t),self.hz))

        # Load the initial input
        self.U1=0 # Input at self.t = -1 s (steering wheel angle in rad (delta))
        self.UTotal=np.zeros(len(self.t)) # To keep track all your inputs over time
        self.UTotal[0]=self.U1

        # # To extract psi_opt from predicted x_aug_opt
        # C_psi_opt=np.zeros((self.hz,(len(self.states)+np.size(self.U1))*self.hz))
        # for i in range(1,self.hz+1):
        #     C_psi_opt[i-1][i+4*(i-1)]=1
        #
        # # To extract Y_opt from predicted x_aug_opt
        # C_Y_opt=np.zeros((self.hz,(len(self.states)+np.size(self.U1))*self.hz))
        # for i in range(3,self.hz+3):
        #     C_Y_opt[i-3][i+4*(i-3)]=1

        # Generate the discrete state space matrices
        self.Ad,self.Bd,self.Cd,self.Dd=self.support.state_space()

        self.cur_idx = 0






    def steering_mpc(self):
        '''##########함수안에 넣을꺼##########'''
        # Initiate the controller - simulation loops
        self.cur_idx = self.select_target_mindis(self.cur.x, self.cur.y, self.cur_idx)  # 차 위치 기준 가장 가까운 node index
        self.path_heading = self.path.heading[self.cur_idx] # 현재지점과 다음지점 사이의 각도

        # Generate the augmented current state and the reference vector
        self.x_aug_t=np.transpose([np.concatenate((self.states,[self.U1]),axis=0)])

        #추가
        self.x_aug_t[1] -= self.path_haeding # 상대 좌표로 변환
        self.x_aug_t[3] -= self.cur.y # 여기 나가서 x_aug 는 다시 절대좌표로 바껴야 하는데 어떤식으로 loop 해야할지 모르겟음 함수 나갈때 다시 더해줘야하나

        # From the self.refSignals vector, only extract the reference values from your [current sample (NOW) + Ts] to [NOW+horizon period (self.hz)]
        # Example: t_now is 3 seconds, self.hz = 15 samples, so from self.refSignals vectors, you move the elements to vector self.r:
        # self.r=[psi_ref_3.1, Y_ref_3.1, psi_ref_3.2, Y_ref_3.2, ... , psi_ref_4.5, Y_ref_4.5]
        # With each loop, it all shifts by 0.1 second because Ts=0.1 s

        self.coordinate()

        # Generate the compact simplification matrices for the cost function
        Hdb,Fdbt,Cdb,Adc=self.support.mpc_simplification(self.Ad,self.Bd,self.Cd,self.Dd,self.hz)
        ft=np.matmul(np.concatenate((np.transpose(self.x_aug_t)[0][0:len(self.x_aug_t)],self.r),axis=0),Fdbt)
        du=-np.matmul(np.linalg.inv(Hdb),np.transpose([ft]))



        # x_aug_opt=np.matmul(Cdb,du)+np.matmul(Adc,self.x_aug_t)
        # psi_opt=np.matmul(C_psi_opt[0:self.hz,0:(len(self.states)+np.size(self.U1))*self.hz],x_aug_opt)
        # Y_opt=np.matmul(C_Y_opt[0:self.hz,0:(len(self.states)+np.size(self.U1))*self.hz],x_aug_opt)
        # # if self.hz<4:
        # #     print(x_aug_opt)
        # psi_opt=np.transpose((psi_opt))[0]
        # self.psi_opt_total[i+1][0:self.hz]=psi_opt
        # Y_opt=np.transpose((Y_opt))[0]
        # Y_opt_total[i+1][0:self.hz]=Y_opt
        # exit()

        # Update the real inputs
        
        steering=radians(self.serial_info.steering)+du[0][0]

        # Establish the limits for the real inputs (max: pi/6 radians)

        if steering < -np.pi/6:
            steering=-np.pi/6
        elif steering > np.pi/6:
            steering=np.pi/6

        #steering, states, 서포터 등등 수정 해야함
        # Keep track of your inputs as you go from t=0 --> t=7 seconds
        UTotal[i+1]=U1

        # Compute new states in the open loop system (interval: Ts/30)
        states=self.support.open_loop_new_states(states,U1)


        '''  r. 
        if   self.cur_idx+self.outputs*self.hz<=len(self.refSignals):
            self.r=self.refSignals[  self.cur_idx: self.cur_idx+self.outputs*self.hz]  # 여기에 현재 위치 빼서 넣어야 할듯
        else:
            self.r=self.refSignals[  self.cur_idx:len(self.refSignals)]
            '''



        return degrees(steering)




    # 추가
    def select_target_mindis(self, cx, cy, cidx):  # lookahead 범위 안에서 가장 근접한 index 찾기

        min = 2147000000
        min_idx = 0
        pre_dis = 2147000000

        for i in range(cidx, len(self.path.x)):
            dis = ((self.path.x[i] - cx) ** 2 + (self.path.y[i] - cy) ** 2) ** 0.5
            # if dis <= self.control_data['look_ahead']:
            #     valid_idx_list.append(i)
            if dis <= min:
                min_idx = i
            if pre_dis - dis < 0 and dis > 5:
                return min_idx

            pre_dis = dis

    def coordinate(self): #  self.r 받아서 self.path.heading[self.cur_idx] 으로 돌릴거야 (x,y,yaw)를.
        for i in range(self.hz):
            self.r[i] = self.path.heading[i+self.cur_idx] - self.path_heading
            X = self.path_heading*(self.path.x[i+self.cur_idx]-self.cur.x)
            Y = self.path_heading*(self.path.y[i+self.cur_idx]-self.cur.y)
            self.r[i+1] = -sin(X) + cos(Y)


###################조향 속도 구분선###################

    def driving(self):
        temp_msg=Serial_Info()
        temp_msg.steer = self.steering_mpc()
        temp_msg.speed = 10
        temp_msg.brake = 0
        temp_msg.encoder = 0
        temp_msg.gear = 0
        temp_msg.emergency_stop = 0
        temp_msg.auto_manual = 1

        return temp_msg