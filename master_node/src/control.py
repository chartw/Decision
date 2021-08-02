import rospy
from math import sqrt

from master_node.msg import Path, Serial_Info, Planning_Info, Local  # 개발할 메세지 타입
from geometry_msgs.msg import Point32
from lib.control_utils.general import General
from lib.control_utils.avoidance import Avoidance
# from lib.control_utils.emergency_stop import EmergencyStop
from lib.control_utils.normal_stop import NormalStop

import time

"""
Serial_Info
{
    auto or man atuo                
    e-stop 0
    gear    0
    speed   pid                     speed--
    steer   global pure-pursuit     target point pure-pursuit
    brake   0
    enc
}
Planning_Info
{
    String mode
    Local local
    Path path
    Point32 point
}
"""


class Control:
    def __init__(self):
        rospy.init_node("Control", anonymous=False)

        control_pub = rospy.Publisher("/control", Serial_Info, queue_size=1)
        self.pub_msg = Serial_Info()

        rospy.Subscriber("/serial", Serial_Info, self.serialCallback) # 여기서 지금 받은거._ 현재  SERIAL 상태.
        rospy.Subscriber("/planner", Planning_Info, self.planningCallback)
        self.planning_info = Planning_Info()
        self.serial_info = Serial_Info() # 위에서 받았는데 얘가 계속 초기화 되는거 아니가?? @@@@@@@@
        self.local = Local()
        self.global_path = Path()
        self.local_point=Point32()
        self.lookahead = 4
        self.past_mode = None

        general = General(self)
        avoidance = Avoidance(self)
        # emergency_stop = EmergencyStop(self)
        normal_stop = NormalStop(self)
        self.is_planning = False
        self.start_time = time.time()
        self.current_time = time.time()

        rate = rospy.Rate(50)  # 100hz

        # main loop
        while not rospy.is_shutdown():
            if self.is_planning:
                if self.planning_info.mode == "general":
                    if self.planning_info.path.x:
                        self.global_path = self.planning_info.path
                    if self.global_path.x:
                        self.pub_msg = general.driving(self)
                elif self.planning_info.mode == 'emergency_stop':                                    
                    self.pub_msg.steer = 0
                    self.pub_msg.speed = 0
                    self.pub_msg.brake = 0
                    self.pub_msg.encoder = 0
                    self.pub_msg.gear = 0
                    self.pub_msg.emergency_stop = 1
                    self.pub_msg.auto_manual = 1

                elif self.planning_info.mode == "small" or self.planning_info.mode == "big":
                    if self.local_point.x!=0 and self.local_point.y!=0:
                        self.pub_msg=avoidance.driving(self.local_point)
                    else:
                        self.pub_msg=general.driving(self)
                
                elif self.planning_info.mode=="kid":
                    self.pub_msg=general.driving(self)
                    
                elif self.planning_info.mode=="bump":
                    self.pub_msg=general.driving(self)
                
                elif self.planning_info.mode=="normal_stop":
                    self.pub_msg=general.driving(self)

                    dist=self.planning_info.dist -0.9 # 범퍼위치로 기준 재설정\
                    self.pub_msg.speed=dist/5
                    t=dist/((self.serial_info.speed/3.6)+0.1)
                    # self.pub_msg.brake=int(200/t) # 유리 함수 200/x
                    # self.pub_msg.brake=int((200/t)-20) # 유리 함수 200/x - 20
                    # self.pub_msg.brake=int((300/t)-60) # 유리 함수 300/x - 60
                    t=max(1,dist/((self.serial_info.speed/3.6)+0.1))
                    # self.pub_msg.brake=int(-100*sqrt(t-1)+200) # 제곱근 함수
                    self.pub_msg.brake=int(-75*sqrt(t-1)+150) # 제곱근 함수
                    self.pub_msg.brake=max(0,min(200,self.pub_msg.brake))

                    
                # elif self.planning_info.mode == 'normal_stop':
                #     is_first = (self.past_mode != 'normal_stop')
                #     self.normal_stop.run(is_first)

                # print(self.planning_info.dist)
                # if not self.planning_info.mode=="avoidance" and self.planning_info.dist!=-1:
                #     dist=self.planning_info.dist -1.05 # 범퍼위치로 기준 재설정

                #     self.pub_msg.speed=dist/5
                #     t=dist/((self.serial_info.speed/3.6)+0.1)
                #     # self.pub_msg.brake=int(200/t) # 유리 함수 200/x
                #     # self.pub_msg.brake=int((200/t)-20) # 유리 함수 200/x - 20
                #     # self.pub_msg.brake=int((300/t)-60) # 유리 함수 300/x - 60
                #     t=max(1,dist/((self.serial_info.speed/3.6)+0.1))
                #     self.pub_msg.brake=int(-100*sqrt(t-1)+200) # 제곱근 함수
                #     # self.pub_msg.brake=int(-75*sqrt(t-1)+150) # 제곱근 함수
                #     self.pub_msg.brake=max(0,min(200,self.pub_msg.brake))
                    

                self.past_mode = self.planning_info.mode
                control_pub.publish(self.pub_msg)
                rate.sleep()

    # Callback Function
    def planningCallback(self, msg):
        self.planning_info=msg
        self.global_path.x = msg.path.x
        self.global_path.y = msg.path.y
        self.global_path.heading = msg.path.heading
        self.global_path.k = msg.path.k
        self.local_point=msg.point
        self.local.x = msg.local.x
        self.local.y = msg.local.y
        self.local.heading = msg.local.heading

        # print(self.planning_info)
        self.is_planning = True

    def serialCallback(self, msg):
        
        self.serial_info.encoder = msg.encoder
        self.serial_info.auto_manual = msg.auto_manual
        self.serial_info.gear = msg.gear
        self.serial_info.steer = msg.steer
        self.serial_info.speed = msg.speed
        self.serial_info.emergency_stop = msg.emergency_stop
        self.serial_info.brake = msg.brake


        # print(self.serial_info) # 얜 잘 받음 / 근데  general 에서 못받아.ㅇㄹ이러이라ㅓㅁ댜ㅐ렁마러ㅑㅐㄷ머랑ㅁ르


control = Control()