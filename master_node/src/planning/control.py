import rospy
from math import sqrt

from master_node.msg import Path, Serial_Info, Planning_Info, Local  # 개발할 메세지 타입
from geometry_msgs.msg import Point32
from lib.control_utils.general import General
from lib.control_utils.avoidance import Avoidance
from nav_msgs.msg import Odometry
# from lib.control_utils.emergency_stop import EmergencyStop
from lib.control_utils.normal_stop import NormalStop
from lib.control_utils.parking import Parking
from lib.control_utils.stack import ParkingStack
from lib.control_utils.delivery_control import deliveryControlClass

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


LEFT_MAX_STEER = 10
RIGHT_MAX_STEER = -10
BGEAR = 0x02
NGEAR = 0x01
FGEAR = 0x00
MAX_BRAKE = 200

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
        self.local_path=Path()
        self.lookahead = 4
        self.past_mode = None
        self.parking_target = 0
        general = General(self)
        avoidance = Avoidance(self)
        self.parking_stack = ParkingStack()
        parkingClass = Parking(self)

        # emergency_stop = EmergencyStop(self)
        normal_stop = NormalStop(self)
        self.is_planning = False
        self.start_time = time.time()
        self.current_time = time.time()
        self.time_count = 0

        rate = rospy.Rate(50)  # 100hz

        # main loop
        while not rospy.is_shutdown():
            if self.is_planning:
                print(self.planning_info.mode)
                if self.planning_info.mode == "general":
                    if self.planning_info.path.x:
                        self.global_path.x = self.planning_info.path.x
                        self.global_path.y = self.planning_info.path.y
                        self.global_path.k = self.planning_info.path.k
                        self.global_path.heading=self.planning_info.path.heading
                        self.global_path.env = self.planning_info.path.env
                        self.global_path.mission = self.planning_info.path.mission

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
                    if self.planning_info.path.x:
                        self.local_path.x = self.planning_info.path.x
                        self.local_path.y = self.planning_info.path.y
                        self.local_path.k = self.planning_info.path.k
                        self.local_path.env = self.planning_info.path.env
                        self.local_path.mission = self.planning_info.path.mission
                    if self.local_path.x:
                        self.pub_msg=avoidance.driving(self)
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

                # base에서 정지
                elif self.planning_info.mode == "parking-base1" or self.planning_info.mode == "parking-base2":
                    self.serialParkingComm(0x00, MAX_BRAKE, FGEAR)

                # base2로 이동
                elif self.planning_info.mode == "parking2":
                    self.serialParkingComm(10, 0x00, FGEAR)
                
                # 라이다에 쏴줄때 정지 - 없어도 될 수도
                elif self.planning_info.mode == "parking_ready":
                    self.serialParkingComm(0x00, MAX_BRAKE, FGEAR)

                # 주행
                elif self.planning_info.mode == "parking_start":

                    self.serialParkingComm(10, 0x00, FGEAR)
                    self.pub_msg.steer = parkingClass.pure_pursuit(self.planning_info.point, self)
                    # 정해진 노드 따라서 주행
                    # self.parking_stack.push(10, 0x00, self.pub_msg.steer)

                # 전진 주차 끝 정지, 후진 기어
                elif self.planning_info.mode == "parking_complete":
                    self.serialParkingComm(0x00, MAX_BRAKE, BGEAR)
                    # self.parking_stack.push(10, 0x00, 0x00)

                # 후진
                elif self.planning_info.mode == "parking_backward":
                    # _, _, self.pub_msg.steer= self.parking_stack.pop()
                    self.serialParkingComm(10, 0x00, BGEAR)
                    self.pub_msg.steer = parkingClass.pure_pursuit(self.planning_info.point, self)
                    self.pub_msg.steer=-self.pub_msg.steer


                elif self.planning_info.mode == "backward_complete":
                    self.serialParkingComm(0x00, MAX_BRAKE, FGEAR)

                elif self.planning_info.mode == "parking_end":
                    self.serialParkingComm(0x00, MAX_BRAKE, FGEAR)
                    #self.planning_info.mode = "general"

                elif self.planning_info.mode == "delivery_a":
                    self.pub_msg.steer = avoidance.pure_pursuit(self.PickUpPath)

                elif self.planning_info.mode == "pickup_stop":
                    self.serialParkingComm(0, MAX_BRAKE, FGEAR)

                elif self.planning_info.mode == "delivery_b":
                    self.pub_msg.steer = avoidance.pure_pursuit(self.deliverPath)

                elif self.planning_info.mode == "delivery_stop":
                    self.serialParkingComm(0, MAX_BRAKE, FGEAR)

                self.past_mode = self.planning_info.mode
                control_pub.publish(self.pub_msg)
                rate.sleep()

    def serialParkingComm(self, speed, brake, gear):
        self.pub_msg.speed = speed
        self.pub_msg.brake = brake
        self.pub_msg.gear = gear

    # Callback Function
    def planningCallback(self, msg):
        self.planning_info=msg
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

    def parkingCallback(self, msg):
        self.parking_target = msg.data

print("Control start")
control = Control()
