import rospy
import time

from master_node.msg import Path, Serial_Info, Planning_Info, Local  # 개발할 메세지 타입
from lib.control_utils.general import General
from lib.control_utils.avoidance import Avoidance
# from lib.control_utils.emergency_stop import EmergencyStop
from lib.control_utils.normal_stop import NormalStop

from lib.control_utils.uturn import Uturn

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

class Control:
    def __init__(self):
        rospy.init_node("Control", anonymous=False)

        control_pub = rospy.Publisher("/control", Serial_Info, queue_size=1)
        # path_pub = rospy.Publisher("/path", )
        self.pub_msg = Serial_Info()

        rospy.Subscriber("/serial", Serial_Info, self.serialCallback) # 여기서 지금 받은거._ 현재  SERIAL 상태.
        rospy.Subscriber("/planner", Planning_Info, self.planningCallback)
        self.planning_info = Planning_Info()
        self.serial_info = Serial_Info() # 위에서 받았는데 얘가 계속 초기화 되는거 아니가?? @@@@@@@@
        self.local = Local()
        self.global_path = Path()
        self.lookahead = 4
        self.past_mode = None

        self.general = General(self)
        self.avoidance = Avoidance(self)
        # emergency_stop = EmergencyStop(self)
        self.normal_stop = NormalStop(self)
        self.uturn = Uturn()
        self.is_planning = False

        rate = rospy.Rate(50)  # 100hz

        # main loop
        while not rospy.is_shutdown():
            # print(self.global_path)
            # rospy.Subscriber("/serial", Serial_Info, self.serialCallback) # 여기서 지금 받은거._ 현재  SERIAL 상태.
            # rospy.Subscriber("/planner", Planning_Info, self.planningCallback)





            if self.is_planning:
                if self.planning_info.mode == "general":
                    if self.planning_info.path_x:
                        self.global_path.x = self.planning_info.path_x
                        self.global_path.y = self.planning_info.path_y
                        self.global_path.heading = self.planning_info.path_heading
                        self.global_path.k = self.planning_info.path_k
                        print(self.global_path.x)

                    if self.global_path.x:
                        # print(1)
                        self.pub_msg = self.general.driving()
                        print("### pub", self.pub_msg)

            # endtime엔 한번만 넣게
            endtime = time.time() + 3
            if self.planning_info.mode == "general":

                # steering value 를 serial로 넣어주기
                self.pub_msg.steer = LEFT_MAX_STEER

                # 3초간 유지
                curtime = time.time()
                if curtime < endtime:
                    pass
                else:
                    self.planning_info_mode = "general"

                    # 새로운 점 잡기
                    new_idx = self.uturn.select_new_target(self.local, self.global_path)

                    # GPP 경로 재 생성 (new idx 경유)

                    





                # elif self.planning_info.mode == "avoidance":
                #     self.pub_msg.steer = avoidance.pure_puresuit()
                #     self.pub_msg.speed = 10
                #     self.pub_msg.brake = 0
                #     self.pub_msg.encoder = 0
                #     self.pub_msg.gear = 0
                #     self.pub_msg.emergency_stop = 0
                #     self.pub_msg.auto_manual = 1

                # elif self.planning_info.mode == "emergency_stop":
                #     self.pub_msg.steer = 0
                #     self.pub_msg.speed = 0
                #     self.pub_msg.brake = 0
                #     self.pub_msg.encoder = 0
                #     self.pub_msg.gear = 0
                #     self.pub_msg.emergency_stop = 1
                #     self.pub_msg.auto_manual = 1

                # elif self.planning_info.mode == "normal_stop":
                #     self.normal_stop.run()

                # elif self.planning_info.mode == "parking":

                self.past_mode = self.planning_info.mode
                control_pub.publish(self.pub_msg)
                rate.sleep()

    # Callback Function
    def planningCallback(self, msg):
        self.planning_info = msg
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


    # def missionCallback(self, msg):
    #     self.


control = Control()
