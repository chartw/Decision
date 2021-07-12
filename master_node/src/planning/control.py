import rospy
import time

from master_node.msg import Path, Serial_Info, Planning_Info, Local  # 개발할 메세지 타입
from geometry_msgs.msg import Point32
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
        self.control_msg = Serial_Info()

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
        avoidance = Avoidance()
        # emergency_stop = EmergencyStop(self)
        normal_stop = NormalStop(self)
        self.uturn = Uturn()
        self.is_planning = False

        rate = rospy.Rate(50)  # 100hz

        # main loop
        while not rospy.is_shutdown():
            if self.is_planning:              
                if self.planning_info.mode == "general":
                    if self.planning_info.path_x:
                        self.global_path.x = self.planning_info.path_x
                        self.global_path.y = self.planning_info.path_y
                        self.global_path.heading = self.planning_info.path_heading
                        self.global_path.k = self.planning_info.path_k
                    if self.global_path.x:
                        self.control_msg = general.driving(self)

                elif self.planning_info.mode == "avoidance":
                    if self.local_point.x!=0 and self.local_point.y!=0:
                        self.control_msg=avoidance.driving(self.local_point)
                    else:
                        self.control_msg=general.driving(self)
                #     self.control_msg.encoder = 0
                #     self.control_msg.gear = 0
                #     self.control_msg.emergency_stop = 0
                #     self.control_msg.auto_manual = 1
                #     self.control_msg.steer = avoidance.pure_puresuit()
                #     self.control_msg.speed = 10
                #     self.control_msg.brake = 0

                # elif self.planning_info.mode == "emergency_stop":
                #     self.control_msg.steer = 0
                #     self.control_msg.speed = 0
                #     self.control_msg.brake = 0
                #     self.control_msg.encoder = 0
                #     self.control_msg.gear = 0
                #     self.control_msg.emergency_stop = 1
                #     self.control_msg.auto_manual = 1

                # elif self.planning_info.mode == "normal_stop":
                #     self.normal_stop.run()

                # elif self.planning_info.mode == "parking":

                self.past_mode = self.planning_info.mode
                control_pub.publish(self.control_msg)
                rate.sleep()

    # Callback Function
    def planningCallback(self, msg):
        self.planning_info = msg
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


    # def missionCallback(self, msg):
    #     self.


control = Control()
