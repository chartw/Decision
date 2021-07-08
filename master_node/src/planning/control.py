import rospy

from master_node.msg import Path, Serial_Info, Planning_Info, Local  # 개발할 메세지 타입
from lib.control_utils.general import General
from lib.control_utils.avoidance import Avoidance
from lib.control_utils.emergency_stop import EmergencyStop
from lib.control_utils.normal_stop import NormalStop

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

        rospy.Subscriber("/serial", Serial_Info, self.serialCallback)
        rospy.Subscriber("/planner", Planning_Info, self.planningCallback)
        self.planning_info = Planning_Info()
        self.local = Local()
        self.serial_info = Serial_Info()
        self.global_path = Path()
        self.lookahead = 4
        self.past_mode = None

        general = General(self)
        avoidance = Avoidance(self)
        emergency_stop = EmergencyStop(self)
        self.normal_stop = NormalStop(self)
        self.is_planning = False

        rate = rospy.Rate(50)  # 100hz

        # main loop
        while not rospy.is_shutdown():
            # print(self.global_path)
            if self.is_planning:
                if self.planning_info.mode == "general":
                    if self.planning_info.path_x:
                        self.global_path.x = self.planning_info.path_x
                        self.global_path.y = self.planning_info.path_y
                        self.global_path.heading = self.planning_info.path_heading
                        self.global_path.k = self.planning_info.path_k
                    if self.global_path.x:
                        self.pub_msg = general.driving()

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
        self.serial_info = msg


control = Control()
