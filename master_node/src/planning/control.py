import rospy

from master_node import Serial_Info, Planning_Info  # 개발할 메세지 타입
from lib.control_utils.general import General()

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
        self.serial_info = Serial_Info()
        self.global_path=None
        self.lookahead=4

        general=General(self)
        self.is_planning=False


        rate = rospy.Rate(100)  # 100hz

        # main loop
        while not rospy.is_shutdown():

            if self.is_planning:
                if self.planning_info.mode=="general":
                    if self.planning_info.path:
                        self.global_path=self.planning_info.path
                    if self.global_path:
                        self.serial_info.steer=general.pure_pursuit()
                    self.serial_info.speed=10
                    self.serial_info.brake=0
                    self.serial_info.enc=0
                    self.serial_info.gear=0
                    self.serial_info.emergency_stop=0
                    self.serial_info.gear=0
                    self.serial_info.auto_manual=1
            control_pub.publish(self.pub_msg)
            rate.sleep()

    # Callback Function
    def planningCallback(self, msg):
        self.planning_info = msg
        self.is_planning=True

    def serialCallback(self, msg):
        self.serial_info = msg