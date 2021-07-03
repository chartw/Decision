import rospy

from master_node.msg import Path, Serial_Info, Planning_Info  # 개발할 메세지 타입
from lib.control_utils.general import General
from lib.control_utils.emergency_stop import EmergencyStop

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
        self.global_path=Path()
        self.lookahead=4

        general=General(self)
        self.is_planning=False
        emergency_stop=EmergencyStop(self)

        rate = rospy.Rate(100)  # 100hz

        # main loop
        while not rospy.is_shutdown():
            # print(self.global_path)
            if self.is_planning:
                if self.planning_info.mode=="general":
                    if self.planning_info.path_x:
                        self.global_path.x=self.planning_info.path_x
                        self.global_path.y=self.planning_info.path_y
                        self.global_path.heading=self.planning_info.path_heading
                    if self.global_path.x:
                        self.pub_msg.steer=general.pure_pursuit()
                    self.pub_msg.speed=10
                    self.pub_msg.brake=0
                    self.pub_msg.encoder=0
                    self.pub_msg.gear=0
                    self.pub_msg.emergency_stop=0
                    self.pub_msg.gear=0
                    self.pub_msg.auto_manual=1
            
                        
            
            
            control_pub.publish(self.pub_msg)
            rate.sleep()

    # Callback Function
    def planningCallback(self, msg):
        self.planning_info = msg
        self.is_planning=True

    def serialCallback(self, msg):
        self.serial_info = msg

    
control=Control()