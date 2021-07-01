import rospy

from master_node import Serial_Info, Planning_Info  # 개발할 메세지 타입

"""
Serial_Info
{
    auto or man
    e-stop
    gear
    speed
    steer
    brake
    enc
}
Planning_Info
{
    String mode
    Local local
    Path path
    Point32 target 
}
"""
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry


class Control:
    def __init__(self):
        rospy.init_node("Control", anonymous=False)

        control_pub = rospy.Publisher("/control", Serial_Info, queue_size=1)
        self.pub_msg = Serial_Info()

        rospy.Subscriber("/serial", Serial_Info, self.serialCallback)
        rospy.Subscriber("/planner", Planning_Info, self.planningCallback)
        self.planning_info = Planning_Info()
        self.serial_info = Serial_Info()

        rate = rospy.Rate(100)  # 100hz

        # main loop
        while not rospy.is_shutdown():

            control_pub.publish(self.pub_msg)
            rate.sleep()

    # Callback Function
    def planningCallback(self, msg):
        self.planning_info = msg

    def serialCallback(self, msg):
        self.serial_info = msg
