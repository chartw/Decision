import rospy
import message_filters
from master_node.msg import Obstacles
from nav_msgs.msg import Odometry

rospy.init_node("test")

def testCallback(lidar,local):
    print("callback")


lidar_sub=message_filters.Subscriber("/obstacles", Obstacles)
local_sub=message_filters.Subscriber("/pose", Odometry)
ats=message_filters.ApproximateTimeSynchronizer([lidar_sub, local_sub],10,0.1,allow_headerless=True)
ats.registerCallback(testCallback)
rospy.spin()