import rospy
from nav_msgs.msg import Odometry


class MissionPlanner():
    def __init__(self, planner):
        self.data = planner.planning_data
        # rospy.Subscriber("/obstacles", Obstacles, self.obstacleCallback)
        # rospy.Subscriber("/pose", Odometry, self.positionCallback)
        # rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.objectCallback)



    def run(self):
        #General
        data['mission'] = 'general'

        #Parking
        data['mission'] = 'parking'

        #Avoidance
        data['mission'] = 'avoidance'

        #Stop
        data['mission'] = 'stop'

        #UTurn
        data['mission'] = 'uturn'
        print("=======mission uturn")        