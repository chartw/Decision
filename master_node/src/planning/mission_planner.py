#-*- coding:utf-8 -*-

import rospy
from nav_msgs.msg import Odometry


class MissionPlanner():
    def __init__(self, planner):
        self.data = planner.planning_data
        # rospy.Subscriber("/obstacles", Obstacles, self.obstacleCallback)
        # rospy.Subscriber("/pose", Odometry, self.positionCallback)
        # rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.objectCallback)



    def run(self):

        #Parking
        if 1 is 2:
            self.data['mission'] = 'parking'
            print("MissionPlanner: mission parking")        

        #Avoidance
        elif 2 is 3:
            self.data['mission'] = 'avoidance'
            print("MissionPlanner: mission avoidance")        

        #Stop
        elif 3 is 4:
            self.data['mission'] = 'stop'
            print("MissionPlanner: mission stop")        

        #UTurn
        elif 4 is 5:
            self.data['mission'] = 'uturn'
            print("MissionPlanner: mission uturn")

        #General
        else:
            self.data['mission'] = 'general'
            print("MissionPlanner: mission general")        
