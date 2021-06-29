#-*- coding:utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class MissionPlanner():
    def __init__(self, planner):
        self.data = planner.planning_data
        self.pub_mission=rospy.Publisher('/mission',String,queue_size=1)

        ## 어떤 미션인지 판단에 필요한 센서 정보들만 subscribe. 아마 표지판???

        # rospy.Subscriber("/obstacles", Obstacles, self.obstacleCallback) => callback으로 mission 변경
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


        #Publish Mission to Vision & LiDAR
        self.pub_mission.publish(self.data['mission'])