import rospy
from sensor_msgs.msg import PointCloud
from master_node.msg import Obstacles
from darknet_ros_msgs.msg import BoundingBoxes
from yolov4_trt_ros.msg import Detector2DArray
from yolov4_trt_ros.msg import Detector2D
from darknet_ros_msgs.msg import BoundingBox
from shapely.geometry import Point, Polygon
from math import sin, cos, radians
import numpy as np

class calibration:
    def __init__(self):
        rospy.init_node("calibration", anonymous=False)
        delivery_publish = rospy.Publisher("delivery_cali", Obstacles, queue_size=1)
        # outlier_publish = rospy.Publisher("outlier_obs", Obstacles(), queue_size=1)
        rospy.Subscriber("lidar_pub", PointCloud, self.get_lidar)
        # rospy.Subscriber("obstacles", Obstacles, self.get_obstacles)
        rospy.Subscriber("/detections", Detector2DArray, self.get_bbox)
        #9.8
        self.lidar_msg = PointCloud()
        self.bbox_msg = Detector2D()
        self.lidar_flag = False
        self.bbox_flag = False
        self.flag = False
        # self.camera_matrix = np.array([[1166.853156, 0.000000, 958.517180], [0.000000, 1172.932471, 556.692563], [0.000000, 0.000000, 1.000000]])
        self.camera_matrix = np.array([[1148.907893521547, 0.0, 963.4094241049174], [0.0, 1145.644693048819, 566.6038679464556], [0.000000, 0.000000, 1.000000]])
        self.inner_temp = Obstacles()
        self.outlier_temp = Obstacles()
        self.radi = radians(9.8)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.point_process()
            rate.sleep()

    def get_lidar(self, msg):
        if not self.lidar_flag:
            self.lidar_msg = msg
            self.lidar_flag = True
    
    def get_bbox(self, msg):
        if not self.bbox_flag:
            self.bbox_msg = msg.detections
            self.bbox_flag = True

    def point_process(self):
        bbox_space_poly = []
        bbox_temp_xyzcnt = []
        
        if self.lidar_flag and self.bbox_flag:
            for i in range(len(self.bbox_msg)):
                center_x, center_y = self.bbox_msg[i].bbox.center.x, self.bbox_msg[i].bbox.center.y
                delta_x, delta_y = self.bbox_msg[i].bbox.size_x/2 , self.bbox_msg[i].bbox.size_y/2 
                bbox_space = [[center_x - delta_x, center_y - delta_y], [center_x - delta_x, center_y + delta_y], [center_x + delta_x, center_y + delta_y], [center_x + delta_x, center_y - delta_y]]
                # print(bbox_space)
                bbox_temp = Polygon(bbox_space)
                bbox_space_poly.append(bbox_temp)
                bbox_temp_xyzcnt.append([0,0,0,0])

            for i in range(len(self.lidar_msg.points)):
                c_x = -self.lidar_msg.points[i].y
                c_y = 1.06 + self.lidar_msg.points[i].x * sin(self.radi) - self.lidar_msg.points[i].z * cos(self.radi)
                c_z = 0.46 + self.lidar_msg.points[i].z * sin(self.radi) + self.lidar_msg.points[i].x * cos(self.radi)
                # print(c_x, c_y, c_z)
                obs_matrix = np.array([[c_x], [c_y], [c_z]])
                pixel_matrix = np.dot(self.camera_matrix, obs_matrix)

                cali_point = Point(pixel_matrix[0]/pixel_matrix[2], pixel_matrix[1]/pixel_matrix[2])
                # print(cali_point.x, cali_point.y)
                for p in range(len(bbox_space_poly)):
                    if cali_point.within(bbox_space_poly[p]):
                        bbox_temp_xyzcnt[p][0] += self.lidar_msg.points[i].x
                        bbox_temp_xyzcnt[p][1] += self.lidar_msg.points[i].y
                        bbox_temp_xyzcnt[p][2] += self.lidar_msg.points[i].z
                        bbox_temp_xyzcnt[p][3] += 1

            for i in range(len(bbox_space_poly)):
                if bbox_temp_xyzcnt[i][3] != 0:
                    bbox_temp_xyzcnt[i][0] = bbox_temp_xyzcnt[i][0]/bbox_temp_xyzcnt[i][3]
                    bbox_temp_xyzcnt[i][1] = bbox_temp_xyzcnt[i][1]/bbox_temp_xyzcnt[i][3]
                    bbox_temp_xyzcnt[i][2] = bbox_temp_xyzcnt[i][2]/bbox_temp_xyzcnt[i][3]
                    print(self.bbox_msg[i].results.id, bbox_temp_xyzcnt[i][0], bbox_temp_xyzcnt[i][1], bbox_temp_xyzcnt[i][2], bbox_temp_xyzcnt[i][3])
            
        self.lidar_flag = False
        self.bbox_flag = False

        
    def circle_z(self):
        obstacle_space_poly = []
        obs_z = []

        for i in range(len(self.obstacle_msg.circles)):
            x1, x2 = self.obstacle_msg.circles[i].center.x - self.obstacle_msg.circles[i].radius, self.obstacle_msg.circles[i].center.x + self.obstacle_msg.circles[i].radius
            y1, y2 = self.obstacle_msg.circles[i].center.y - self.obstacle_msg.circles[i].radius, self.obstacle_msg.circles[i].center.y + self.obstacle_msg.circles[i].radius

            obstacle_space = [[x1, y1], [x1, y2], [x2, y2], [x2, y1]]
            obstalce_temp = Polygon(obstacle_space)
            obstacle_space_poly.append(obstalce_temp)

        for i in range(len(self.lidar_msg.points)):
            obs_z.append([0,0])

        for i in range(len(self.lidar_msg.points)):
            test_point = Point(self.lidar_msg.points[i].x, self.lidar_msg.points[i].y)
            for p in range(len(obstacle_space_poly)):
                if test_point.within(obstacle_space_poly[p]):
                    obs_z[p][0] += self.lidar_msg.points[i].z
                    obs_z[p][1] += 1

        for i in range(len(obs_z)):
            if obs_z[i][1] == 0:
                obs_z[i][1] = 1

        for i in range(len(self.obstacle_msg.circles)):
            self.obstacle_msg.circles[i].center.z = obs_z[i][0] / obs_z[i][1]
        
        self.calibration()
    
    def calibration(self):
        atf_cali = []
        bbox_space_poly = []
        
        # print(self.bbox_msg[1])
        
        if self.flag:
            self.inner_temp = Obstacles()
            self.outlier_temp = Obstacles()

            # print(self.bbox_msg)
            # print(self.bbox_msg[0].xmin)
            for i in range(len(self.bbox_msg)):
                bbox_space = [[self.bbox_msg[i].xmin, self.bbox_msg[i].ymin], [self.bbox_msg[i].xmin, self.bbox_msg[i].ymax], [self.bbox_msg[i].xmax, self.bbox_msg[i].ymax], [self.bbox_msg[i].xmax, self.bbox_msg[i].ymin]]
                bbox_temp = Polygon(bbox_space)
                bbox_space_poly.append(bbox_temp)

            for i in range(len(self.obstacle_msg.circles)):
                c_x = -self.obstacle_msg.circles[i].center.y
                c_y = 1.13 - self.obstacle_msg.circles[i].center.z
                c_z = 0.6 + self.obstacle_msg.circles[i].center.x
                obs_matrix = np.array([[c_x], [c_y], [c_z]])

                pixel_matrix = np.dot(self.camera_matrix, obs_matrix)

                # print(pixel_matrix.shape)
                print("obs :", i)
                print(pixel_matrix[0]/pixel_matrix[2], pixel_matrix[1]/pixel_matrix[2], pixel_matrix[2]/pixel_matrix[2])
                cali_point = Point(pixel_matrix[0]/pixel_matrix[2], pixel_matrix[1]/pixel_matrix[2])

                for p in range(len(bbox_space_poly)):
                    if cali_point.within(bbox_space_poly[p]):
                        if self.bbox_msg[p].id == 'corn':
                            self.inner_temp.circles.append(self.obstacle_msg.circles[i])
                        else:
                            self.outlier_temp.circles.append(self.obstacle_msg.circles[i])            
                        # print(self.bbox_msg[p].id)
                        # print(self.bbox_msg[p].Class)
                self.inner_publish.publish(self.inner_temp)
                self.outlier_publish.publish(self.outlier_temp)
                
print("ON") 
Calibration = calibration()       
