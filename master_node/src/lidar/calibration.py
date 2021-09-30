import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from master_node.msg import Obstacles, CalibObjects
from yolov4_trt_ros.msg import Detector2DArray
from yolov4_trt_ros.msg import Detector2D
from shapely.geometry import Point, Polygon
from math import sin, cos, radians
import numpy as np

class calibration:
    def __init__(self):
        rospy.init_node("calibration", anonymous=False)
        self.calib_pub = rospy.Publisher("/calib_object", CalibObjects, queue_size=1)
        self.pub_msg=CalibObjects
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
        self.pub_msg=CalibObjects()
        
        if self.lidar_flag and self.bbox_flag:
            for i in range(len(self.bbox_msg)):
                center_x, center_y = self.bbox_msg[i].bbox.center.x, self.bbox_msg[i].bbox.center.y
                delta_x, delta_y = self.bbox_msg[i].bbox.size_x/3 , self.bbox_msg[i].bbox.size_y/3 
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
                    point=Point32(bbox_temp_xyzcnt[i][0], bbox_temp_xyzcnt[i][1], bbox_temp_xyzcnt[i][2])
                    self.pub_msg.points.append(point)
                    self.pub_msg.Classes.append(self.bbox_msg[i].results.id)
        
        self.pub_msg.header.stamp=rospy.Time.now()
        self.pub_msg.header.frame_id = "world"

        self.calib_pub.publish(self.pub_msg)

        self.lidar_flag = False
        self.bbox_flag = False
                
print("ON") 
Calibration = calibration()       
