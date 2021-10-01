import rospy
from sensor_msgs.msg import PointCloud
from master_node.msg import Obstacles
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from yolov4_trt_ros.msg import Detector2DArray
from yolov4_trt_ros.msg import Detector2D
from shapely.geometry import Point, Polygon
import numpy as np

class calibration:
    def __init__(self):
        rospy.init_node("calibration", anonymous=False)
        self.delivery_publish = rospy.Publisher("delivery_obstacle", Obstacles, queue_size=1)
        # self.outlier_publish = rospy.Publisher("outlier_obs", Obstacles, queue_size=1)
        rospy.Subscriber("lidar_pub", PointCloud, self.get_lidar)
        rospy.Subscriber("obstacles", Obstacles, self.get_obstacles)
        # rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.get_bbox)
        rospy.Subscriber("/detections", Detector2DArray, self.get_bbox)
        self.lidar_msg = PointCloud()
        self.obstacle_msg = Obstacles()
        self.new_obstacle = Obstacles()
        self.bbox_msg = Detector2D()
        self.flag = False
        self.ob_flag = False
        self.lidar_flag = False
        self.camera_matrix = np.array([[1166.853156, 0.000000, 958.517180], [0.000000, 1172.932471, 556.692563], [0.000000, 0.000000, 1.000000]])
        self.inner_temp = Obstacles()
        self.outlier_temp = Obstacles()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.circle_z()
            rate.sleep()

    def get_lidar(self, msg):
        if not self.lidar_flag: 
            self.lidar_msg = msg
        self.lidar_flag = True
    
    def get_obstacles(self, msg):
        if not self.ob_flag:
            self.obstacle_msg = msg
        self.ob_flag = True
        # self.circle_z()

    def get_bbox(self, msg):
        # print("do?")
        if not self.flag:
            self.bbox_msg = msg.detections
        self.flag = True

    def circle_z(self):
        obstacle_space_poly = []
        obs_z = []
        if self.lidar_flag and self.ob_flag:
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

            self.inner_temp.header.frame_id = 'world'
            self.inner_temp.header.stamp = rospy.Time.now()
            self.outlier_temp.header.frame_id = 'world'
            self.outlier_temp.header.stamp = rospy.Time.now()

            # print(self.bbox_msg)
            # print(self.bbox_msg[0].xmin)
            for i in range(len(self.bbox_msg)):
                center_x, center_y = self.bbox_msg[i].bbox.center.x, self.bbox_msg[i].bbox.center.y
                delta_x, delta_y = self.bbox_msg[i].bbox.size_x + 20, self.bbox_msg[i].bbox.size_y + 20
                bbox_space = [[center_x - delta_x, center_y - delta_y], [center_x - delta_x, center_y + delta_y], [center_x + delta_x, center_y + delta_y], [center_x + delta_x, center_y - delta_y]]
                # print(bbox_space)
                bbox_temp = Polygon(bbox_space)
                bbox_space_poly.append(bbox_temp)
            # print(bbox_space_poly)

            for i in range(len(self.obstacle_msg.circles)):
                love_flag = False
                c_x = -0.452 - self.obstacle_msg.circles[i].center.y 
                c_y = 0.52 - self.obstacle_msg.circles[i].center.z
                c_z = 0.6 + self.obstacle_msg.circles[i].center.x
                obs_matrix = np.array([[c_x], [c_y], [c_z]])

                pixel_matrix = np.dot(self.camera_matrix, obs_matrix)

                # print(pixel_matrix.shape)
                # print("obs :", i)
                # print(pixel_matrix[0]/pixel_matrix[2], pixel_matrix[1]/pixel_matrix[2], pixel_matrix[2]/pixel_matrix[2])
                cali_point = Point(pixel_matrix[0]/pixel_matrix[2], pixel_matrix[1]/pixel_matrix[2])

                for p in range(len(bbox_space_poly)):
                    if cali_point.within(bbox_space_poly[p]):
                        # print("in")
                        if self.bbox_msg[p].results.id == 0:
                            self.obstacle_msg.circles[i].name = 0
                            continue
                        elif self.bbox_msg[p].results.id == 1:
                            self.obstacle_msg.circles[i].name = 1
                            continue
                        elif self.bbox_msg[p].results.id == 2:
                            self.obstacle_msg.circles[i].name = 1
                            continue
                        elif self.bbox_msg[p].results.id == 3:
                            self.obstacle_msg.circles[i].name = 3
                            continue
                        elif self.bbox_msg[p].results.id == 4:
                            self.obstacle_msg.circles[i].name = 4
                            continue
                        elif self.bbox_msg[p].results.id == 5:
                            self.obstacle_msg.circles[i].name = 5
                            continue
                        elif self.bbox_msg[p].results.id == 6:
                            self.obstacle_msg.circles[i].name = 6
                            continue
                        else:
                            self.obstacle_msg.circles[i].name = -1
                # self.outlier_temp.circles.append(self.obstacle_msg.circles[i])
        self.delivery_publish.publish(self.obstacle_msg)
            # self.inner_publish.publish(self.inner_temp)
            # self.outlier_publish.publish(self.outlier_temp)
        self.lidar_flag, self.ob_flag, self.flag = False, False, False
                
print("ON") 
Calibration = calibration()       
