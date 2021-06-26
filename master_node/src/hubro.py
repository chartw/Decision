import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion

def line_marker(fir, last, name, idd):
    scale = Vector3(0.04, 0.04, 0.04)
    line_mk = Marker()
    line_mk.header.frame_id = "world"
    line_mk.header.stamp = rospy.Time.now()
    line_mk.ns = name
    line_mk.id = idd
    line_mk.type = Marker.ARROW
    line_mk.action = Marker.ADD
    line_mk.pose.position.x = 0.0
    line_mk.pose.position.y = 0.0
    line_mk.pose.position.z = -0.1
    line_mk.pose.orientation.x = 0.0
    line_mk.pose.orientation.y = 0.0
    line_mk.pose.orientation.z = 0.0
    line_mk.pose.orientation.w = 0.0
    line_mk.scale = scale
    line_mk.color.r = 1.0
    line_mk.color.g = 0.0
    line_mk.color.b = 0.0
    line_mk.color.a = 1.0
    line_mk.points.append(fir)
    line_mk.points.append(last)

    return line_mk

def line_detect(point):
    x_sum = 0
    y_sum = 0
    number = len(point)
    for i in range(number):
        x_sum += point[i].x
        y_sum += point[i].y
    number = float(number)
    x_mean = x_sum / number
    y_mean = y_sum / number

    up = 0
    down = 0

    for i in range(int(number)):
        up += (point[i].x-x_mean) * (point[i].y - y_mean)
        down += (point[i].x-x_mean) ** 2

    a = up / down
    b = y_mean - a * x_mean

    return a, b

def getline(msg):
    left = []
    right = []
    center = []
    left_temp = PointCloud()
    right_temp = PointCloud()
    center_temp = PointCloud()
    left_first = Point32()
    left_last = Point32()
    right_first = Point32()
    right_last = Point32()
    center_first = Point32()
    center_last = Point32()
    left_x = []
    right_x = []
    for i in range(len(msg.points)):
        if msg.points[i].y > 0 :
            left = msg.points[i]
            left_x.append(msg.points[i].x)
            left_temp.points.append(left)
        else:
            right = msg.points[i]
            right_x.append(msg.points[i].x)
            right_temp.points.append(right)

    l_a, l_b = line_detect(left_temp.points)
    r_a, r_b = line_detect(right_temp.points)

    c_a = (l_a + r_a) / 2
    c_b = 0.0
    # left_first.x = left_temp.points[0].x
    # left_first.y = left_temp.points[0].x * l_a + l_b
    # left_last.x = left_temp.points[len(left_temp.points)-1].x
    # left_last.y = left_temp.points[len(left_temp.points)-1].x * l_a + l_b

    # right_first.x = right_temp.points[0].x
    # right_first.y = right_temp.points[0].x * r_a + r_b
    # right_last.x = right_temp.points[len(right_temp.points)-1].x
    # right_last.y = right_temp.points[len(right_temp.points)-1].x * r_a + r_b

    left_first.x = min(left_x)
    left_first.y = min(left_x) * l_a + l_b
    left_last.x = max(left_x)
    left_last.y = max(left_x) * l_a + l_b

    right_first.x = min(right_x)
    right_first.y = min(right_x) * r_a + r_b
    right_last.x = max(right_x)
    right_last.y = max(right_x) * r_a + r_b

    center_first.x = 0.0
    center_first.y = 0.0
    center_last.x = 2.0
    center_last.y = 2.0 * c_a + c_b


    l_line = line_marker(left_first, left_last, "left", 0)
    r_line = line_marker(right_first, right_last, "right", 1)
    c_line = line_marker(center_first, center_last, "center", 2)

    left_temp.header.frame_id = 'world'
    left_temp.header.stamp = rospy.Time.now()

    right_temp.header.frame_id = 'world'
    right_temp.header.stamp = rospy.Time.now()


    left_line_pub.publish(l_line)
    right_line_pub.publish(r_line)
    center_line_pub.publish(c_line)

    left_pub.publish(left_temp)
    right_pub.publish(right_temp)

rospy.init_node("hubro", anonymous=True)
left_pub = rospy.Publisher("leftline_pub", PointCloud ,queue_size = 10)
right_pub = rospy.Publisher("rightline_pub", PointCloud ,queue_size = 10)
left_line_pub = rospy.Publisher('left_line', Marker, queue_size = 10)
right_line_pub = rospy.Publisher('right_line', Marker, queue_size = 10)
center_line_pub = rospy.Publisher('center_line', Marker, queue_size = 10)
while not rospy.is_shutdown():
    rospy.Subscriber("lidar_pub", PointCloud, getline)
    rospy.spin()