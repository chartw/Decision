import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion

def cube_marker(fir, last, name, idd):
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

    # line_mk.points.append([2.21831,-2.03464])
    # line_mk.points.append([6.50700,-4.43930])
    # line_mk.points.append([5.32394,-6.65894])
    # line_mk.points.append(fir),[2.21831,-2.03464],[6.50700,-4.43930],[5.32394,-6.65894]
    # line_mk.points.append(last)

    return line_mk

rospy.init_node("cube", anonymous=True)
line1_pub = rospy.Publisher('line1', Marker, queue_size = 10)
line2_pub = rospy.Publisher('line2', Marker, queue_size = 10)
line3_pub = rospy.Publisher('line3', Marker, queue_size = 10)
line4_pub = rospy.Publisher('line4', Marker, queue_size = 10)
line5_pub = rospy.Publisher('line5', Marker, queue_size = 10)
line6_pub = rospy.Publisher('line6', Marker, queue_size = 10)
line7_pub = rospy.Publisher('line7', Marker, queue_size = 10)
line8_pub = rospy.Publisher('line8', Marker, queue_size = 10)
line9_pub = rospy.Publisher('line9', Marker, queue_size = 10)
line10_pub = rospy.Publisher('line10', Marker, queue_size = 10)
line11_pub = rospy.Publisher('line11', Marker, queue_size = 10)
line12_pub = rospy.Publisher('line12', Marker, queue_size = 10)
line13_pub = rospy.Publisher('line13', Marker, queue_size = 10)
line14_pub = rospy.Publisher('line14', Marker, queue_size = 10)
line15_pub = rospy.Publisher('line15', Marker, queue_size = 10)
line16_pub = rospy.Publisher('line16', Marker, queue_size = 10)
line17_pub = rospy.Publisher('line17', Marker, queue_size = 10)
line18_pub = rospy.Publisher('line18', Marker, queue_size = 10)
line19_pub = rospy.Publisher('line19', Marker, queue_size = 10)


# cube2_pub = rospy.Publisher('cube2', Marker, queue_size = 10)
# cube3_pub = rospy.Publisher('cube3', Marker, queue_size = 10)
# cube4_pub = rospy.Publisher('cube4', Marker, queue_size = 10)\
parking_point_x_y = [[9.021042, 9.433509],[8.873212, 12.023061],[10.647854, 12.023063],[10.352018, 14.612615],[11.978832, 14.612617],[11.830912, 17.202281],[13.309809, 17.202283],[13.161889, 19.791835],[14.640786, 19.791837],[14.492865, 22.381390],[15.971761, 22.566400],[15.823840, 24.971055],[20.709098, 25.340970],[20.556188, 22.751418],[19.225208, 22.751414],[19.373130, 20.161751],[17.894234, 20.161748],[17.894239, 17.572196],[16.563259, 17.387185],[16.563263, 14.982641],[15.084455, 14.797630],[15.232287, 12.392975],[13.753477, 12.392972],[13.901309, 9.803420]]
# parking_point_x_y = [[8.87, 12.02],[8.873212, 12.023061],[10.647854, 12.023063],[10.352018, 14.612615],[11.978832, 14.612617],[11.830912, 17.202281],[13.309809, 17.202283],[13.161889, 19.791835],[14.640786, 19.791837],[14.492865, 22.381390],[15.971761, 22.566400],[15.823840, 24.971055],[20.709098, 25.340970],[20.556188, 22.751418],[19.225208, 22.751414],[19.373130, 20.161751],[17.894234, 20.161748],[17.894239, 17.572196],[16.563259, 17.387185],[16.563263, 14.982641],[15.084455, 14.797630],[15.232287, 12.392975],[13.753477, 12.392972],[13.901309, 9.803420]]

one = Point()
one.x = parking_point_x_y[0][0]
one.y = parking_point_x_y[0][1]

two = Point()
two.x = parking_point_x_y[1][0]
two.y = parking_point_x_y[1][1]

three = Point()
three.x = parking_point_x_y[2][0]
three.y = parking_point_x_y[2][1]

four = Point()
four.x = parking_point_x_y[3][0]
four.y = parking_point_x_y[3][1]

five = Point()
five.x = parking_point_x_y[4][0]
five.y = parking_point_x_y[4][1]

six = Point()
six.x = parking_point_x_y[5][0]
six.y = parking_point_x_y[5][1]

seven = Point()
seven.x = parking_point_x_y[6][0]
seven.y = parking_point_x_y[6][1]

eight = Point()
eight.x = parking_point_x_y[7][0]
eight.y = parking_point_x_y[7][1]

nine = Point()
nine.x = parking_point_x_y[8][0]
nine.y = parking_point_x_y[8][1]

ten = Point()
ten.x = parking_point_x_y[9][0]
ten.y = parking_point_x_y[9][1]

t_one = Point()
t_one.x = parking_point_x_y[10][0]
t_one.y = parking_point_x_y[10][1]

t_two = Point()
t_two.x = parking_point_x_y[11][0]
t_two.y = parking_point_x_y[11][1]

t_three = Point()
t_three.x = parking_point_x_y[12][0]
t_three.y = parking_point_x_y[12][1]

t_four = Point()
t_four.x = parking_point_x_y[13][0]
t_four.y = parking_point_x_y[13][1]

t_five = Point()
t_five.x = parking_point_x_y[14][0]
t_five.y = parking_point_x_y[14][1]

t_six = Point()
t_six.x = parking_point_x_y[15][0]
t_six.y = parking_point_x_y[15][1]

t_seven = Point()
t_seven.x = parking_point_x_y[16][0]
t_seven.y = parking_point_x_y[16][1]

t_eight = Point()
t_eight.x = parking_point_x_y[17][0]
t_eight.y = parking_point_x_y[17][1]

t_nine = Point()
t_nine.x = parking_point_x_y[18][0]
t_nine.y = parking_point_x_y[18][1]

twelve = Point()
twelve.x = parking_point_x_y[19][0]
twelve.y = parking_point_x_y[19][1]

tw_one = Point()
tw_one.x = parking_point_x_y[20][0]
tw_one.y = parking_point_x_y[20][1]

tw_two = Point()
tw_two.x = parking_point_x_y[21][0]
tw_two.y = parking_point_x_y[21][1]

tw_three = Point()
tw_three.x = parking_point_x_y[22][0]
tw_three.y = parking_point_x_y[22][1]

tw_four = Point()
tw_four.x = parking_point_x_y[23][0]
tw_four.y = parking_point_x_y[23][1]

# cube1 = cube_marker("cube1", 1, 3.77112, -4.34679, 1, 0, 0)
line1 = cube_marker(one, tw_four, "line1", 1)
line2 = cube_marker(one, two, "line2", 1)
line3 = cube_marker(two, tw_two, "line3", 1)
line4 = cube_marker(tw_three, tw_four, "line4", 1)
line5 = cube_marker(three, four, "line5", 1)
line6 = cube_marker(tw_two, tw_one, "line6", 1)
line7 = cube_marker(four, twelve, "line7", 1)
line8 = cube_marker(five, six, "line8", 1)
line9 = cube_marker(twelve, t_nine, "line9", 1)
line10 = cube_marker(six, t_eight, "line10", 1)
line11 = cube_marker(seven, eight, "line11", 1)
line12 = cube_marker(t_seven, t_eight, "line12", 1)
line13 = cube_marker(eight, t_six, "line13", 1)
line14 = cube_marker(nine, ten, "line14", 1)
line15 = cube_marker(t_five, t_six, "line15", 1)
line16 = cube_marker(ten, t_four, "line16", 1)
line17 = cube_marker(t_one, t_two, "line17", 1)
line18 = cube_marker(t_three, t_four, "line18", 1)
line19 = cube_marker(t_two, t_three, "line19", 1)
# line13 = cube_marker(tk, clf, "line13", 1)
# cube2 = cube_marker("cube2", 2, 4.95421, -2.12715, 0, 1, 0)
# cube3 = cube_marker("cube3", 3, 6.10034, -0.09252, 0, 0, 1)
# cube4 = cube_marker("cube4", 4, 7.20952, 2.31216, 0.5, 0.5, 1)
# while not rospy.is_shutdown():
#     line1_pub.publish(line1)
#     rospy.spin()

while True:
    line1_pub.publish(line1)
    line2_pub.publish(line2)
    line3_pub.publish(line3)
    line4_pub.publish(line4)
    line5_pub.publish(line5)
    line6_pub.publish(line6)
    line7_pub.publish(line7)
    line8_pub.publish(line8)
    line9_pub.publish(line9)
    line10_pub.publish(line10)
    line11_pub.publish(line11)
    line12_pub.publish(line12)
    line13_pub.publish(line13)
    line14_pub.publish(line14)
    line15_pub.publish(line15)
    line16_pub.publish(line16)
    line17_pub.publish(line17)
    line18_pub.publish(line18)
    line19_pub.publish(line19)



# cube2_pub.publish(cube2)
# cube3_pub.publish(cube3)
# cube4_pub.publish(cube4)
