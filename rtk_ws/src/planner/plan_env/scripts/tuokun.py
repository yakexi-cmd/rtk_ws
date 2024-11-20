#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import threading

pub = rospy.Publisher('map_2d_line', MarkerArray, queue_size=10)
# pub = rospy.Publisher('map_2d_line', Marker, queue_size=10)
out_msg = MarkerArray()
 
def occCallback(msg):
    # 转换OccupancyGrid为OpenCV图像
    height = msg.info.height
    width = msg.info.width
    data = msg.data
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    resolution = msg.info.resolution
    map_image = np.zeros((height, width, 1), np.uint8)
    for i in range(len(data)):
        row = int(i / width)
        col = i % width
        if data[i] > 90:
            map_image[row, col] = 255

    # 应用Canny边缘检测
    edges = cv2.Canny(map_image, threshold1=100, threshold2=200)
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    out_msg.markers.clear()
    for i in range(len(contours)):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacle_edges"
        marker.id = i
        # marker.lifetime = rospy.Duration(0.5)
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1 #线宽

        # 将边缘点转换为ROS坐标系
        for j in range(len(contours[i])):
            p = Point()
            p.x = origin_x + (contours[i][j])[0][0] * resolution
            p.y = origin_y + (contours[i][j])[0][1] * resolution
            p.z = 0.0
            marker.points.append(p)

        #最后一个点与第一个点连接，形成封闭边缘
        if len(contours[i]) > 1:
            marker.points.append(marker.points[0])

        #设置Marker颜色
        marker.color.a = 1.0
        marker.color.r = 135.0 / 255
        marker.color.g = 206.0 / 255
        marker.color.b = 250.0 / 255

        out_msg.markers.append(marker)
        
    # print(len(out_msg.markers))

def time_thread():
    rate = rospy.Rate(1)
    while(not rospy.is_shutdown()):
        if len(out_msg.markers)>1:
            pub.publish(out_msg)
        rate.sleep()
        
if __name__ == "__main__":
    rospy.init_node('pc2img', anonymous=True)
    sub = rospy.Subscriber('map_2d', OccupancyGrid, occCallback, queue_size=10)
    # timer = rospy.Timer(rospy.Duration(1), timerCallback)
    thread_pub = threading.Thread(target=time_thread)
    thread_pub.start()

    # rospy.spin()



