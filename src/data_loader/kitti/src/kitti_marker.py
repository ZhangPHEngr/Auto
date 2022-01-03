#! /usr/bin/python2.7
# -*- coding: UTF-8 -*-
"""
@Project ：  Auto 
@Author:     Zhang P.H
@Date ：     1/3/22 4:57 PM
"""

# ! /usr/bin/python2.7
# -*- coding: UTF-8 -*-
"""
@Project ：  Auto 
@Author:     Zhang P.H
@Date ：     1/3/22 4:24 PM
"""
import os
import sys
import rospy
import tf
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

FRAME_ID = 'map'
marker_array = MarkerArray()


def add_ego_car():
    # publish left and right 45 degree FOV lines and ego car model mesh
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()

    marker.id = 0
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration()
    marker.type = Marker.LINE_STRIP
    # line
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 0.2  # line width

    marker.points = []

    # check the kitti axis model
    marker.points.append(Point(5, -5, 0))  # left up
    marker.points.append(Point(0, 0, 0))  # center
    marker.points.append(Point(5, 5, 0))  # right up

    marker_array.markers.append(marker)


def add_car_model():
    mesh_marker = Marker()
    mesh_marker.header.frame_id = FRAME_ID
    mesh_marker.header.stamp = rospy.Time.now()

    mesh_marker.id = -1
    mesh_marker.lifetime = rospy.Duration()
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.mesh_resource = "/home/zhangph/develop/Auto/src/data_loader/kitti/sources/Car.dae"  #@TODO error

    mesh_marker.pose.position.x = 0.0
    mesh_marker.pose.position.y = 0.0
    mesh_marker.pose.position.z = -1.73

    q = tf.transformations.quaternion_from_euler(np.pi / 2, 0, np.pi)
    mesh_marker.pose.orientation.x = q[0]
    mesh_marker.pose.orientation.y = q[1]
    mesh_marker.pose.orientation.z = q[2]
    mesh_marker.pose.orientation.w = q[3]

    mesh_marker.color.r = 1.0
    mesh_marker.color.g = 1.0
    mesh_marker.color.b = 1.0
    mesh_marker.color.a = 1.0

    mesh_marker.scale.x = 0.9
    mesh_marker.scale.y = 0.9
    mesh_marker.scale.z = 0.9

    marker_array.markers.append(mesh_marker)


def marker_publish():

    # ROS节点初始化
    rospy.init_node("kitti_marker")
    ego_pub = rospy.Publisher('kitti_marker_info', MarkerArray, queue_size=10)
    rospy.loginfo("当前node开启：{}".format(rospy.get_name()))
    # 设置循环的频率
    rate = rospy.Rate(10)
    # 添加marker
    add_ego_car()
    add_car_model()
    # 循环播放
    while not rospy.is_shutdown():
        ego_pub.publish(marker_array)
    rospy.loginfo("当前node结束：{}".format(rospy.get_name()))


if __name__ == '__main__':
    try:
        marker_publish()
    except rospy.ROSInterruptException:
        pass
