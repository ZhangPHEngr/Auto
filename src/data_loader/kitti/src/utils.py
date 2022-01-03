#! /usr/bin/python2.7
# -*- coding: UTF-8 -*-
"""
@Project ：  Auto
@Author:     Zhang P.H
@Date ：     1/1/22 3:19 PM
"""
import cv2
import os
import rospy
import tf
import numpy as np
import pandas as pd
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from cv_bridge import CvBridge
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2

# 默认路径
PATH = "/media/zhangph/Elements/dataset/kitti/raw_data/2011_09_26/2011_09_26_drive_0005_sync"

# 转换器
bridge = CvBridge()

# 预定义
IMU_COLUMN_NAMES = ['lat', 'lon', 'alt', 'roll', 'pitch', 'yaw', 'vn', 've', 'vf', 'vl', 'vu', 'ax', 'ay', 'az', 'af','al', 'au', 'wx', 'wy', 'wz', 'wf', 'wl', 'wu', 'posacc', 'velacc', 'navstat', 'numsats', 'posmode','velmode', 'orimode']

class ImagePublisher:
    def __init__(self, queue_size=10):
        # ROS节点初始化
        rospy.init_node("ImagePublisher")
        self.node_name = rospy.get_name()
        self.cam_id = rospy.get_param(self.node_name + "/cam_id")
        self.image_type = rospy.get_param(self.node_name + "/image_type")

        self.topic_name = "{}/rec_{}_{}".format(self.node_name, self.image_type, self.cam_id)
        self.msg_type = Image
        self.queue_size = queue_size

        kitti_path = os.path.join(rospy.get_param("kitti_path", PATH), self.cam_id)
        self.image_dir = os.path.join(kitti_path, "data")
        self.image_ts_file = os.path.join(kitti_path, "timestamps.txt")
        self.img_pub = None

    def init_pub(self):
        # 创建一个Publisher，设置发布topic名称和数据类型，以及默认队列长度
        self.img_pub = rospy.Publisher(self.topic_name, self.msg_type, queue_size=self.queue_size)

    def send(self, img_idx, ts):
        img = cv2.imread(os.path.join(self.image_dir, img_idx))
        self.img_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8' if self.image_type == "rgb" else "bgr8"))
        rospy.loginfo("publish image {} with {} in {} on {}".format(img_idx, img.shape, ts, self.cam_id))

    def load_data_list(self):
        timestamp = []
        with open(self.image_ts_file, "r") as f:
            for line in f:
                line = line.strip('\n')
                timestamp.append(line)
        return os.listdir(self.image_dir), timestamp


class PointCloudPublisher:
    def __init__(self, queue_size=10):
        # ROS节点初始化
        rospy.init_node("PointCloudPublisher")
        self.node_name = rospy.get_name()

        self.topic_name = "{}/velodyne_points".format(self.node_name)
        self.msg_type = PointCloud2
        self.queue_size = queue_size

        kitti_path = os.path.join(rospy.get_param("kitti_path", PATH), "velodyne_points")
        self.points_dir = os.path.join(kitti_path, "data")
        self.points_ts_file = os.path.join(kitti_path, "timestamps.txt")
        self.pt_pub = None

    def init_pub(self):
        # 创建一个Publisher，设置发布topic名称和数据类型，以及默认队列长度
        self.pt_pub = rospy.Publisher(self.topic_name, self.msg_type, queue_size=self.queue_size)

    def send(self, pt_idx, ts):
        point_cloud = np.fromfile(os.path.join(self.points_dir, pt_idx), dtype=np.float32).reshape(-1, 4)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        self.pt_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))
        rospy.loginfo("publish point cloud {} in {} on velodyne".format(pt_idx, ts))

    def load_data_list(self):
        timestamp = []
        with open(self.points_ts_file, "r") as f:
            for line in f:
                line = line.strip('\n')
                timestamp.append(line)
        return os.listdir(self.points_dir), timestamp


class OXTSPublisher:
    """
    参考文档：
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html
    """
    def __init__(self, queue_size=10):
        # ROS节点初始化
        rospy.init_node("OXTSPublisher")
        self.node_name = rospy.get_name()

        self.topic_name_imu = "{}/imu".format(self.node_name)
        self.topic_name_gps = "{}/gps".format(self.node_name)
        self.queue_size = queue_size

        kitti_path = os.path.join(rospy.get_param("kitti_path", PATH), "oxts")
        self.points_dir = os.path.join(kitti_path, "data")
        self.points_ts_file = os.path.join(kitti_path, "timestamps.txt")
        self.pt_pub = None

    def init_pub(self):
        self.imu_pub = rospy.Publisher(self.topic_name_imu, Imu, queue_size=self.queue_size)
        self.gps_pub = rospy.Publisher(self.topic_name_gps, NavSatFix, queue_size=self.queue_size)

    def send(self, oxts_idx, ts):
        oxts_data = pd.read_csv(os.path.join(self.points_dir, oxts_idx), header=None, sep=' ')
        oxts_data.columns = IMU_COLUMN_NAMES
        # 转换imu数据
        imu = Imu()
        imu.header.frame_id = "map"  # 坐标系
        imu.header.stamp = rospy.Time.now()
        q = tf.transformations.quaternion_from_euler(float(oxts_data.roll), float(oxts_data.pitch), float(oxts_data.yaw))  # prevent the data from being overwritten
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = oxts_data.af
        imu.linear_acceleration.y = oxts_data.al
        imu.linear_acceleration.z = oxts_data.au
        imu.angular_velocity.x = oxts_data.wf
        imu.angular_velocity.y = oxts_data.wl
        imu.angular_velocity.z = oxts_data.wu
        self.imu_pub.publish(imu)
        # gps数据转换
        gps = NavSatFix()
        gps.header.frame_id = "map"
        gps.header.stamp = rospy.Time.now()
        gps.latitude = oxts_data.lat
        gps.longitude = oxts_data.lon
        gps.altitude = oxts_data.alt
        self.gps_pub.publish(gps)
        rospy.loginfo("publish IMU and GPS {} in {}".format(oxts_idx, ts))

    def load_data_list(self):
        timestamp = []
        with open(self.points_ts_file, "r") as f:
            for line in f:
                line = line.strip('\n')
                timestamp.append(line)
        return os.listdir(self.points_dir), timestamp
