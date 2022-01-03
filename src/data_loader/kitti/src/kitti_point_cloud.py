#! /usr/bin/python2.7
# -*- coding: UTF-8 -*-
"""
@Project ：  Auto 
@Author:     Zhang P.H
@Date ：     1/3/22 4:24 PM
"""
import os
import sys
import rospy

sys.path.append(os.path.abspath(__file__))
from utils import PointCloudPublisher

LOOP = rospy.get_param("loop_mode")


def point_cloud_publish():
    # 初始化
    pt_publisher = PointCloudPublisher()
    pt_publisher.init_pub()
    # 准备要播放数据
    pt_list, ts_list = pt_publisher.load_data_list()
    # 设置循环的频率
    rate = rospy.Rate(10)
    # 开始循环发送
    cnt = 0
    while not rospy.is_shutdown():
        img, ts = pt_list[cnt], ts_list[cnt]
        # 发布消息
        pt_publisher.send(img, ts)
        # 按照循环频率延时
        rate.sleep()
        cnt += 1
        if cnt == len(pt_list):
            if LOOP:
                cnt = 0
            else:
                break
    rospy.loginfo("当前node结束：{}".format(rospy.get_name()))


if __name__ == '__main__':
    try:
        point_cloud_publish()
    except rospy.ROSInterruptException:
        pass
