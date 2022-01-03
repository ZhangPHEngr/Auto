#! /usr/bin/python2.7
# -*- coding: UTF-8 -*-
"""
@Project ：  Auto 
@Author:     Zhang P.H
@Date ：     1/3/22 4:56 PM
"""
import os
import sys
import rospy

sys.path.append(os.path.abspath(__file__))
from utils import OXTSPublisher

LOOP = rospy.get_param("loop_mode", True)


def oxts_publish():
    # 初始化
    oxts_publisher = OXTSPublisher()
    oxts_publisher.init_pub()
    # 准备要播放数据
    data_list, ts_list = oxts_publisher.load_data_list()
    # 设置循环的频率
    rate = rospy.Rate(10)
    # 开始循环发送
    cnt = 0
    while not rospy.is_shutdown():
        data, ts = data_list[cnt], ts_list[cnt]
        # 发布消息
        oxts_publisher.send(data, ts)
        # 按照循环频率延时
        rate.sleep()
        cnt += 1
        if cnt == len(data_list):
            if LOOP:
                cnt = 0
            else:
                break
    rospy.loginfo("当前node结束：{}".format(rospy.get_name()))


if __name__ == '__main__':
    try:
        oxts_publish()
    except rospy.ROSInterruptException:
        pass