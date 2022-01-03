#! /usr/bin/python2.7
# -*- coding: UTF-8 -*-
"""
@Project ：  Auto 
@Author:     Zhang P.H
@Date ：     1/1/22 3:19 PM
"""
import os
import sys
import rospy

sys.path.append(os.path.abspath(__file__))
from utils import ImagePublisher

LOOP = rospy.get_param("loop_mode")


def image_publish():
    # 初始化
    image_publisher = ImagePublisher()
    image_publisher.init_pub()
    # 准备要播放数据
    img_list, ts_list = image_publisher.load_data_list()
    # 设置循环的频率
    rate = rospy.Rate(10)
    # 开始循环发送
    cnt = 0
    while not rospy.is_shutdown():
        img, ts = img_list[cnt], ts_list[cnt]
        # 发布消息
        image_publisher.send(img, ts)
        # 按照循环频率延时
        rate.sleep()
        cnt += 1
        if cnt == len(img_list):
            if LOOP:
                cnt = 0
            else:
                break
    rospy.loginfo("当前node结束：{}".format(rospy.get_name()))


if __name__ == '__main__':
    try:
        image_publish()
    except rospy.ROSInterruptException:
        pass
