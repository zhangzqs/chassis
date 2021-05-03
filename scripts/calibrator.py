#!/usr/bin/env python3
# coding=utf-8

"""
本脚本用于底盘系数的标定
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf.transformations
import threading

rospy.init_node("chassis_calibrator")
threading.Thread(target=rospy.spin)

# 定义里程计全局变量
odom_x: float = 0
odom_y: float = 0
odom_th: float = 0


def odom_callback(odom: Odometry):
    """
        里程计信息的回调函数,实时更新里程计信息全局变量
    """
    global odom_x, odom_y, odom_th
    odom_x = odom.pose.pose.position.x
    odom_y = odom.pose.pose.position.y

    # 四元数转欧拉角
    ori: Quaternion = odom.pose.pose.orientation

    _, _, yaw = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

    # 航向角
    odom_th = yaw
    pass


def get_k() -> float:
    """
        获取底盘运动系数
    """
    return rospy.get_param('/chassis/k')


def input_real_dist():
    """
        请求用户从键盘上输入实际距离
    """
    try:
        return float(input('请输入从起点到当前点的实际距离: '))
    except ValueError:
        print('您的输入有误，请重新输入！')
        return input_real_dist()


def calc_k(real_dist: float, odom_dist: float, ori_k: float):
    """
    计算新的底盘运动系数
    """
    new_k = odom_dist / real_dist
    return ori_k * new_k


if __name__ == '__main__':

    rospy.Subscriber("/odom", Odometry, callback=odom_callback)

    while not rospy.is_shutdown():
        real_dist_ = input_real_dist()
        ori_k_ = get_k()
        new_k_ = calc_k(real_dist_, odom_x, ori_k_)
        print('请校准新系数：%f' % new_k_)
