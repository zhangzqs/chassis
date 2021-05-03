#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
本脚本用于机器人的遥控
"""

import rospy
from geometry_msgs.msg import Twist

import sys
import select
import termios
import tty

msg = """
Control robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

h/; : Left/Right move

w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop

click q to quit
"""

# 运动方向(vx,vy,vth)
moveBindings = {
    'i': (1, 0, 0),  # 前进
    'o': (1, 0, -1),  # 右转前进
    'j': (0, 0, 1),  # 左转
    'l': (0, 0, -1),  # 右转
    'u': (1, 0, 1),  # 左转前进
    ',': (-1, 0, 0),  # 后退
    '.': (-1, 0, 1),  # 左转后退
    'm': (-1, 0, -1),  # 右转后退
    'h': (0, 1, 0),  # 左平移
    ';': (0, -1, 0),  # 右平移
}

# 速度增量
speed_delta = {
    'w': (0.1, 0),  # 增加最大线速度
    'x': (-0.1, 0),  # 减小最大线速度
    'e': (0, 0.1),  # 增大最大角速度
    'c': (0, -0.1),  # 减小最大角速度
}

rospy.init_node('robot_teleop')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)


def get_key() -> str:
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    if select.select([sys.stdin], [], [], 0.1)[0]:
        key_ = sys.stdin.read(1)
    else:
        key_ = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key_


linear_speed: float = 0.2  # 最大线速度
yaw_speed: float = 1  # 最大角速度


def print_current_speed():
    """
    打印当前能达到的最大速度
    """
    global linear_speed, yaw_speed
    print("currently:\t speed %f \t turn %f " % (linear_speed, yaw_speed))


def publish_speed(vx: float, vy: float, vth: float):
    """
    发布速度信息
    """
    # 创建并发布twist消息
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = vth
    pub.publish(twist)


# 方向
x = 0
y = 0
th = 0
# 最终要控制的速度
control_speed_x = 0
control_speed_y = 0
control_turn = 0

def speed_handle(e):
    """
    速度处理函数
    """
    global linear_speed,yaw_speed
    global x,y,th 
    global control_speed_x,control_speed_y,control_turn
    key = get_key()
    
    if key == 'q':
        rospy.loginfo('teleop will normal exit')
        rospy.signal_shutdown("Normal exit")
    # 运动控制方向键（1：正方向，-1负方向）
    elif key in moveBindings.keys():
        x, y, th = moveBindings[key]
    # 速度修改键
    elif key in speed_delta.keys():
        linear_speed += speed_delta[key][0]  # 增大最大线速度
        yaw_speed += speed_delta[key][1]  # 增大最大角速度
        print_current_speed()
    
    # 停止键
    else:
        x, y, th = 0, 0, 0
        if key in (' ', 'k'):
            control_speed_x, control_speed_y, control_turn = 0, 0, 0

    # 目标速度=速度值*方向值
    target_speed_x = linear_speed * x
    target_speed_y = linear_speed * y
    target_turn = yaw_speed * th

    for i, (control, target, delta) in enumerate(((control_speed_x, target_speed_x, 0.005),
                                                  (control_speed_y, target_speed_y, 0.005),
                                                  (control_turn, target_turn, 0.1))):
        if target > control:
            control = min(target, control + delta)
        elif target < control:
            control = max(target, control - delta)

        if i == 0:
            control_speed_x = control
        elif i == 1:
            control_speed_y = control
        else:
            control_turn = control
    publish_speed(control_speed_x, control_speed_y, control_turn)


def on_shutdown():
    """
    当节点关闭时的清理工作
    """
    publish_speed(0,0,0)


# 打印首页
print(msg)
print_current_speed()


tr = rospy.Timer(rospy.Duration(0.05),speed_handle)
rospy.on_shutdown(on_shutdown)

rospy.spin()