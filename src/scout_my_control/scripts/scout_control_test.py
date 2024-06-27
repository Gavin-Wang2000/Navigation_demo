#! /usr/bin/env python


"""
    功能：通过给/cmd/vel节点发送指令来控制scout机器人的运动
        具体实现：让小车走一定距离之后停下来，通过订阅小车里程计在回调函数中进行实现
    通过odom_gazebo节点订阅小车的里程计信息,通过tf库将小车的四元数转换为欧拉角
"""
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations
import math

mode_State = 0  # 小车行走模式状态机编写


def callback(m):
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # 创建发布者对象
    msg = Twist()
    if m.pose.pose.position.x < 5.0:
        msg.linear.x = 1.0
    else:
        msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 1.0
    pub.publish(msg)
    rospy.loginfo(
        "里程计:x:%.2f,y:%.2f\r\n", m.pose.pose.position.x, m.pose.pose.position.y
    )  # 打印小车的里程计信息
    # 将四元数转换为欧拉角
    q = [
        m.pose.pose.orientation.x,
        m.pose.pose.orientation.y,
        m.pose.pose.orientation.z,
        m.pose.pose.orientation.w,
    ]
    euler = tf.transformations.euler_from_quaternion(q)  # 调用tf库将四元数转换为欧拉角
    # 将获得到的欧拉角转换为-180 - 180度
    roll = euler[0] * 180 / math.pi
    pitch = euler[1] * 180 / math.pi
    yaw = euler[2] * 180 / math.pi
    rospy.loginfo(
        "欧拉角:roll:%.2f,pitch:%.2f,yaw:%.2f\r\n", roll, pitch, yaw
    )  # 打印小车的欧拉角信息


if __name__ == "__main__":
    rospy.init_node("scout_control_test")
    rate = rospy.Rate(10)  # 设置发布频率为10Hz
    sub = rospy.Subscriber(
        "/odom_gazebo", Odometry, callback, queue_size=100
    )  # 订阅小车的里程计
    while rospy.is_shutdown:
        rate.sleep()  # 控制消息发布频率
