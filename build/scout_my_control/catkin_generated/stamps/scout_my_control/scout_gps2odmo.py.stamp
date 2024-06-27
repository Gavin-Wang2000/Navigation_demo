#! /usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import utm
import math

# 小车在世界坐标系下的位置和偏航角
world_x = 0.0
world_y = 0.0
world_yaw = 0.0

def callback_gps(msg):
    global world_x, world_y, world_yaw
    # 将经纬度转换为UTM坐标系下的坐标
    utm_easting, utm_northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
    # 将UTM坐标系下的坐标转换为世界坐标系下的xy坐标
    x = (utm_easting - world_x) * math.cos(world_yaw) - (utm_northing - world_y) * math.sin(world_yaw)
    y = (utm_easting - world_x) * math.sin(world_yaw) + (utm_northing - world_y) * math.cos(world_yaw)
    # 发布世界坐标系下的坐标
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "world"
    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.orientation.z = math.sin(world_yaw / 2)
    odom_msg.pose.pose.orientation.w = math.cos(world_yaw / 2)
    pub_odom.publish(odom_msg)

if __name__=="__main__":
    rospy.init_node("gps_to_odom")
    sub_gps = rospy.Subscriber("/gps/fix", NavSatFix, callback_gps)
    pub_odom = rospy.Publisher("/gps2odom_world", Odometry, queue_size=10)

    # 将小车在世界坐标系下的初始位置设置为(0, 0)
    msg = rospy.wait_for_message("/gps/fix", NavSatFix)
    utm_easting, utm_northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
    world_x = utm_easting
    world_y = utm_northing

    rospy.spin()
