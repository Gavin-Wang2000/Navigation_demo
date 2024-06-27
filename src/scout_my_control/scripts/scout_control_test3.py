#! /usr/bin/env python

"""
    小车功能测试 使用A*算法进行路径规划
"""

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
from queue import PriorityQueue
import math

# 设置地图参数
map_width = 10  # 地图宽度
map_height = 10  # 地图高度
map_resolution = 0.1  # 地图分辨率
map_origin_x = -5.0  # 地图原点x坐标
map_origin_y = -5.0  # 地图原点y坐标

# 设置目标点位置
target_x = 3.5
target_y = 4.0

# 定义地图和起点、终点
map_data = None
start_x = None
start_y = None
end_x = None
end_y = None


class Node:
    def __init__(self, x, y, g_cost, h_cost, parent=None):
        self.x = x
        self.y = y
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.parent = parent

    def f_cost(self):
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        return self.f_cost() < other.f_cost()


def get_map_index(x, y):
    """
    将坐标转换为地图索引
    """
    map_x = int((x - map_origin_x) / map_resolution)
    map_y = int((y - map_origin_y) / map_resolution)
    return map_y * map_width + map_x


def get_map_coord(index):
    """
    将地图索引转换为坐标
    """
    map_x = index % map_width
    map_y = index // map_width
    x = map_x * map_resolution + map_origin_x + map_resolution / 2
    y = map_y * map_resolution + map_origin_y + map_resolution / 2
    return x, y


def get_neighbors(node):
    """
    获取相邻节点
    """
    neighbors = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            x = node.x + dx
            y = node.y + dy
            if x < 0 or x >= map_width or y < 0 or y >= map_height:
                continue
            index = get_map_index(x, y)
            if map_data[index] > 50:
                continue
            g_cost = node.g_cost + math.sqrt(dx ** 2 + dy ** 2)
            h_cost = math.sqrt((x - end_x) ** 2 + (y - end_y) ** 2)
            neighbors.append(Node(x, y, g_cost, h_cost, node))
    return neighbors


def a_star():
    """
    A*算法
    """
    global map_data, start_x, start_y, end_x, end_y
    # 将起点和终点转换为地图索引
    start_index = get_map_index(start_x, start_y)
    end_index = get_map_index(end_x, end_y)
    # 初始化起点和终点节点
    start_node = Node(start_x, start_y, 0, 0)
    end_node = Node(end_x, end_y, 0, 0)
    # 初始化open和closed列表
    open_list = PriorityQueue()
    open_list.put(start_node)
    closed_list = set()
    # 开始搜索
    while not open_list.empty():
        # 从open列表中取出f值最小的节点
        current_node = open_list.get()
        # 如果当前节点是终点，则返回路径
        if current_node.x == end_node.x and current_node.y == end_node.y:
            path = []
            while current_node is not None:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]
        # 将当前节点加入closed列表
        closed_list.add((current_node.x, current_node.y))
        # 获取相邻节点
        neighbors = get_neighbors(current_node)
        for neighbor in neighbors:
            # 如果相邻节点已经在closed列表中，则跳过
            if (neighbor.x, neighbor.y) in closed_list:
                continue
            # 如果相邻节点不在open列表中，则加入open列表
            if neighbor not in open_list.queue:
                open_list.put(neighbor)
            # 如果相邻节点已经在open列表中，则更新g值和父节点
            else:
                for n in open_list.queue:
                    if n.x == neighbor.x and n.y == neighbor.y:
                        if neighbor.g_cost < n.g_cost:
                            n.g_cost = neighbor.g_cost
                            n.parent = neighbor.parent
    # 如果open列表为空，则无法到达终点，返回空路径
    return []


def callback_map(m):
    global map_data
    map_data = m.data


def callback_odom(m):
    global start_x, start_y, end_x, end_y
    # 获取小车当前位置
    x = m.pose.pose.position.x
    y = m.pose.pose.position.y
    orientation = m.pose.pose.orientation
    _, _, yaw = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w]
    )
    # 设置起点和终点
    start_x = x
    start_y = y
    end_x = target_x
    end_y = target_y
    # 进行路径规划
    path = a_star()
    # 发布路径
    pub_path = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    for x, y in path:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y = get_map_coord(get_map_index(x, y))
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        pub_path.publish(pose)

if __name__ == "__main__":
    rospy.init_node("scout_control_test3")
    sub_map = rospy.Subscriber("/map", OccupancyGrid, callback_map)
    sub_odom = rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.spin()