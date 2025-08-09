#! /usr/bin/env python3

import rospy
from server_client.srv import *

"""
客户端：提交请求，处理服务器端响应
    step 1.导包
    step 2.初始化ROS节点
    step 3.创建客户端对象
    step 4.组织请求数据，并发送请求（对客户端而言，请求如何发送、何时发送是可控的，不需要spin函数来处理）
    step 5.处理响应
"""

if __name__ == "__main__":
    # step 2.初始化ROS节点
    rospy.init_node("home")
    # step 3.创建客户端对象
    client = rospy.ServiceProxy("add", AddInts)
    # step 4.组织请求数据，并发送请求
    response = client.call(num1 = 10, num2 = 13) # 调用client.call函数发送数据，调用完后的返回值为响应的内容，于是在这使用response接收
    # step 5.处理响应
    rospy.loginfo("响应数据：%d", response.sum)
    