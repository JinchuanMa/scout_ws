#! /usr/bin/env python3

import rospy
from server_client.srv import *
import sys

"""
客户端：提交请求，处理服务器端响应
    step 1.导包
    step 2.初始化ROS节点
    step 3.创建客户端对象
    step 4.组织请求数据，并发送请求（对客户端而言，请求如何发送、何时发送是可控的，不需要spin函数来处理）
    step 5.处理响应

    优化实现：
        1.可以在执行节点时，动态传入参数

        2.问题：客户端先于服务端启动，会抛出异常
          需要：客户端先于服务端启动，不要抛出异常，而是挂起，等待服务器启动后再发送请求
          实现:ros中内置了相关函数，这些函数可以判断服务器的状态，如果服务器没有启动，就让客户端挂起
              方案1：client.wait_for_service()
              方案2：rospy.wait_for_service("加上你的话题名称")
"""

if __name__ == "__main__":
    # 判断参数长度
    if len(sys.argv) != 3:
        rospy.logerr("传入的参数个数不对！")
        sys.exit(1)

    # step 2.初始化ROS节点
    rospy.init_node("home")
    # step 3.创建客户端对象
    client = rospy.ServiceProxy("add", AddInts)
    # step 4.组织请求数据，并发送请求
    # 4.1.解析传入的参数
    # argv[0]:argv数组索引为0的位置为文件名，argv[1]:索引为1的位置为第二个元素，即输入的第一个数字
    num1 = int(sys.argv[1])
    num2 = int(sys.argv[2])
    # 4.2.等待服务器启动
    client.wait_for_service()
    response = client.call(num1, num2) # 调用client.call函数发送数据，调用完后的返回值为响应的内容，于是在这使用response接收
    # step 5.处理响应
    rospy.loginfo("响应数据：%d", response.sum)
    