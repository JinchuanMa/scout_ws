#! /usr/bin/env python3

import rospy
# from server_client.srv import AddInts, AddIntsRequest, AddIntsResponse
from server_client.srv import *

"""
服务端：解析客户端请求，产生响应
    step 1.导包
    step 2.初始化ROS节点
    step 3.创建服务端对象
    step 4.处理逻辑（回调函数）
    step 5.spin()
"""

# 定义回调函数
# 参数：封装了请求数据
# 返回值：相应数据
def doNum(request):
    # 1.解析提交的两个整数
    num1 = request.num1
    num2 = request.num2
    # 2.求和
    sum = num1 + num2
    # 3.将结果封装进响应
    response = AddIntsResponse()
    response.sum = sum

    rospy.loginfo("服务器解析的数据：num1 = %d, num2 = %d, 响应的结果：sum = %d", num1, num2, sum)
    return response

if __name__ == "__main__":
    # step 2.初始化ROS节点
    rospy.init_node("company")
    # step 3.创建服务端对象
    server = rospy.Service("add", AddInts, doNum) # 参数：服务通信话题，消息类型，回调函数
    rospy.loginfo("服务器已经启动了！")
    # step 4.处理逻辑（回调函数）
    # step 5.spin()
    rospy.spin()