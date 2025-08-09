#! /usr/bin/env python3
import rospy
from std_msgs.msg import String

"""
订阅实现流程
    step 1.导包
    step 2.初始化ros节点
    step 3.创建订阅者对象
    step 4.回调函数处理
    step 5.spin()
"""

def doMsg(msg):
    rospy.loginfo("我订阅的数据：%s", msg.data)

if __name__ == "__main__":
    # step 2.初始化ros节点
    rospy.init_node("cuiHua")
    # step 3.创建订阅者对象
    sub = rospy.Subscriber("car", String, doMsg, queue_size=10) # name, data_class, callback->回调函数doMsg, queue_size
    # step 4.回调函数处理
    # step 5.spin()
    rospy.spin()
