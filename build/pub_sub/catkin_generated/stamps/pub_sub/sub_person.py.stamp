#! /usr/bin/env python3

import rospy
from pub_sub.msg import Person

"""
订阅消息：订阅person的消息
    step 1.导入包
    step 2.初始化ros节点
    step 3.创建订阅者对象
    step 4.通过回调函数处理订阅的数据
    step 5.spin()
"""

# 定义回调函数
def doPersonMsg(people): # 参数：订阅到的数据
    rospy.loginfo("订阅的人的信息：%s, %d, %f", people.name, people.age, people.height)

if __name__ == "__main__":
    # step 2.初始化ros节点
    rospy.init_node("lisilu") # 订阅者lulisi

    # step 3.创建订阅者对象
    sub = rospy.Subscriber("chat", Person, doPersonMsg, queue_size=10) # 参数：话题名称（和发布方一样），订阅消息的类型，回调函数（需要在主函数外定义一下），队列长度
    
    # step 4.通过回调函数处理订阅的数据
    
    # step 5.spin()
    rospy.spin()