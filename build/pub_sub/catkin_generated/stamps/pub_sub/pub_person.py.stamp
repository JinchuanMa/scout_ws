#! /usr/bin/env python3
import rospy
from pub_sub.msg import Person

"""
发布消息：发布person的消息
    step 1.导入包
    step 2.初始化ros节点
    step 3.创建发布者对象
    step 4.组织发布逻辑并发布数据
"""

if __name__ == "__main__":
    # step 2.初始化ros节点
    rospy.init_node("jinchuanma") # 发布者majinchuan

    # step 3.创建发布者对象
    pub = rospy.Publisher("chat", Person, queue_size=10) # 参数：话题名称, 话题类型， 队列长度

    # step 4.组织发布逻辑并发布数据
    # 4.1 创建person数据
    people = Person()
    people.name = "奥特曼"
    people.age = 18
    people.height = 1.8
    # 4.2 创建Rate对象
    rate = rospy.Rate(1) # 1Hz
    # 4.3 循环发布数据
    while not rospy.is_shutdown():
        pub.publish(people)
        rospy.loginfo("发布的消息：%s, %d, %f", people.name, people.age, people.height)
        rate.sleep()