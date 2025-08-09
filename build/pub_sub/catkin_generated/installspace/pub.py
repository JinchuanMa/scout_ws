#! /usr/bin/env python3

# 导包
import rospy 
# 导入和消息载体相关的包，ros内置了string类型的数据
from std_msgs.msg import String # 发布消息的类型

"""
使用python实现消息发布流程：
    step 1.导包
    step 2.初始化ros节点
    step 3.创建发布者对象
    step 4.编写发布逻辑，并发布数据
"""

# 程序主入口
if __name__ == "__main__":
    # step 2.初始化ros节点
    rospy.init_node("erGouZi") # 传入节点名称

    # step 3.创建发布者对象
    pub = rospy.Publisher("car", String, queue_size=10) #name: car话题名称, data_class: Any, queue_size: Any 超过这个长度，之前的消息会被覆盖
    
    # step 4.编写发布逻辑，并发布数据
    # 创建发布数据
    msg = String() # 创建一个被发布的数据类型

    # 制定发布频率
    rate = rospy.Rate(1) # 1Hz

    # 设置计数器
    count = 0

    # 休眠3秒钟，等注册完毕，防止丢失数据,因为发布者需要在master里面注册，注册过程中，可能消息就发出去了，还没注册完，订阅者收不到信息
    # 解决办法，确保注册完毕，然后再去发布数据，所以发布前休眠3秒
    rospy.sleep(3)

    # 使用循环发布数据
    while not rospy.is_shutdown():
        count += 1
        msg.data = "hello" + str(count) # 给msg赋值
        # 发布数据
        pub.publish(msg)
        # 加个日志输出
        rospy.loginfo("发布的数据:%s", msg.data)

        rate.sleep() # 启动了休眠功能，每隔1秒钟(1Hz)醒过来
