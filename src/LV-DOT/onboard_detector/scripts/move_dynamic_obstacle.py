#!/usr/bin/env python3

import rospy
import math
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion

class DynamicObstacleMover:
    def __init__(self):
        rospy.init_node('dynamic_obstacle_mover', anonymous=True)
        
        # 等待Gazebo服务
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        self.time = 0
        rospy.loginfo("Dynamic obstacle mover started!")
        
    def move_obstacle(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            self.time += 0.1
            
            try:
                # 创建模型状态
                model_state = ModelState()
                model_state.model_name = 'dynamic_box_050_050_100'
                
                # 圆形运动路径
                center_x = -2.0
                center_y = 2.0
                radius = 1.5
                
                model_state.pose.position.x = center_x + radius * math.cos(self.time * 0.5)
                model_state.pose.position.y = center_y + radius * math.sin(self.time * 0.5)
                model_state.pose.position.z = 0.5
                
                # 设置朝向
                model_state.pose.orientation.x = 0
                model_state.pose.orientation.y = 0
                model_state.pose.orientation.z = 0
                model_state.pose.orientation.w = 1
                
                # 设置速度
                model_state.twist.linear.x = -radius * 0.5 * math.sin(self.time * 0.5)
                model_state.twist.linear.y = radius * 0.5 * math.cos(self.time * 0.5)
                model_state.twist.linear.z = 0
                
                # 发送状态更新
                self.set_state(model_state)
                
            except rospy.ServiceException as e:
                rospy.logwarn(f"Service call failed: {e}")
                
            rate.sleep()

if __name__ == '__main__':
    try:
        mover = DynamicObstacleMover()
        mover.move_obstacle()
    except rospy.ROSInterruptException:
        pass
