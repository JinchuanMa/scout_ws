#!/usr/bin/env python3
"""
检查Scout机器人的话题，特别是里程计话题
用于确定fake_detector应该监听哪个话题
"""

import rospy
import subprocess
import sys

def get_topic_list():
    """获取当前所有ROS话题"""
    try:
        result = subprocess.run(['rostopic', 'list'], 
                              capture_output=True, text=True, check=True)
        return result.stdout.strip().split('\n')
    except subprocess.CalledProcessError as e:
        print(f"Error getting topic list: {e}")
        return []

def check_topic_type(topic):
    """检查话题的消息类型"""
    try:
        result = subprocess.run(['rostopic', 'type', topic], 
                              capture_output=True, text=True, check=True)
        return result.stdout.strip()
    except subprocess.CalledProcessError:
        return "Unknown"

def main():
    print("=== Scout机器人话题检查工具 ===\n")
    
    topics = get_topic_list()
    
    print("1. 所有可用话题:")
    for topic in topics:
        print(f"   {topic}")
    
    print("\n2. 里程计相关话题:")
    odom_topics = [topic for topic in topics if 'odom' in topic.lower()]
    
    if not odom_topics:
        print("   未找到包含'odom'的话题")
        print("   检查是否有其他位置信息话题...")
        
        # 检查其他可能的位置话题
        pose_topics = [topic for topic in topics if any(keyword in topic.lower() 
                      for keyword in ['pose', 'position', 'state'])]
        
        if pose_topics:
            print("   找到位置相关话题:")
            for topic in pose_topics:
                msg_type = check_topic_type(topic)
                print(f"     {topic} ({msg_type})")
    else:
        for topic in odom_topics:
            msg_type = check_topic_type(topic)
            print(f"   {topic} ({msg_type})")
    
    print("\n3. Gazebo模型状态话题:")
    gazebo_topics = [topic for topic in topics if 'gazebo' in topic.lower()]
    for topic in gazebo_topics:
        msg_type = check_topic_type(topic)
        print(f"   {topic} ({msg_type})")
    
    print("\n4. 建议配置:")
    if odom_topics:
        nav_odom = [t for t in odom_topics if 'nav_msgs/Odometry' == check_topic_type(t)]
        if nav_odom:
            print(f"   推荐使用: {nav_odom[0]}")
            print(f"   在fake_detector_param.yaml中设置:")
            print(f"   odom_topic: \"{nav_odom[0]}\"")
        else:
            print(f"   可以尝试: {odom_topics[0]}")
    else:
        print("   请检查Scout机器人是否正确启动")
        print("   或者查看是否有其他位置信息话题可用")

if __name__ == "__main__":
    main()