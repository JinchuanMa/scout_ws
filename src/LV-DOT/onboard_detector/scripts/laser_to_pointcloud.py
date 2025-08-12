#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

class LaserToPointCloud:
    def __init__(self):
        rospy.init_node('laser_to_pointcloud_converter')
        
        # 订阅LaserScan
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # 发布PointCloud2
        self.pc_pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=10)
        
        rospy.loginfo("LaserScan到PointCloud2转换器已启动")
    
    def scan_callback(self, scan):
        # 创建点云消息
        points = []
        
        for i, range_val in enumerate(scan.ranges):
            if scan.range_min <= range_val <= scan.range_max and not math.isinf(range_val):
                # 计算角度
                angle = scan.angle_min + i * scan.angle_increment
                
                # 转换为笛卡尔坐标
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                z = 0.0  # 2D激光雷达设为0
                
                points.append([x, y, z])
        
        if points:  # 只有在有点的时候才发布
            # 创建PointCloud2消息
            header = std_msgs.msg.Header()
            header.stamp = scan.header.stamp
            header.frame_id = scan.header.frame_id  # 使用scan的原始frame_id
            
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
            ]
            
            pc_msg = pc2.create_cloud(header, fields, points)
            self.pc_pub.publish(pc_msg)
            
            # 调试信息
            rospy.loginfo_throttle(5, f"发布了 {len(points)} 个点，frame_id: {scan.header.frame_id}")

if __name__ == '__main__':
    try:
        converter = LaserToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass