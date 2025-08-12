/*
    FILE: lidar_2d_to_3d_converter.cpp
    ----------------------------------
    Convert 2D LiDAR scan to 3D point cloud for TurtleBot3
*/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Lidar2DTo3DConverter
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher pointcloud_pub_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    std::string input_scan_topic_;
    std::string output_pointcloud_topic_;
    std::string frame_id_;
    int height_layers_;
    double height_increment_;
    double z_offset_;

public:
    Lidar2DTo3DConverter() : tf_listener_(tf_buffer_)
    {
        // Get parameters
        nh_.param<std::string>("input_scan_topic", input_scan_topic_, "/scan");
        nh_.param<std::string>("output_pointcloud_topic", output_pointcloud_topic_, "/pointcloud");
        nh_.param<std::string>("frame_id", frame_id_, "base_scan");
        nh_.param<int>("height_layers", height_layers_, 5);
        nh_.param<double>("height_increment", height_increment_, 0.2);
        nh_.param<double>("z_offset", z_offset_, 0.1);
        
        // Initialize subscribers and publishers
        scan_sub_ = nh_.subscribe(input_scan_topic_, 1, &Lidar2DTo3DConverter::scanCallback, this);
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_pointcloud_topic_, 1);
        
        ROS_INFO("2D LiDAR to 3D Point Cloud Converter initialized");
        ROS_INFO("Input topic: %s", input_scan_topic_.c_str());
        ROS_INFO("Output topic: %s", output_pointcloud_topic_.c_str());
        ROS_INFO("Height layers: %d", height_layers_);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        // Create point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        
        // Convert 2D scan to 3D point cloud with multiple height layers
        for (int layer = 0; layer < height_layers_; layer++)
        {
            double z_height = z_offset_ + layer * height_increment_;
            
            for (size_t i = 0; i < scan_msg->ranges.size(); i++)
            {
                float range = scan_msg->ranges[i];
                
                // Check if range is valid
                if (range < scan_msg->range_min || range > scan_msg->range_max || std::isnan(range) || std::isinf(range))
                {
                    continue;
                }
                
                // Calculate angle
                float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                
                // Convert to Cartesian coordinates
                pcl::PointXYZ point;
                point.x = range * cos(angle);
                point.y = range * sin(angle);
                point.z = z_height;
                
                cloud->points.push_back(point);
            }
        }
        
        // Add some random noise to make it more realistic
        addNoise(cloud);
        
        // Convert to ROS message
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud, output_msg);
        output_msg.header = scan_msg->header;
        output_msg.header.frame_id = frame_id_;
        
        // Publish point cloud
        pointcloud_pub_.publish(output_msg);
    }

private:
    void addNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // Add small random noise to make point cloud more realistic
        for (auto& point : cloud->points)
        {
            point.x += (rand() % 100 - 50) * 0.001; // ±5cm noise
            point.y += (rand() % 100 - 50) * 0.001;
            point.z += (rand() % 50 - 25) * 0.001;  // ±2.5cm noise in z
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_2d_to_3d_converter");
    
    Lidar2DTo3DConverter converter;
    
    ros::spin();
    
    return 0;
}