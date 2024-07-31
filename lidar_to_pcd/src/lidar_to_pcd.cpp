#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <chrono>
#include <pcl/segmentation/segment_differences.h>

class ObstacleDetectionNode : public rclcpp::Node {
public:
    ObstacleDetectionNode() : Node("obstacle_detection_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/Laser_map", 10, std::bind(&ObstacleDetectionNode::lidar_callback, this, std::placeholders::_1));
        
        // Initialize parameters
        this->declare_parameter<double>("distance_threshold", 0.01);
        this->get_parameter("distance_threshold", distance_threshold_);
        this->declare_parameter<std::string>("pcd_file_name", "processed_data.pcd");
        this->get_parameter("pcd_file_name", pcd_file_name_);
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::cout << "ll" << std::endl;
        // Convert PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *pcl_pc);

        // Process point cloud (replace this with your detection code)
        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_pc(new pcl::PointCloud<pcl::PointXYZ>());
        // Example processing: segment differences
        pcl::SegmentDifferences<pcl::PointXYZ> segment_diff;
        segment_diff.setInputCloud(pcl_pc);
        segment_diff.setTargetCloud(previous_pc_);
        segment_diff.setDistanceThreshold(distance_threshold_); // Use parameterized threshold
        segment_diff.segment(*processed_pc);

        // Update previous point cloud for next iteration
        previous_pc_ = pcl_pc;

        // Save processed data to PCD file
        pcl::io::savePCDFileASCII(pcd_file_name_, *processed_pc);
        RCLCPP_INFO(this->get_logger(), "Processed data saved to %s", pcd_file_name_.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_pc_;
    double distance_threshold_;
    std::string pcd_file_name_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetectionNode>();
    rclcpp::spin(node);
    return 0;
}
