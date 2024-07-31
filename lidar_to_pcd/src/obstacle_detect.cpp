#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/features/normal_3d.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/visualization/cloud_viewer.h>

class ObstacleDetectionNode : public rclcpp::Node {
public:
    ObstacleDetectionNode() : Node("obstacle_detection_node"), ground_pcl_(new pcl::PointCloud<pcl::PointXYZ>) {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser_map", 10,
            std::bind(&ObstacleDetectionNode::laserScanCallback, this, std::placeholders::_1));

        // Create a KD tree for clustering
        tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    }

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Convert LaserScan to point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            pcl::PointXYZ point;
            point.x = msg->ranges[i] * std::cos(msg->angle_min + msg->angle_increment * i);
            point.y = msg->ranges[i] * std::sin(msg->angle_min + msg->angle_increment * i);
            point.z = 0.0; // Assuming 2D laser scan, adjust for 3D if necessary
            cloud->push_back(point);
        }

        // Voxel grid downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f); // Adjust leaf size as needed
        voxel_grid.filter(*cloud_downsampled);

        // SAC segmentation using RANSAC to find ground plane
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.1); // Adjust threshold as needed
        seg.setInputCloud(cloud_downsampled);
        seg.segment(*inliers, *coefficients);

        // Extract ground points
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_downsampled);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*ground_pcl);

        // Remove ground points from original cloud
        extract.setNegative(true);
        extract.filter(*cloud_downsampled);

        // Perform clustering for obstacles
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.2); // Adjust tolerance as needed
        ec.setMinClusterSize(10);    // Adjust min cluster size as needed
        ec.setMaxClusterSize(25000); // Adjust max cluster size as needed
        ec.setSearchMethod(tree_);
        ec.setInputCloud(cloud_downsampled);
        ec.extract(cluster_indices);

        // Visualization and height calculation
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Obstacle Clusters"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "original_cloud");

        int cluster_id = 0;
        for (auto indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (auto index : indices.indices) {
                cluster->points.push_back(cloud_downsampled->points[index]);
            }
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;

            // Calculate height of the obstacle cluster
            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
            float height = max_pt.z - min_pt.z;
            std::cout << "Obstacle " << cluster_id << " height: " << height << " meters" << std::endl;

            // Visualize obstacle cluster
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color(cluster, 0, 255, 0); // Green color
            viewer->addPointCloud<pcl::PointXYZ>(cluster, cluster_color, "obstacle_cluster_" + std::to_string(cluster_id));
            ++cluster_id;
        }

        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_pcl_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
