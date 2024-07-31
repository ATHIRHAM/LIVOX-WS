#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from std_srvs.srv import Trigger
import hdbscan
import matplotlib.pyplot as plt

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')

        # Load ground point cloud from PCD file
        self.pcd_path = "/home/athir/livox_ws/src/FAST_LIO_ROS2/FAST_LIO/PCD/result.pcd"
        self.pcd = o3d.io.read_point_cloud(self.pcd_path)
        self.get_logger().info(f"Loaded point cloud with {len(self.pcd.points)} points")
        filtered_pcd = self.filter_and_downsample(self.pcd)
        self.detect_obstacles_with_meanshift(filtered_pcd)
        # Call a service to trigger obstacle detection
        self.trigger_service_client = self.create_client(Trigger, '/trigger_obstacle_detection')

        # Attempt to call the service
        self.call_obstacle_detection_service()

    def filter_and_downsample(self, pcd):
        # Downsample the point cloud
        voxel_size = 0.03               # can be changed depending on the evironment
        downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        points = np.asarray(downsampled_pcd.points)

        # Filter points within specified ranges
        filtered_indices = np.logical_and(
            np.logical_and(points[:, 1] > -2.3, points[:, 1] < 2.3),
            np.logical_and(points[:, 2] > -0.34, points[:, 2] < 1)  #can be changed, once the actual height of the Mid-360 on the robot is determined with precision
        )
        filtered_points = points[filtered_indices]

        # Convert filtered numpy array back to Open3D point cloud
        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

        return filtered_pcd

    def detect_obstacles_with_meanshift(self, pcd):
        # Convert point cloud to NumPy array
        points = np.asarray(pcd.points)

        # Check if points array is empty
        if points.shape[0] == 0:
            self.get_logger().warning('No points in the point cloud after filtering. Skipping obstacle detection.')
            return

        # Perform HDBSCAN clustering
        clusterer = hdbscan.HDBSCAN(min_cluster_size=10, gen_min_span_tree=True)
        labels = clusterer.fit_predict(points)

        # Initialize Open3D visualizer
        vis = o3d.visualization.Visualizer()
        vis.create_window()

        # Add filtered point cloud to the visualizer
        vis.add_geometry(pcd)

        # Colors for clusters
        cluster_colors = plt.cm.jet(np.linspace(0, 1, len(np.unique(labels))))

        for label in np.unique(labels):
            if label == -1:  # Skip noise points
                continue
            cluster_points = points[labels == label]
            centroid = np.mean(cluster_points, axis=0)
            #calculate the height of the obstacle
            min_height = np.min(cluster_points[:, 2])
            max_height = np.max(cluster_points[:, 2])
            height = (max_height - min_height) * 100 

            # Create sphere at centroid
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.07)
            sphere.paint_uniform_color(cluster_colors[label][:3])  # Cluster color
            sphere.translate(centroid)
            vis.add_geometry(sphere)

            self.get_logger().info(f"Obstacle Cluster {label}: Centroid: {centroid}, Height: {height} cm")

        # Run the visualizer
        vis.run()
        vis.destroy_window()

def main(args=None):
    rclpy.init(args=args)
    obstacle_detection_node = ObstacleDetectionNode()
    try:
        rclpy.spin(obstacle_detection_node)
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Shutting down...")
    finally:
        if rclpy.ok():
            obstacle_detection_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
