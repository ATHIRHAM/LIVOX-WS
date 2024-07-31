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
        self.pcd_path = "/home/athir/livox_ws/src/FAST_LIO_ROS2/FAST_LIO/PCD/result.pcd"  # Change this path to where your PCD files are saved
        self.pcd = o3d.io.read_point_cloud(self.pcd_path)
        self.get_logger().info(f"Loaded point cloud with {len(self.pcd.points)} points")

        # Create a timer to call the obstacle detection periodically
        self.timer = self.create_timer(10.0, self.execute_obstacle_detection)  # Adjust frequency as needed(depedning on the frequency defined in map_saver_node.py file)

        # Create a service client
        self.trigger_service_client = self.create_client(Trigger, '/trigger_obstacle_detection')
        self.wait_for_service_and_call()

    def wait_for_service_and_call(self):
        # Wait for the service to be available
        self.get_logger().info('Waiting for /trigger_obstacle_detection service...')
        self.trigger_service_client.wait_for_service(timeout_sec=10.0)
        self.call_obstacle_detection_service()  # Call the service immediately

    def filter_and_downsample(self, pcd):
        # Downsample the point cloud
        voxel_size = 0.03  # Size of the voxel grid, can be changed depending on the environment
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
            min_height = np.min(cluster_points[:, 2])
            max_height = np.max(cluster_points[:, 2])
            height = (max_height - min_height) * 100  # Height in centimeters

            # Create sphere at centroid
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.07)
            sphere.paint_uniform_color(cluster_colors[label][:3])  # Cluster color
            sphere.translate(centroid)
            vis.add_geometry(sphere)

            # Log cluster information
            self.get_logger().info(f"Obstacle Cluster {label}: Centroid: {centroid}, Height: {height} cm")
            # Print height of the cluster to console
            print(f"Cluster {label}: Height: {height} cm")

        # Run the visualizer
        vis.run()
        vis.destroy_window()

    def execute_obstacle_detection(self):
        # This method is called by the timer to execute obstacle detection periodically
        self.get_logger().info('Executing obstacle detection...')
        
        # Filter and downsample the point cloud
        filtered_pcd = self.filter_and_downsample(self.pcd)
        
        # Detect obstacles with meanshift
        self.detect_obstacles_with_meanshift(filtered_pcd)

    def call_obstacle_detection_service(self):
        # Log that the obstacle detection service is being called
        self.get_logger().info('Calling obstacle detection service...')
        # Create and send a request to the service
        request = Trigger.Request()
        future = self.trigger_service_client.call_async(request)
        future.add_done_callback(self.callback_obstacle_detection_service)

    def callback_obstacle_detection_service(self, future):
        try:
            # Retrieve the result from the service call
            response = future.result()
            self.get_logger().info(f'Obstacle detection service response: {response.message}')
        except Exception as e:
            # Log any error that occurs during the service call
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
