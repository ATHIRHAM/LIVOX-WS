from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    node2= Node (
        package='lidar_to_pcd',
        executable='obstacle_detection',
        name= 'obstacle_detection',
        output='screen'
    )
   
    node =[node2]
    return LaunchDescription(node)
