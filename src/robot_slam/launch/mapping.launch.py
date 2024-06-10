""" A launch file for slam toolbox used for mapping the environment"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    saved_map_path = LaunchConfiguration('saved_map_path')
    use_saved_map = LaunchConfiguration('use_saved_map')

    default_map_path = PathJoinSubstitution(
        [FindPackageShare("robot_slam"), 'maps', 'boxes_map_serial']
    )
    
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare("robot_slam"), 'config', 'slam_toolbox_params.yaml']
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='use simulation/Gazebo clock'
    )
    
    use_saved_map_arg = DeclareLaunchArgument(
        'use_saved_map',
        default_value='false',
        description='Whether to use a saved map or not'
    )

    saved_map_path_arg = DeclareLaunchArgument(
        'saved_map_path',
        default_value=default_map_path,
        description='Path to the saved map'
    )
    
    slam_node_saved_map = Node(
        condition=IfCondition(use_saved_map),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time, 'map_file_name': saved_map_path}],
    )

    slam_node_no_map = Node(
        condition=UnlessCondition(use_saved_map),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time, 'map_file_name': ''}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        saved_map_path_arg,
        use_saved_map_arg,
        slam_params_file_arg,
        slam_node_saved_map,
        slam_node_no_map
    ])

if __name__ == '__main__':
    generate_launch_description()
