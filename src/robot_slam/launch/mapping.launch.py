from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  slam_params_file = LaunchConfiguration('slam_params_file')
  use_sim_time = LaunchConfiguration('use_sim_time')
  map_file = PathJoinSubstitution(
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
  
  slam_node = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam',
    parameters=[slam_params_file, {'use_sim_time':use_sim_time,
                                   'map_file_name':map_file}],
  )
  
  return LaunchDescription([use_sim_time_arg,
                            slam_params_file_arg,
                            slam_node])