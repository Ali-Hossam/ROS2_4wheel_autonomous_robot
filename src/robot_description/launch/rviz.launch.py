import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
  # Get package and launch directory
  bringup_dir = get_package_share_directory('robot_description')
  launch_dir = os.path.join(bringup_dir, 'launch')

  # RViz configuration file argument
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
      'rviz_config_file',
      default_value=os.path.join(bringup_dir, 'rviz', 'config.rviz'),
      description='Full path to the RViz config file to use'
  )

  # RViz Node
  rviz_cmd = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config_file],
      output='screen',
      parameters=[{'use_sim_time': True}]
  )

  # Launch robot_state_publisher and joint_state_publisher
  use_joint_state_pub = LaunchConfiguration('use_joint_state_publisher', 
                                            default='False')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_publisher', 
                                            default='False')

  robot_launch_path = os.path.join(launch_dir, 'robot.launch.py')
  robot_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(robot_launch_path),
      launch_arguments={'use_joint_state_publisher': use_joint_state_pub,
                        'use_robot_state_publisher':use_robot_state_pub}.items()
  )

  # Create the launch description
  ld = LaunchDescription()

  # Add RViz configuration argument
  ld.add_action(declare_rviz_config_file_cmd)

  # Add RViz Node
  ld.add_action(rviz_cmd)

  # Add robot launch file
  ld.add_action(robot_launch)

  return ld
