'''
This is a launch file that launches rviz, robot_state_publisher and
joint_state_publisher.
'''

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  # get launch directory
  bringup_dir = get_package_share_directory('robot_description')
  launch_dir = os.path.join(bringup_dir, 'launch')
  
  # launch configuration variables specific to simulation
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  urdf_file = LaunchConfiguration('urdf_file')
  use_sim_time = LaunchConfiguration('use_sim_time', default='True')
  
  # declare launch arguments (arguments that can be specified from command line)
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    'rviz_config_file',
    default_value=os.path.join(bringup_dir, 'rviz', 'config.rviz'),
    description='Full path to the RVIZ config file to use')

  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    'use_robot_state_pub',
    default_value='True',
    description='whether to start the robot state publisher')
  
  declare_use_robot_joint_state_pub_cmd = DeclareLaunchArgument(
    'use_joint_state_pub',
    default_value='True',
    description='whether to start the joint state publisher')
  
  declare_use_rviz_cmd = DeclareLaunchArgument(
    'use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
  
  declare_urdf_cmd = DeclareLaunchArgument(
    'urdf_file',
    default_value=os.path.join(bringup_dir,
                               'models',
                               'myCar',
                               'BMW.urdf'),
    description='urdf file path')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    default_value='False',
    description='whether to use sim_time'
  )
  # declare Nodes
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{'use_sim_time':use_sim_time}],
    arguments=[urdf_file]
  )
  
  start_joint_state_publisher_cmd = Node(
    condition=IfCondition(use_joint_state_pub),
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    output='screen',
    parameters=[{'use_sim_time':use_sim_time}],
    arguments=[urdf_file]
  )
  
  rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen'
  )
  
  # create the launch description
  ld = LaunchDescription()
  
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_urdf_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)
  ld.add_action(declare_use_robot_joint_state_pub_cmd)
  ld.add_action(declare_use_rviz_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  
  # add nodes
  ld.add_action(start_joint_state_publisher_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(rviz_cmd)
  
  return ld