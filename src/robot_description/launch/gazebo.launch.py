'''
This is a launch file that launches Gazebo and spawn the robot inside..
'''

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  # get launch directory
  bringup_dir = get_package_share_directory('robot_description')
  
  # launch configuration variables specific to simulation
  urdf_file = LaunchConfiguration('urdf_file')
  
  use_sim_time = LaunchConfiguration('use_sim_time', default='True')
  
  # declare launch arguments (arguments that can be changed from CL)
  declare_urdf_cmd = DeclareLaunchArgument(
    'urdf_file',
    default_value=os.path.join(bringup_dir,
                               'models',
                               'myCar',
                               'BMW.urdf'),
    description='urdf file path')
  
  declare_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='whether to use sim_time'
  )
  
  # declare Nodes
  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{'use_sim_time':use_sim_time}],
    arguments=[urdf_file]
  )
  
  spawn_robot_cmd = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-topic', 'robot_description', '-entity', 'BMW'],
    output='screen'
  )
  
  # Include Gazebo launch file
  gazebo_launch_file = os.path.join(
    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

  gazebo_params_file_dir = os.path.join(bringup_dir, 'config', 'gazebo_params.yaml')

  start_gazebo_launch_file = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([gazebo_launch_file]),
      launch_arguments={'params_file' : gazebo_params_file_dir}.items()
  )
  
  # create the launch description
  ld  = LaunchDescription()
  
  ld.add_action(declare_sim_time_cmd)
  ld.add_action(declare_urdf_cmd)
  
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_gazebo_launch_file)
  ld.add_action(spawn_robot_cmd)


  return ld