import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # get launch directory
    bringup_dir = get_package_share_directory('robot_description')

    # launch gazebo
    gazebo_launch_path = os.path.join(bringup_dir, 'launch', 'gazebo.launch.py')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )
    
    # launch rviz
    rviz_config_file_path = os.path.join(bringup_dir, 'rviz', 'sim_config.rviz')
    rviz_config_file_arg = LaunchConfiguration('rviz_config_file',
                                               default=rviz_config_file_path)
    
    rviz_launch_path = os.path.join(bringup_dir, 'launch', 'rviz.launch.py')
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path),
        launch_arguments={'rviz_config_file':rviz_config_file_arg}.items()
    )

    # add ros control spawner nodes
    diff_drive_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['diff_cont']
    )
    joint_broad_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['joint_state_broadcaster']
    )
    
    # add delays for all node other than gazebo, as gazebo takes sometime to launch
    
    return LaunchDescription([
      gazebo_launch,
      TimerAction(period=4.0, actions=[rviz_launch]),
      TimerAction(period=5.0, actions=[diff_drive_spawner, joint_broad_spawner])
      ])