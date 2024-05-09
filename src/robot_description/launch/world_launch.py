from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
  pkg_robot_description = get_package_share_directory('robot_description')
  
  gazebo_models_path = os.path.join(pkg_robot_description, 'models', 'gazebo-11')
  
  # gazebo launch
  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    )
  )
  
  return launch.LaunchDescription([
    DeclareLaunchArgument(
      'world',
      default_value=[os.path.join(gazebo_models_path, 'worlds', 'willowgarage.world'), ''],
      description="SDF world file"),
    gazebo
    
  ])
