from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_params_file = LaunchConfiguration("amcl_params_file")
    map_file = LaunchConfiguration("map_file")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation/Gazebo clock"
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([FindPackageShare("robot_slam"), "maps", "boxes_map.yaml"]),
        description="Full path to the yaml map file",
    )

    amcl_params_file_arg = DeclareLaunchArgument(
        "amcl_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("robot_slam"), "config", "amcl.config.yaml"]
        ),
        description="Full path to the ROS2 parameters file to use for the amcl node",
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_file}],
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[amcl_params_file, {"use_sim_time": use_sim_time}],
    )

    nav_manager = Node(
        package="nav2_util",
        executable="lifecycle_bringup",
        name="lifecycle_bringup",
        arguments=["map_server", "amcl"],
        output="screen"
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            amcl_params_file_arg,
            map_file_arg,
            map_server_node,
            amcl_node,
            nav_manager,
        ]
    )