# swarm_robots_ROS2

## Step 1: Create a Launch File for the Robot and the World

* Create a package for `robot_description` and add a `/models` directory.
* Download [JupyterLab Editor](https://jupyterlab-urdf.readthedocs.io/en/latest/use_editor.html) for JupyterLab.
* Create a robot in URDF format:
    * You need to adjust the orientation of the robot's components in your URDF/Xacro file so that the x-axis points forward, the y-axis points sideways (left), and the z-axis points up.
    * The robot file must have a dummy link called anything but "world" (as naming it world causes a bug if the model is opened in gazebo), and the chassis shall be fixed to it.
    * Origin of the wheels (x, y, z) and (r, p, y) shall be defined in the joint tag.
* To test the robot URDF model:
    * Publish the robot links into `/robot_description` topic using the command line:
        ```bash
        ros2 run robot_state_publisher robot_state_publisher BMW.urdf
        ```
    * Run RViz2 using the command line:
        ```bash
        ros2 run rviz2 rviz2
        ```
    * Control joints using the command line:
        ```bash
        ros2 run joint_state_publisher_gui joint_state_publisher_gui
        ```
* Create a launch file for rviz which should contain:
  1. Nodes : (robot_state_publisher, Joint_state_publisher, rivz)
  2. Launch default args : (rviz_config_dir, robot_urdf_dir, etc)

> URDF Files can be checked using the following command : `check_urdf MODEL.urdf`

## Step 2: Run the model in gazebo
   * Using Xacro in the urdf file can cause problems, therefore we can convert it to urdf and use it directly with the following command `xacro model.urdf.xacro > model.urdf`
   * To run the model in gazebo with Command Lines do the following :
        * Publish the robot to robot_state_publisher using the previous rviz launch file
          ```bash
             ros2 launch robot_description rviz.launch.py use_rviz:=false use_joint_state_pub:=false
          ```
        * Run gazebo using ros2
          ```bash
             ros2 launch gazebo_ros gazebo.launch.py
          ```
        * Spawn the robot inside gazebo
          ```bash
             ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity ROBOT_NAME
          ```
     * Alternatively, previous commands can be wrapped into a launch file

## Step 3: Control the robot
* The first step in controlling the robot is to add `diff_drive` plugin to the xacro file of the robot. To control a 4 wheel robot we need to use [skid_steer_drive](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Skid-Steer-drive) plugin.
   * > Make sure that max_wheel_torque and max_wheel_acceleration values are appropriate

## Step 4: Add sensors
1. Lidar sensor using laser control plugin of libgazebo_ros_ray_sensor
2. Camera sensor

## Step 5: Add a launch file for both gazebo and rviz (launch_sim.launch)
Create a custom world for gazebo in a separate directory and launch it using the new launch file with the following command
```bash
ros2 launch robot_description launch_sim.launch.py world:=./src/robot_description/models/myWorld/boxes_world.sdf
```

## Step 6: Integrate ros2_control package with new launch file(optional)
> ⚠️ **Warning:** 
> I experienced synchronization problems with the `ros2_control` package when using Gazebo and RViz together. In RViz, the motion was based on joint input commands rather than the robot's actual state in Gazebo. This issue becomes apparent when you place obstacles in front of the robot in Gazebo and move it using `teleop_twist_keyboard`. The robot stops in Gazebo but continues moving in RViz.

- To use teleop_twist_keyboard after adding ros2_control use the following command that remaps the topic (cmd_vel) into another topic:
  `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped`


## Step 7: Add [SLAM](https://husarion.com/tutorials/ros2-tutorials/8-slam/#localize-robot)
Simultaneous Localization and Mapping (SLAM) is a technique used in robotics and computer vision to create a map of an unknown environment while simultaneously keeping track of the robot's location within that environment. It involves the robot using sensor data to build a map of its surroundings and determine its own position relative to that map in real-time.

### Mapping
* Create a new package for SLAM ("robot_slam")
* Add a launch file for mapping the environment and a param file which shall be launched via the following command : `ros2 launch robot_slam amcl.launch.py`
* After creating a map save it with SlamToolboxPlugin (Panels - Add new panel - SlamToolboxPlugin)

### Localization
* AMCL: The AMCL node is used for localization, which means it estimates the robot's pose based on the map and sensor data (e.g., LIDAR). It does not control or move the robot. We previously had the odom frame which allows us to compute the robot position based on the joint movements, now we want a map frame that computes the robot's pose based on the map and the sensor data.

* After saving the map Create a launch file with nodes for map server, amcl and nav_manager (or lifecycle bringup).

> Note that the map_server also only loads the map one time, so I recommend having rviz2 open with the map topic added before you launch the map_server.

```bash
ros2 launch robot_description launch_sim.launch.py world:=./src/robot_description/models/myWorld/boxes_world.sdf use_sim_time:=true rviz_config_file:=./src/robot_description/rviz/sim_map.config.rviz

ros2 launch robot_slam amcl.launch.py use_sim_time:=true
```
## Step 8: Add Navigation (Path Planning & Control)
we will be using nav2 package. Our controller is expecting the command velocities on /diff_cont/cmd_vel_unstamped but nav2 uses /cmd_vel, so we will be using a node called twist_mux which takes a bunch of twist topics and it is going to multiplix them into a single topic.

* Create a new robot_nav package
* Add twist_mux.yaml config file
* Copy params file from `/opt/ros/humble/share/nav2_bringup/params/navigation_launch.py` into our config folder and modify and base_frame parameter name into dummy_link as defined in our robot urdf file.

* Copy launch file from `/opt/ros/humble/share/nav2_bringup/launch/nav2_params.yaml`
* Add launch directory to CMAKELists file
## Problems & Solutions
1. The robot moves forward in gazebo and sideward in rviz
   - Reason : The Robot urdf model doesn't have x-axis points forward and y-axis points sideward
   - Solution : Manually change xacro or urdf model so that x-axis points forward

## Resources
1. https://docs.nav2.org/getting_started/index.html
2. https://husarion.com/tutorials/ros2-tutorials/8-slam/#rviz-visualization
