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
          

## Step 5: Integrate ros2_control package (optional) and create a launch file for both gazebo and rviz
- To use teleop_twist_keyboard after adding ros2_control use the following command that remaps the topic (cmd_vel) into another topic:
  `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped`

### Problems
1. Motion in Gazebo is very slow relative to rviz
   - Reason : Gazebo is a physics simulation so it accounts for mass, friction, etc however rviz is a visualization tool. 
2. The robot moves forward in gazebo and sideward in rviz
   - Reason : The Robot urdf model doesn't have x-axis points forward and y-axis points sideward
   - Solution : Manually change xacro or urdf model so that x-axis points forward
3. a complete rotation in rviz != a complete rotation in gazebo
4. When the robot collide with an object, the robot stops in gazebo but keeps moving in rviz
   - Reason : Joint_states topic shows that robot is moving even after collision