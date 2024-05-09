# swarm_robots_ROS2

## Step 1: Create a Launch File for the Robot and the World

* Create a package for `robot_description` and add a `/models` directory.
* Download [JupyterLab Editor](https://jupyterlab-urdf.readthedocs.io/en/latest/use_editor.html) for JupyterLab.
* Create a robot in URDF format:
    * The robot file must have a link called "world" or "base_link", and the chassis shall be fixed to it.
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
