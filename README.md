# swarm_robots_ROS2

## Step 1. Create a launch file for the robot and the world
* Create a pkg for robot_description and add /models directory.
* Download jupyterlab-editor<https://jupyterlab-urdf.readthedocs.io/en/latest/use_editor.html> for jupyterlab
* Create a robot in urdf format
    * The robot file must have a link called "world" or "base_link", and the chassis shall be fixed to it.
    * Origin of the wheels (x,y,z) (r,p,y) shall be defined in the joint tag.
