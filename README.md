# infrastructure-on-demand

A collection of ROS packages for simulating and deploying mobile wireless infrastructure on demand.

# getting started (without PX4)

These instructions assume ROS has been installed and sourced in the current terminal. For more information on how to accomplish that visit the [ROS wiki](https://www.ros.org/install/). These packages have been thoroughly tested with ROS Kinetic but should work with newer versions as well. Most of the simulation examples are designed to run without PX4 software-in-the-loop (SITL) emulation which adds significant computational overhead and is known to be unstable when the simulation does not run in real time.

Make a new catkin workspace and clone the repo:
```bash
mkdir -p ~/ws_catkin/src
cd ~/ws_catkin/src
git clone -b devel git@github.com:alelab-upenn/infrastructure-on-demand.git
```

From the source directory, ensure all build dependencies are satisfied by running:
```bash
rosdep install -i -y --from-paths .
sudo apt-get install python-catkin-tools
```

Initialize the workspace and build it with catkin tools:
```bash
cd ~/ws_catkin
catkin init
catkin config --extend /opt/ros/<your-distro>
catkin build
```

Launch one of the simulation scenarios:
```bash
source devel/setup.bash
roslaunch intel_aero_demos 3_task_1_network.launch margin:=0.05
```

You should see an rviz window pop up showing 3 (task) quadrotors in a triangle formation and 1 (network) quadrotor in the interior. After the nodes have initialized the network quadrotor takes off and positions itself at the center of the triangle, the optimal configuration for this scenario. A margin of 0.05 has been manually specified on command line as a single network agent is unable to deliver the default margin of 0.15 set in the launch file.

