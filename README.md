# intel_aero

A collection of packages for simulating and operating the [Intel Aero](https://software.intel.com/en-us/aero).

## intel_aero_description

A package containing description files (urdf, stl, mesh, etc.) defining the physical structure of the Aero and its sensors. While no description files exist for the Intel Aero, a similar quadrotor model has been configured with the same sensor suite.

## intel_aero_gazebo

A package for simulating the Intel Aero. The simulation is comprised of rviz for displaying the robot model and sensor streams, Gazebo for simulating physics and the environment, and PX4 software-in-the-loop.

For instructions on setting up the simulation see the [wiki](https://gitlab.sitcore.net/dcist/intel_aero/wikis/Legacy-PX4-Installation).

## intel_aero_navigation

A simple navigation action server with obstacle avoidance.

## network_status

A way of visualizing the channel rate between agents in a mobile autonomous network in rviz.