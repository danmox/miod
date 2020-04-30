import roslaunch
import rospy

# running the script with poython2 ./name_of_the_script.py
# requires installation of package ros-melodic-ros-comm and sourcing setup.bash
# packacge requires rebuilding catkin and running rosdep install -i -y --from-paths .
package = 'intel_aero_demos'
executable = 'line'

launch = roslaunch.scriptapi.ROSLaunch()

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid)
environment = ['intel_aero_experiments', 'intel_aero_live.launch', 'dest_arg:="10.42.0.1"', 'server:=false', 'srv:="10.42.0.1"', 'robot_ns:=nuc4',
               'id:=4', 'remote:=true']
roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(environment)[0]
roslaunch_args = environment[2:]
launches = [(roslaunch_file, roslaunch_args)]


trajectory = ['intel_aero_demos', 'line_demo', '_nav_nodelet:=px4_waypoint_navigation_nodelet']
rosrun_file = roslaunch.rlutil.resolve_launch_arguments(trajectory)
rosrun_args = trajectory[2:]

launch.parent = roslaunch.parent.ROSLaunchParent(uuid, launches,is_core=False)

launch.start()

rospy.sleep(20)

node = roslaunch.core.Node(rosrun_file, args=rosrun_args)
process = launch.launch(node)

rospy.sleep(10)
print(process.is_alive())

# 3 seconds later
launch.stop()
