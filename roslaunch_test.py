import roslaunch
import rospy
#running the script with poython2 ./name_of_the_script.py
#requires installation of package ros-melodic-ros-comm and sourcing setup.bash
#packacge requires rebuilding catkin and running rosdep install -i -y --from-paths .
package = 'intel_aero_demos'
executable = 'line'

launch = roslaunch.scriptapi.ROSLaunch()

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#roslaunch.configure_logging(uuid)
cli_args1 = ['intel_aero_demos', 'line.launch']
roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
cli_args2 = ['intel_aero_demos', 'line_demo', '_nav_nodelet:=gazebo_vel_nav_nodelet']

launch.parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

launch.start()

rospy.sleep(20)

node = roslaunch.core.Node(cli_args2[0], cli_args2[1], args=cli_args2[2])
process = launch.launch(node)
rospy.sleep(10)
# 3 seconds later
launch.shutdown()