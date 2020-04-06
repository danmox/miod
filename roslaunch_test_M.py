import roslaunch
import rospy
from trajectory import run_trajectory
#run roscore localy
#running the script with poython2 ./name_of_the_script.py
#requires installation of package ros-melodic-ros-comm and sourcing setup.bash
#packacge requires rebuilding catkin and running rosdep install -i -y --from-paths .
package = 'intel_aero_demos'
#executable = 'trajectory'

launch = roslaunch.scriptapi.ROSLaunch()

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#roslaunch.configure_logging(uuid)
cli_args1 = ['intel_aero_demos', 'simple_spawn_M.launch']
roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args1)

launch.parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

launch.start()

rospy.sleep(20)
id_list=[1, 3, 5]
trajectory_list=[
    [[3,3,3],[3,3,0]],
    [[3,0,3],[3,0,0]],
    [[0,3,3],[0,3,0]]
]
rospy.set_param('/aero1/gazebo_vel_nav_nodelet/desired_trajectory',trajectory_list[0])
rospy.set_param('/aero3/gazebo_vel_nav_nodelet/desired_trajectory',trajectory_list[1])
rospy.set_param('/aero5/gazebo_vel_nav_nodelet/desired_trajectory',trajectory_list[2])

for i in id_list:
    run_trajectory(i)

rospy.sleep(10)
# 3 seconds later
#launch.parent.shutdown()
launch.stop()