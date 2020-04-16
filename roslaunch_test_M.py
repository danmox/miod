import roslaunch
import rospy

from trajectory import run_trajectory
from trajectory import get_data


#run roscore localy
#running the script with poython2 ./name_of_the_script.py
#requires installation of package ros-melodic-ros-comm and sourcing setup.bash
#packacge requires rebuilding catkin and running rosdep install -i -y --from-paths .
package = 'intel_aero_demos'
#executable = 'trajectory'
nav_node= 'gazebo_vel_nav_nodelet'
#nav_nod= 'vel_nav_nodelet'
dron_name ='aero'
launch = roslaunch.scriptapi.ROSLaunch()

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#roslaunch.configure_logging(uuid)
cli_args1 = ['intel_aero_demos', 'simple_spawn_M.launch']
roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args1)

launch.parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

launch.start()

rospy.sleep(20)

id_list, trajectory_list = get_data("dictex.yaml")

for id in id_list:
    rospy.set_param('/'+ dron_name + id +'/'+ nav_node +'/desired_trajectory',trajectory_list[id_list.index(id)])
    client=run_trajectory(id,dron_name,nav_node)
    client.wait_for_result()

rospy.sleep(10)
# 3 seconds later
#launch.parent.shutdown()
launch.stop()