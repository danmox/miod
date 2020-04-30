import roslaunch
import rospy
from trajectories import run_custom_trajectory, take_off

import rosparam

# run roscore localy
# running the script with poython2 ./name_of_the_script.py
# requires installation of package ros-melodic-ros-comm and sourcing setup.bash
# packacge requires rebuilding catkin and running rosdep install -i -y --from-paths .

package = 'intel_aero_demos'
nav_node = 'gazebo_vel_nav_nodelet'
# nav_nod= 'vel_nav_nodelet'
dron_name = 'aero'
#dron_name = 'nuc'
ciclos=3

launch = roslaunch.scriptapi.ROSLaunch()
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

cli_args1 = ['intel_aero_demos', '2t3n_PingPong.launch']
cli_args2 = ['intel_aero_demos', 'network_planner_node', 'connectivity_network_planner']

#rosparam._rosparam_cmd_list("rosparam load cfg/2t3n_PingPong_patrol.yaml")

roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
launch.parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)#, is_core =True)
launch.start()

rospy.sleep(2)

node = roslaunch.core.Node(cli_args2[0], cli_args2[1], namespace='/',name=cli_args2[2])
process = launch.launch(node)
rospy.init_node('trajectory_node')
rospy.sleep(2)

id_list= rospy.get_param('/task_agent_ids')

#for id in id_list:
#    client = take_off(id, dron_name, nav_node, 3.0)
#    client.wait_for_result()

for j in range(ciclos):
    for id in id_list:
        client = run_custom_trajectory(id, dron_name, nav_node)
        #client.wait_for_result()


#rospy.sleep(80)


#launch.stop()
