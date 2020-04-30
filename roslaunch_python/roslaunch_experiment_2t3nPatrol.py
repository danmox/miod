import roslaunch
import rospy
from trajectories import run_custom_trajectory, take_off

import rosparam

# run roscore localy
# running the script with poython2 ./name_of_the_script.py
# requires installation of package ros-melodic-ros-comm and sourcing setup.bash
# packacge requires rebuilding catkin and running rosdep install -i -y --from-paths .

package = 'intel_aero_demos'
nav_node = 'px4_waypoint_navigation_nodelet'
# nav_nod= 'vel_nav_nodelet'
dron_name = 'nuc'
#dron_name = 'nuc'
ciclos=3

launch = roslaunch.scriptapi.ROSLaunch()
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
environment = []
environment.append(['intel_aero_experiments', 'intel_aero_live.launch', 'server:=true', 'robot_ns:=nuc1', 'id:=1', 'remote:=true'])
environment.append(['intel_aero_experiments', 'intel_aero_live.launch', 'dest_arg:="10.42.0.1"', 'server:=false', 'srv:="10.42.0.1"', 'robot_ns:=nuc3',
               'id:=3', 'remote:=true'])
environment.append(['intel_aero_experiments', 'intel_aero_live.launch', 'server:=false', 'srv:="10.42.0.1"', 'robot_ns:=nuc4', 'id:=4', 'remote:=true'])
environment.append(['intel_aero_experiments', 'intel_aero_live.launch', 'server:=false', 'srv:="10.42.0.1"', 'robot_ns:=nuc6', 'id:=6', 'remote:=true'])
environment.append(['intel_aero_experiments', 'intel_aero_live.launch', 'server:=false', 'srv:="10.42.0.1"', 'robot_ns:=nuc7', 'id:=7', 'remote:=true'])
environment.append(['intel_aero_experiments', 'line_server.launch', 'param_file:=/cfg/2t3n_PingPong_patrol.yaml', 'remote:=true'])
launches = []
for i in environment:
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(i)[0]
    roslaunch_args = i[2:]
    launches.append((roslaunch_file,roslaunch_args))

launch.parent = roslaunch.parent.ROSLaunchParent(uuid, launches)#, is_core =True)
launch.start()

rospy.sleep(10)

rospy.init_node('trajectory_node')

rospy.sleep(2)

id_list= rospy.get_param('/task_agent_ids')

for id in id_list:
    client = take_off(id, dron_name, nav_node, 3.0)
    client.wait_for_result()
for j in range(ciclos):
    clients = []
    for id in id_list:
        client = run_custom_trajectory(id, dron_name, nav_node)
        clients.append(client)
    for client in clients:
        client.wait_for_result()

#rospy.sleep(80)


#launch.stop()
