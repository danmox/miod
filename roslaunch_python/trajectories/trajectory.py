import copy
import actionlib
import rospy
import yaml
from geometry_msgs.msg import Point, PoseStamped, Pose
from intel_aero_navigation.msg import WaypointNavigationAction, WaypointNavigationGoal
from functools import partial

def get_data(file_name):
    with open(file_name, 'r') as stream:
        data_loaded = yaml.safe_load(stream)
    id_list = []
    trajectory_list = []
    for x in data_loaded['wp_dict']:
        id_list.append(x)
        trajectory_list.append(data_loaded['wp_dict'][x])
    return id_list, trajectory_list


def fetch_param_warn(name):
    try:
        value = rospy.get_param(name)
        return value
    except KeyError:
        rospy.logwarn('failed to fetch param \'%s\' from parameter server; using a default value of %s' % (name))


def run_custom_trajectory(id, dname, node):
    name = '/' + dname + str(id) + '/' + node
    desired_trajectory = fetch_param_warn(name + '/desired_trajectory')
    waypoint_count = len(desired_trajectory)
    #
    # send goals (from /aero%id/params) to id quad
    #

    #rospy.init_node('trajectory_node')
    client = actionlib.SimpleActionClient(name, WaypointNavigationAction)
    rospy.loginfo('waiting for %s to start' % (name))
    client.wait_for_server()
    goal = WaypointNavigationGoal()
    goal.header.frame_id = 'world'
    goal.header.stamp = rospy.get_rostime()
    goal.end_action = WaypointNavigationGoal.HOVER

    for j in range(waypoint_count):
        pose = Pose()
        # pose =desired_trajectory[j]
        pose.position.x = desired_trajectory[j][0]
        pose.position.y = desired_trajectory[j][1]
        pose.position.z = desired_trajectory[j][2]
        pose.orientation.w = 1.0

        goal.waypoints += [copy.deepcopy(pose)]

        client.send_goal(goal)

    rospy.loginfo('waiting for nuc%s to complete' % (id))

    #client.wait_for_result()
    return client

def take_off(id, dname, node, altitud):
    agent_Pose = Pose()
    pose_rcv_msg = [False]
    name = '/' + dname + str(id) + '/' + node
    client = actionlib.SimpleActionClient(name, WaypointNavigationAction)
    rospy.loginfo('waiting for %s to start' % (name))
    client.wait_for_server()
    topic =dname+ str(id) + '/pose'

    pos_subscriber=rospy.Subscriber(topic, PoseStamped, partial( pose_rcv, pose_rcv_msg, agent_Pose))

    while not pose_rcv_msg[0] and not rospy.is_shutdown():
        rospy.sleep(0.25)

    goal = WaypointNavigationGoal()
    goal.header.frame_id = 'world'
    goal.header.stamp = rospy.get_rostime()
    goal.end_action = WaypointNavigationGoal.HOVER

    agent_Pose.position.z = altitud
    goal.waypoints += [copy.deepcopy(agent_Pose)]
    client.send_goal(goal)


    rospy.loginfo('waiting for '+ dname +'%s to complete' % (id))

    pos_subscriber.unregister()
#    client.wait_for_result()
    return client


def pose_rcv(rcv_msg, agent, msg):
    rcv_msg[0] = True
    agent.position = msg.pose.position
    agent.orientation = msg.pose.orientation
