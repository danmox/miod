import numpy as np
import time as systime
from functools import partial
from connectivity_optimization import ConnectivityOpt

import rospy
import actionlib
from intel_aero_navigation.msg import WaypointNavigationAction, WaypointNavigationGoal
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
from socp.rr_socp_server import RRSOCPServer
from socp.rr_socp_tests import numpy_to_ros, socp_info
from socp.msg import QoS
from socp.srv import RobustRoutingSOCPRequest
from routing_msgs.msg import NetworkUpdate, PRTableEntry, ProbGateway
from visualization_msgs.msg import Marker, MarkerArray
from routing_msgs.msg import NetworkUpdate
from std_msgs.msg import Float64MultiArray, Header


def np_debug(msg): rospy.logdebug('[network_planner] ' + msg)
def np_info(msg): rospy.loginfo('[network_planner] ' + msg)
def np_warn(msg): rospy.logwarn('[network_planner] ' + msg)
def np_error(msg): rospy.logerr('[network_planner] ' + msg)
def np_fatal(msg): rospy.logfatal('[network_planner] ' + msg)


class NetworkPlanner:
    def __init__(self, comm_reqs=None):

        self.run_planner = True

        # save task team communication requirements (flows)

        if comm_reqs is None:
            np_error('no communication requirements provided')
            self.run_planner = False
            return
        else:
            self.comm_reqs = comm_reqs

        # load required parameters and quit if any fail

        required_params = ['/desired_altitude', '~collision_distance', '~minimum_update_rate',
                           '~pose_topic', '~nav_nodelet', '~world_frame', '/task_agent_ids',
                           '/comm_agent_ids', '~planner_type', '/N0', '/n', '/L0','/a', '/b']
        self.params = {}
        for param in required_params:
            if rospy.has_param(param):
                self.params[param[1:]] = rospy.get_param(param)
            else:
                np_fatal('failed to fetch ROS param \'{}\''.format(param))
                self.run_planner = False
                return

        # initialize connectivity optimization planner and robust routing solver

        self.conn_opt = ConnectivityOpt(print_values=False, n0=self.params['N0'], n=self.params['n'],
                                        l0=self.params['L0'], a=self.params['a'], b=self.params['b'])
        np_info('N0 = {}'.format(self.params['N0']))
        np_info('n = {}'.format(self.params['n']))
        np_info('L0 = {}'.format(self.params['L0']))
        np_info('a = {}'.format(self.params['a']))
        np_info('b = {}'.format(self.params['b']))

        self.routing_solver = RRSOCPServer(print_values=False, n0=self.params['N0'], n=self.params['n'],
                                           l0=self.params['L0'], a=self.params['a'], b=self.params['b'])

        # load optional parameters

        optional_params = {'/safety_x_min': None, '/safety_y_min': None, '/safety_x_max': None,
                           '/safety_y_max': None, '~print_routes': False}
        for param in optional_params.keys():
            if rospy.has_param(param):
                self.params[param[1:]] = rospy.get_param(param)
            else:
                self.params[param[1:]] = optional_params[param]
                np_warn('failed to fetch ROS param \'{}\', using default value of {}'.format(param[1:], optional_params[param]))

        # helpful indexing conversions

        self.agent_ids = self.params['task_agent_ids'] + self.params['comm_agent_ids']
        self.agent_count = len(self.agent_ids)
        self.idx_to_id = {}
        self.id_to_idx = {}
        self.idx_to_ip = {} # NOTE here we assume id maps to ip (i.e. ip = XXX.XXX.XXX.id)
        self.id_to_ip  = {} # ''
        for idx, id in enumerate(self.agent_ids):
            ip = '10.42.0.{}'.format(id)
            self.idx_to_id[idx] = id
            self.id_to_idx[id]  = idx
            self.idx_to_ip[idx] = ip
            self.id_to_ip[id]   = ip

        # subscribe to team configuration pose messages and navigation clients
        # NOTE: the pose_topic parameter uses '#' as a wildcard; all instances
        # of # will get replaced by the agent id when subscribing to the topic

        self.team_config = self.agent_count*[Point]
        self.pose_subs = []
        self.received_pose = self.agent_count*[False]
        for id in self.agent_ids:
            topic = self.params['pose_topic'].replace('#', str(id))
            self.pose_subs.append(rospy.Subscriber(topic, PoseStamped, partial(self.pose_cb, id)))

        self.nav_clients = []
        for id in self.params['comm_agent_ids']:
            topic = self.params['nav_nodelet'].replace('#', str(id))
            self.nav_clients.append(actionlib.SimpleActionClient(topic, WaypointNavigationAction))

        self.viz_pub = rospy.Publisher('planner', MarkerArray, queue_size=2)
        self.net_pub = rospy.Publisher('network_update', NetworkUpdate, queue_size=2)
        self.qos_pub = rospy.Publisher('qos', Float64MultiArray, queue_size=2)

    def pose_cb(self, id, msg):
        self.team_config[self.id_to_idx[id]] = msg.pose.position
        self.received_pose[self.id_to_idx[id]] = True

    def update_config_loop(self):

        # initialize agents

        np_info('waiting for action servers to start')
        for client in self.nav_clients:
            client.wait_for_server()
        np_info('action servers started')

        np_info('waiting for team config')
        while not all(self.received_pose) and not rospy.is_shutdown():
            systime.sleep(0.25)
        np_info('team config received')

        np_info('sending takeoff goals')
        for id, client in zip(self.params['comm_agent_ids'], self.nav_clients):
            point = self.team_config[self.id_to_idx[id]]
            goal = WaypointNavigationGoal()
            goal.header = Header(frame_id='world', stamp=rospy.get_rostime())
            goal.waypoints = [Pose(Point(point.x, point.y, self.params['desired_altitude']), Quaternion(w=1.))]
            goal.end_action = WaypointNavigationGoal.HOVER
            client.send_goal(goal)

        np_info('waiting for agents to takeoff')
        for client in self.nav_clients:
            client.wait_for_result()
        np_info('initialization complete')

        # planning loop

        it = 1
        t_prev_it = systime.time()
        while not rospy.is_shutdown():

            # update team config

            x_task = np.zeros((len(self.params['task_agent_ids']),2))
            for idx, id in enumerate(self.params['task_agent_ids']):
                pt = self.team_config[self.id_to_idx[id]]
                x_task[idx,:] = np.asarray([pt.x, pt.y])

            x_comm = np.zeros((len(self.params['comm_agent_ids']),2))
            for idx, id in enumerate(self.params['comm_agent_ids']):
                pt = self.team_config[self.id_to_idx[id]]
                x_comm[idx,:] = np.asarray([pt.x, pt.y])

            self.conn_opt.set_team_config(x_task, x_comm)

            # compute target network team config

            t_start = systime.time()
            updates = 0
            while systime.time() - t_start < 1.0/self.params['minimum_update_rate']:
                lambda2 = self.conn_opt.update_network_config(step_size=0.2)
                updates += 1

            # send network team update

            target_config = numpy_to_ros(self.conn_opt.config, z=self.params['desired_altitude'])
            for id, client in zip(self.params['comm_agent_ids'], self.nav_clients):
                goal = WaypointNavigationGoal()
                goal.header = Header(frame_id='world', stamp=rospy.get_rostime())
                goal.waypoints = [Pose(target_config[self.id_to_idx[id]], Quaternion(w=1.))]
                goal.end_action = WaypointNavigationGoal.HOVER
                client.send_goal(goal)

            # update network routes

            self.update_routes()

            # print iteration information

            print('\niteration {} complete'.format(it))
            print('connectivity iterations completed     : {}'.format(updates))
            print('iteration duration / minimum duration : {:.3f}/{:.3f}s'.format(
                systime.time() - t_prev_it, 1.0/self.params['minimum_update_rate']))
            print('lambda2                               : {}'.format(lambda2))
            print('slack                                 : {}'.format(self.slack))
            if self.params['print_routes']:
                print('')
                socp_info(self.routing_vars, self.comm_reqs)

            it += 1
            t_prev_it = systime.time()

    def update_routing_loop(self):

        # initialize agents

        np_info('waiting for team config')
        while not all(self.received_pose) and not rospy.is_shutdown():
            systime.sleep(0.25)
        np_info('team config received')

        # routing loop

        it = 1
        t_prev_it = systime.time()
        while not rospy.is_shutdown():

            # update network routes

            success = self.update_routes()

            # don't run faster than self.params['minimum_update_rate']

            elapsed_time = systime.time() - t_prev_it
            if elapsed_time < 1.0 / self.params['minimum_update_rate']:
                systime.sleep(1.0 / self.params['minimum_update_rate'] - elapsed_time)
            t_current = systime.time()
            iteration_duration = t_current - t_prev_it
            t_prev_it = t_current

            # print iteration information

            print('\niteration {} complete'.format(it))
            print('iteration duration / minimum duration : {:.3f}/{:.3f}s'.format(
                iteration_duration, 1.0/self.params['minimum_update_rate']))
            if success:
                print('slack                                 : {}'.format(self.slack))
                if self.params['print_routes']:
                    print('')
                    socp_info(self.routing_vars, self.comm_reqs)

            it += 1

    def update_routes(self):

        # solve for routing variables

        idx_to_id_list = [self.idx_to_id[i] for i in range(self.agent_count)]
        req = RobustRoutingSOCPRequest(self.team_config, self.comm_reqs, idx_to_id_list)
        res = self.routing_solver.solve_socp(req)

        if res.status != 'optimal':
            np_error('routing_solver returned with status \'{}\''.format(res.status))
            self.slack = None
            self.routing_vars = []
            return False

        if len(res.routes) != self.agent_count**2 * len(self.comm_reqs):
            np_error('routing_solver returned invalid number of routing variables')
            self.slack = None
            self.routing_vars = []
            return False

        self.slack = res.slack
        self.routing_vars = np.reshape(np.asarray(res.routes),
                                       (self.agent_count, self.agent_count, len(self.comm_reqs)), 'F')

        # pack and publish routing update message

        network_update = NetworkUpdate()
        for k in range(len(self.comm_reqs)):
            src_ip = self.id_to_ip[self.comm_reqs[k].src]
            dest_ips = [self.id_to_ip[id] for id in self.comm_reqs[k].dest]
            for i in range(self.agent_count):

                # TODO normalize routes to sum to 1 (but doing so causes the
                # routing visualization to be less insightful so not doing it
                # for now)
                routes = self.routing_vars[i,:,k]
                valid = self.agent_count*[True]
                valid = routes > 0.01
                if not any(valid):
                    continue
                routes[~valid] = 0.0

                valid_ips = [self.id_to_ip[id] for id, val in zip(self.agent_ids, valid) if val]
                prob_gws = [ProbGateway(ip, p) for ip, p in zip(valid_ips, routes[valid])]
                node_ip = self.idx_to_ip[i]
                network_update.routes += [PRTableEntry(node_ip, src_ip, ip, prob_gws) for ip in dest_ips]

        self.net_pub.publish(network_update)

        return True

    def run(self):
        if not self.run_planner:
            np_fatal('self.run_planner set to false: exiting')
            return

        if self.params['planner_type'] == 'moving':
            self.update_config_loop()
        elif self.params['planner_type'] == 'fixed':
            self.update_routing_loop()
        else:
            np_error('unknown planner type: {}'.format(self.params['planner_type']))
            return


if __name__ == '__main__':
    try:
        rospy.init_node('connectivity_network_planner')

        network_planner = NetworkPlanner()
        network_planner.run()

    except rospy.ROSInterruptException:
        pass
