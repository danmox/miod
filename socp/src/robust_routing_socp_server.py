#!/usr/bin/env python

import numpy as np
from scipy import special, spatial, stats
import cvxpy as cp
import rospy
from socp.srv import RobustRoutingSOCP, RobustRoutingSOCPResponse


#
# Robust Routing SOCP functions
#

def dbm2mw(dbm):
    return 10.0 ** (np.asarray(dbm) / 10.0)


class ChannelModel:
    def __init__(self, fetch_params=None, print_values=None, n0=-70.0, n=2.52, l0=-53.0, a=0.2, b=6.0):
        if fetch_params is None or fetch_params is False:
            self.L0 = l0  # transmit power (dBm)
            self.n = n  # decay rate
            self.N0 = n0  # noise at receiver (dBm)
            self.a = a  # sigmoid parameter 1
            self.b = b  # sigmoid parameter 2
        else:
            self.N0 = rospy.get_param("/N0")
            self.n = rospy.get_param("/n")
            self.L0 = rospy.get_param("/L0")
            self.a = rospy.get_param("/a")
            self.b = rospy.get_param("/b")
        self.PL0 = dbm2mw(self.L0)  # transmit power (mW)
        self.PN0 = dbm2mw(self.N0)  # noise at receiver (mW)
        if print_values is not None and print_values is True:
            print('L0 = %.3f' % self.L0)
            print('n  = %.3f' % self.n)
            print('N0 = %.3f' % self.N0)
            print('a  = %.3f' % self.a)
            print('b  = %.3f' % self.b)

    def predict(self, d):
        dist_mask = ~np.eye(d.shape[0], dtype=bool)
        power = np.zeros(d.shape)
        power[dist_mask] = dbm2mw(self.L0 - 10 * self.n * np.log10(d[dist_mask]))
        rate = 1 - special.erfc(np.sqrt(power / self.PN0))
        var = (self.a * d / (self.b + d)) ** 2
        return rate, var


# create global instance of channel model
cm = ChannelModel()


def nodemarginconsts(qos, rate_mean, rate_var):
    n = rate_mean.shape[0]
    k = len(qos)

    zero_vars = np.reshape(np.eye(n, dtype=bool), (n, n, 1), 'F')
    zero_vars = np.repeat(zero_vars, k, axis=2)
    # for k in range(0,K):
    #    zero_vars[np.asarray(qos[k].dest)-1,:,k] = True

    a_mat = np.zeros([n * k, n * n * k + 1])
    b_mat = np.zeros([n * k, n * n * k + 1])

    idx = 0
    for flow_idx in range(0, k):
        for i in range(0, n):
            aki = np.zeros([n, n])
            bki = np.zeros([n, n])

            aki[:, i] = np.sqrt(rate_var[:, i])
            aki[i, :] = np.sqrt(rate_var[i, :])
            aki[zero_vars[:, :, flow_idx]] = 0.0

            if not np.any(np.asarray(qos[flow_idx].dest) - 1 == i):
                bki[:, i] = -rate_mean[:, i]
                bki[i, :] = rate_mean[i, :]
                bki[zero_vars[:, :, flow_idx]] = 0.0
            else:
                bki[:, i] = rate_mean[:, i]
                # Bki[i,:] = -R[i,:]
                bki[zero_vars[:, :, flow_idx]] = 0.0

            a_mat[idx, flow_idx * n * n:(flow_idx + 1) * n * n] = np.reshape(aki, (1, -1), 'F')
            b_mat[idx, flow_idx * n * n:(flow_idx + 1) * n * n] = np.reshape(bki, (1, -1), 'F')
            b_mat[idx, n * n * k] = -1

            idx += 1

    return a_mat, b_mat, zero_vars


# qos - a list of qos dicts
# x   - a 2xN np array of x,y positions
def rrsocpconf(qos, x):
    # node margin constraints

    dist = spatial.distance_matrix(x.T, x.T)
    rate_mean, rate_var = cm.predict(dist)
    a_mat, b_mat, zero_vars = nodemarginconsts(qos, rate_mean, rate_var)

    # margin and confidence vectors

    k = len(qos)
    n = x.shape[1]

    conf = np.ones([n, k]) * np.asarray([qos[i].confidence for i in range(0, k)])
    conf = np.reshape(conf, [-1, 1], 'F')
    conf = stats.norm.ppf(conf)

    m_ik = np.zeros([n, k])
    for i in range(0, k):
        m_ik[np.hstack([qos[i].src, qos[i].dest]) - 1, i] = qos[i].margin
    m_ik = np.reshape(m_ik, (n * k), 'F')

    # form and solve SOCP

    # optimization variables
    slack, routes = cp.Variable((1)), cp.Variable((n * n * k))
    y = cp.hstack([routes, slack])

    # cone constraints
    cone_consts = []
    for i in range(a_mat.shape[0]):
        cone_consts += [cp.SOC((cp.matmul(b_mat[i, :], y) - m_ik[i]) / conf[i], cp.matmul(cp.diag(a_mat[i, :]), y))]

    # linear availability constraints
    routing_sum_mat = cp.reshape(routes[:n * n], (n, n))  # acts like np.reshape with 'F'
    for i in range(1, k):  # summing over dim 3
        routing_sum_mat += cp.reshape(routes[i * n * n:(i + 1) * n * n], (n, n))
    lin_consts = [cp.sum(routing_sum_mat, 2) <= 1]
    lin_consts += [cp.sum(routing_sum_mat, 1) <= 1]

    # solve program with CVX
    sign_consts = [0 <= routes, routes <= 1, 0 <= slack]
    zero_var_consts = [routes[np.reshape(zero_vars, (n * n * k), 'F')] == 0]
    constraints = cone_consts + lin_consts + sign_consts + zero_var_consts
    socp = cp.Problem(cp.Maximize(slack), constraints)
    socp.solve(warm_start=True)

    return slack.value, routes.value, socp.status


#
# ROS service functions
#

def solve_socp(req):
    res = RobustRoutingSOCPResponse()

    # check for valid message
    if len(req.config) == 0 or len(req.qos) == 0:
        rospy.logwarn('[robust_routing_socp_server] invalid reqeust')
        return res

    # unpack the message

    x = np.zeros((2, len(req.config)))
    for i in range(len(req.config)):
        x[0, i] = req.config[i].x
        x[1, i] = req.config[i].y

    slack, routes, status = rrsocpconf(req.qos, x)
    if status == 'optimal':
        res.slack = slack
        res.routes = routes
        # N = len(req.config)
        # K = len(req.qos)
        # routes = np.reshape(res.routes, (N,N,K), 'F')
        # pdb.set_trace()

    res.status = status

    return res


def robust_routing_socp_server():
    rospy.init_node('robust_routing_socp_server')
    s = rospy.Service('RobustRoutingSOCP', RobustRoutingSOCP, solve_socp)
    print "robust_routing_socp_server ready"
    rospy.spin()


if __name__ == "__main__":
    cm = ChannelModel(True, True)
    robust_routing_socp_server()
