#!/usr/bin/env python

import copy
import sys
import numpy as np
import rr_socp
import rr_socp_server
import time
from socp.srv import RobustRoutingSOCPRequest
from socp.msg import QoS
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import matplotlib as mpl

# helps the figures to be readable on hidpi screens
mpl.rcParams['figure.dpi'] = 200

def numpy_to_ros(np_config, z=0.):
    """
    convert a Nx2 numpy array of 2D node positions to a list of
    geometry_msgs.Points

    """
    ros_config = []
    for i in range(np_config.shape[0]):
        pt = Point()
        pt.x = np_config[i,0]
        pt.y = np_config[i,1]
        pt.z = z
        ros_config += [copy.deepcopy(pt)]
    return ros_config

def plot_config(config, ax=None, pause=None, clear_axes=False, show=True, title=None):
    """
    plot the 2D spatial configuration of the network

    Input:
      config: a list of geometry_msgs.Point msgs
      ax (optional): axes to plot on
      pause (optional): avoids blocking by continuing after a short pause
      clear_axes: clear ax before plotting
      show: call plt.show()

    """
    x = []
    y = []
    for pt in config:
        x += [pt.x]
        y += [pt.y]

    if ax is None:
        fig, ax = plt.subplots()

    if clear_axes:
        ax.cla()

    ax.plot(x, y, 'ro', markersize=10)
    ax.axis('scaled')
    ax.axis([min(x) - 2.0, max(x) + 2.0, min(y) - 2.0, max(y) + 2.0])
    for i in range(len(x)):
        ax.annotate(str(i + 1), (x[i] + 0.6, y[i] + 0.6), fontweight='bold')

    if title is not None:
        ax.set_title(title)

    if show:
        if pause is None:
            plt.show()
        else:
            plt.pause(pause)


def socp_info(routes, qos, config=None, solver=None, ids=None):
    """
    print information about the robust routing solution

    Input:
      routes: an NxNxK array of routing variables
      qos: an array of flow requirements with length(qos) == K
      config: (optional) the configuration of the team to plot
      solver: (optional) will print the channel rate, var if the config is provided
      ids: (optional) node ids to use instead of 1,...,n

    """
    assert len(qos) == routes.shape[2]
    n = routes.shape[0]
    if ids is not None:
        assert len(ids) == n
    else:
        ids = range(1,n+1)
    id_to_idx = {id: idx for id, idx in zip(ids, range(n))}
    idx_to_id = {idx: id for id, idx in zip(ids, range(n))}

    for k in range(len(qos)):
        # flow info
        print "flow %d: %d -> %s, margin = %.2f, confidence = %.2f:"\
              % (k+1, qos[k].src, ", ".join(map(str, qos[k].dest)), qos[k].margin, qos[k].confidence)
        # column header
        node_names = ["%6s" % i for i in ids]
        node_names[id_to_idx[qos[k].src]] = "%6s" % ("(s) " + str(qos[k].src))
        for d in qos[k].dest:
            node_names[id_to_idx[d]] = "%6s" % ("(d) " + str(d))
        print "   %6s|%s|%5s" % (" ", "".join(map(str, node_names)), "Tx")
        print "   %s" % ("-" * (6 * (n+2) + 1))
        for i in range(n):
            num_str = ["%6.2f" % a if a > 0.01 else "%6s" % '-' for a in routes[i, :, k]]
            if sum(routes[i, :, k]) > 0.01:
                num_str += ["|%5.2f" % sum(routes[i, :, k])]
            else:
                num_str += ["|%5s" % "-"]
            print "   %6s|%s" % (node_names[i], "".join(num_str))
        print "   %6s|%s|" % ("", "-" * (6*n))
        rx_str = ["%6.2f" % sum(routes[:, i, k]) if sum(routes[:, i, k]) > 0.01 else "%6s" % "-" for i in range(n)]
        print "   %6s|%s" % ("Rx", "".join(rx_str))

    if config is not None and solver is not None:
        rate_mean, rate_var = solver.cm.predict(config)
        with np.printoptions(precision=3, suppress=True):
            print('rate_mean:')
            print(rate_mean)
            print('rate_var:')
            print(rate_var)


# Test 1
def multiple_dest_test():
    print "running multiple_dest_test()\n"
    msg = RobustRoutingSOCPRequest()

    x = [20.0, -10.0, -10.0, 5.0]
    y = [0.0, 17.32, -17.32, 5.0]
    n = len(x)
    for i in range(n):
        pt = Point()
        pt.x = x[i]
        pt.y = y[i]
        msg.config += [copy.deepcopy(pt)]

    qos = QoS()
    qos.margin = 0.02
    qos.confidence = 0.7
    src = [0, 1, 2]
    dest = [[2, 1], [2, 0], [1, 0]]
    k = len(src)
    for i in range(k):
        qos.src = src[i]
        qos.dest = dest[i]
        msg.qos += [copy.deepcopy(qos)]

    rrserver = rr_socp_server.RRSOCPServer(fetch_params=False)
    res = rrserver.solve_socp(msg)
    if res.status != 'optimal':
        print("solve_socp returned with status: %s" % res.status)
        return

    routes = np.reshape(res.routes, (n, n, k), 'F')
    print("slack = %f" % res.slack)
    socp_info(routes, msg.qos, msg.config)


# Test 2
def speed_test():
    print "running speed test\n"

    patrol_rad = 20
    comm_rad = 8
    task_agent_count = 3
    comm_agent_count = 6

    qos_list = []
    src = [0, 1, 2]
    dest = [[1, 2], [0, 2], [0, 1]]
    qos = QoS()
    qos.margin = 0.10
    qos.confidence = 0.70
    for i in range(len(src)):
        qos.src = src[i]
        qos.dest = dest[i]
        qos_list += [copy.deepcopy(qos)]

    # build team structure

    x_task = np.zeros((task_agent_count, 2))
    x_task[:, 0] = patrol_rad * np.cos(2.0 * np.pi / task_agent_count * np.arange(0, task_agent_count))
    x_task[:, 1] = patrol_rad * np.sin(2.0 * np.pi / task_agent_count * np.arange(0, task_agent_count))
    x_comm = np.zeros((comm_agent_count, 2))
    x_comm[1:, 0] = comm_rad * np.cos(2.0 * np.pi / (comm_agent_count - 1) * np.arange(0, comm_agent_count - 1))
    x_comm[1:, 1] = comm_rad * np.sin(2.0 * np.pi / (comm_agent_count - 1) * np.arange(0, comm_agent_count - 1))
    x = np.vstack([x_task, x_comm])

    # solve SOCP

    rrsolver = rr_socp.RobustRoutingSolver(print_values=True)
    for i in range(10):
        start = time.time()
        slack, routes, status = rrsolver.solve_socp(qos_list, x)
        print("found solution in %.4f second" % (time.time() - start))


# Test 3
def simple_routing_test(margin=0.1, confidence=0.7):
    print "running simple_routing_test()\n"
    msg = RobustRoutingSOCPRequest()

    dist = 30
    x = [0.0, dist, 1.0/3.0 * dist, 2.0/3.0 * dist]
    y = [0.0, 0.0, 3.0, -3.0]
    n = len(x)
    for i in range(n):
        pt = Point()
        pt.x = x[i]
        pt.y = y[i]
        msg.config += [copy.deepcopy(pt)]

    print("margin = %.3f, confidence = %.3f" % (margin, confidence))
    qos = QoS()
    qos.margin = margin
    qos.confidence = confidence
    qos.src = 1
    qos.dest = [0]
    msg.qos += [copy.deepcopy(qos)]
    k = len(msg.qos)

    rrserver = rr_socp_server.RRSOCPServer(fetch_params=False)
    res = rrserver.solve_socp(msg)
    if res.status != 'optimal':
        print("solve_socp returned with status: %s" % res.status)
        return

    routes = np.reshape(res.routes, (n, n, k), 'F')
    print("slack = %.4f" % res.slack)
    socp_info(routes, msg.qos, msg.config)


# Test 4
# TODO these results are inconsistent with MATLAB
def matlab_match_test(margin=0.05, confidence=0.7):
    print "running matlab_match_test()\n"

    x = [20.0, -10.0, -10.0, 5.0]
    y = [0.0, 17.32, -17.32, 5.0]
    z = [0.05, 0.05, 0.05, 1.83]
    n = len(x)
    msg = RobustRoutingSOCPRequest()
    for i in range(n):
        pt = Point()
        pt.x = x[i]
        pt.y = y[i]
        pt.z = z[i]
        msg.config += [copy.deepcopy(pt)]

    src = [0, 1, 2]
    dest = [[1, 2], [0, 2], [0, 1]]
    k = len(src)
    qos = QoS()
    qos.margin = margin
    qos.confidence = confidence
    for i in range(k):
        qos.src = src[i]
        qos.dest = dest[i]
        msg.qos += [copy.deepcopy(qos)]

    rrserver = rr_socp_server.RRSOCPServer(fetch_params=False, l0=-48.0)
    res = rrserver.solve_socp(msg)
    if res.status != 'optimal':
        print("solve_socp returned with status: %s" % res.status)
        return

    routes = np.reshape(res.routes, (n, n, k), 'F')
    print("slack = %.4f" % res.slack)
    socp_info(routes, msg.qos, msg.config)


# Test 5
def infeasible_test(margin=0.5, confidence=0.9):
    print "running simple_routing_test()\n"
    msg = RobustRoutingSOCPRequest()

    dist = 30
    x = [0.0, dist, 1.0/3.0 * dist, 2.0/3.0 * dist]
    y = [0.0, 0.0, 3.0, -3.0]
    n = len(x)
    for i in range(n):
        pt = Point()
        pt.x = x[i]
        pt.y = y[i]
        msg.config += [copy.deepcopy(pt)]

    print("margin = %.3f, confidence = %.3f" % (margin, confidence))
    qos = QoS()
    qos.margin = margin
    qos.confidence = confidence
    qos.src = 1
    qos.dest = [0]
    msg.qos += [copy.deepcopy(qos)]
    k = len(msg.qos)

    rrserver = rr_socp_server.RRSOCPServer(fetch_params=False)
    res = rrserver.solve_socp(msg)
    if res.status != 'optimal':
        print("solve_socp returned with status: %s" % res.status)
        return

    routes = np.reshape(res.routes, (n, n, k), 'F')
    print("slack = %.4f" % res.slack)
    socp_info(routes, msg.qos, msg.config)


if __name__ == "__main__":
    if len(sys.argv) is not 2:
        print "running all tests\n"
        multiple_dest_test()
        print "\n"
        speed_test()
        print "\n"
        simple_routing_test()
        print "\n"
        matlab_match_test()
        print "\n"
        infeasible_test()
    else:
        arg = int(sys.argv[1])
        if arg == 1:
            multiple_dest_test()
        elif arg == 2:
            speed_test()
        elif arg == 3:
            simple_routing_test()
        elif arg == 4:
            matlab_match_test()
        elif arg == 5:
            infeasible_test()
        else:
            print "unkown argument %d" % arg
