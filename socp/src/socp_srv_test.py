#!/usr/bin/env python

import copy
import sys
import numpy as np
import robust_routing_socp_server as rrss
import time
from socp.srv import RobustRoutingSOCPRequest
from socp.msg import QoS
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt


def plot_config(config):
    x = []
    y = []
    for pt in config:
        x += [pt.x]
        y += [pt.y]

    plt.plot(x, y, 'ro', markersize=12)
    plt.axis('scaled')
    plt.axis([min(x) - 2.0, max(x) + 2.0, min(y) - 2.0, max(y) + 2.0])
    for i in range(len(x)):
        plt.annotate(str(i + 1), (x[i] + 0.6, y[i] + 0.6), fontweight='bold')
    plt.show()


def socp_info(routes, qos, config=None):
    assert len(qos) == routes.shape[2]
    n = routes.shape[0]

    for k in range(len(qos)):
        # flow info
        print "flow %d: %d -> %s, margin = %.2f, confidence = %.2f:"\
              % (k+1, qos[k].src, ", ".join(map(str, qos[k].dest)), qos[k].margin, qos[k].confidence)
        # column header
        node_names = ["%6s" % (i+1) for i in range(n)]
        node_names[qos[k].src-1] = "%6s" % ("(s) " + str(qos[k].src))  # nodes numbered from 1
        for d in qos[k].dest:
            node_names[d-1] = "%6s" % ("(d) " + str(d))
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

    if config:
        plot_config(config)


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
    src = [1, 2, 3]
    dest = [[3, 2], [3, 1], [2, 1]]
    k = len(src)
    for i in range(k):
        qos.src = src[i]
        qos.dest = dest[i]
        msg.qos += [copy.deepcopy(qos)]

    rrsolver = rrss.RobustRoutingSolver()
    res = rrsolver.solve_socp(msg)
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
    src = [1, 2, 3]
    dest = [[2, 3], [1, 3], [1, 2]]
    qos = QoS()
    qos.margin = 0.10
    qos.confidence = 0.70
    for i in range(len(src)):
        qos.src = src[i]
        qos.dest = dest[i]
        qos_list += [copy.deepcopy(qos)]

    # build team structure

    x_task = np.zeros([2, task_agent_count])
    x_task[0, :] = patrol_rad * np.cos(2.0 * np.pi / task_agent_count * np.arange(0, task_agent_count))
    x_task[1, :] = patrol_rad * np.sin(2.0 * np.pi / task_agent_count * np.arange(0, task_agent_count))
    x_comm = np.zeros([2, comm_agent_count])
    x_comm[0, 1:] = comm_rad * np.cos(2.0 * np.pi / (comm_agent_count - 1) * np.arange(0, comm_agent_count - 1))
    x_comm[1, 1:] = comm_rad * np.sin(2.0 * np.pi / (comm_agent_count - 1) * np.arange(0, comm_agent_count - 1))
    x = np.hstack([x_task, x_comm])

    # solve SOCP

    rrsolver = rrss.RobustRoutingSolver()
    for i in range(10):
        start = time.time()
        slack, routes, status = rrsolver.rrsocpconf(qos_list, x)
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
    qos.src = 2
    qos.dest = [1]
    msg.qos += [copy.deepcopy(qos)]
    k = len(msg.qos)

    rrsolver = rrss.RobustRoutingSolver()
    res = rrsolver.solve_socp(msg)
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

    src = [1, 2, 3]
    dest = [[2, 3], [1, 3], [1, 2]]
    k = len(src)
    qos = QoS()
    qos.margin = margin
    qos.confidence = confidence
    for i in range(k):
        qos.src = src[i]
        qos.dest = dest[i]
        msg.qos += [copy.deepcopy(qos)]

    rrsolver = rrss.RobustRoutingSolver(l0=-48.0)
    res = rrsolver.solve_socp(msg)
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
        else:
            print "unkown argument %d" % arg
