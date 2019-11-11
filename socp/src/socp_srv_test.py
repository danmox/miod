#!/usr/bin/env python

import copy
import sys
import numpy as np
import robust_routing_socp_server as rrss
from socp.srv import RobustRoutingSOCPRequest
from socp.msg import QoS
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import pdb


# Test 1
def test1():

    msg = RobustRoutingSOCPRequest()

    pt = Point()
    pt.x = 20
    pt.y = 0
    pt.z = 0
    #pt.z = 0.05
    msg.config += [copy.deepcopy(pt)]
    pt.x = -10
    pt.y = 17.32
    msg.config += [copy.deepcopy(pt)]
    pt.x = -10
    pt.y = -17.32
    msg.config += [copy.deepcopy(pt)]
    pt.x = 5
    pt.y = 5
    #pt.z = 1.85
    msg.config += [copy.deepcopy(pt)]
    N = len(msg.config)

    qos = QoS()
    qos.margin = 0.02
    qos.confidence = 0.7
    qos.src = 1
    qos.dest = [3, 2]
    msg.qos += [copy.deepcopy(qos)]
    qos.src = 2
    qos.dest = [3, 1]
    msg.qos += [copy.deepcopy(qos)]
    qos.src = 3
    qos.dest = [2, 1]
    msg.qos += [copy.deepcopy(qos)]
    K = len(msg.qos)

    res = rrss.solveSOCP(msg)
    routes = np.reshape(res.routes, (N,N,K), 'F')
    print(res.slack)
    for i in range(K):
        print('alpha_ij_%d' % (i+1))
        print(np.array_str(routes[:,:,i], precision=4, suppress_small=True))


### Test 2
def test2():

    patrol_rad = 20;
    comm_rad = 8;
    task_agent_count = 3;
    comm_agent_count = 6;

    qos = []
    qos1 = {"flow" : {"src" : 1, "dest" : [2, 3]}, "margin" : 0.10, "confidence" : 0.70}
    qos2 = {"flow" : {"src" : 2, "dest" : [1, 3]}, "margin" : 0.10, "confidence" : 0.70}
    qos3 = {"flow" : {"src" : 3, "dest" : [1, 2]}, "margin" : 0.10, "confidence" : 0.70}
    qos = [qos1, qos2, qos3]

    # build team structure

    x_task = np.zeros([2, task_agent_count])
    x_task[0,:] = patrol_rad*np.cos(2.0*np.pi/task_agent_count * np.arange(0,task_agent_count))
    x_task[1,:] = patrol_rad*np.sin(2.0*np.pi/task_agent_count * np.arange(0,task_agent_count))
    x_comm = np.zeros([2, comm_agent_count])
    x_comm[0,1:] = comm_rad*np.cos(2.0*np.pi/(comm_agent_count-1) * np.arange(0,comm_agent_count-1))
    x_comm[1,1:] = comm_rad*np.sin(2.0*np.pi/(comm_agent_count-1) * np.arange(0,comm_agent_count-1))
    x = np.hstack([x_task, x_comm])

    # solve SOCP

    for i in range(10):
        start = time.time()
        slack, routes, status = rrsocpconf(qos, x)
        print("found solution in %.4f second" % (time.time() - start))


### Test 3
def test3(margin=0.1,confidence=0.7):

    dist = 30;

    msg = RobustRoutingSOCPRequest()
    pt = Point()
    msg.config += [copy.deepcopy(pt)]
    pt.x = dist
    msg.config += [copy.deepcopy(pt)]
    pt.x = 1.0/3.0*dist
    pt.y = 3.0
    msg.config += [copy.deepcopy(pt)]
    pt.x = 2.0/3.0*dist
    pt.y = -3.0
    msg.config += [copy.deepcopy(pt)]
    N = len(msg.config)

    print("margin = %.3f, confidence = %.3f" % (margin, confidence))
    qos = QoS()
    qos.margin = margin
    qos.confidence = confidence
    qos.src = 2
    qos.dest = [1]
    msg.qos += [copy.deepcopy(qos)]
    K = len(msg.qos)

    res = rrss.solveSOCP(msg)
    if res.status != 'optimal':
        print("solveSOCP returned with status: %s" % res.status)
        return

    routes = np.reshape(res.routes, (N,N,K), 'F')
    print("slack = %.4f" % res.slack)
    for i in range(K):
        print('alpha_ij_%d' % (i+1))
        routes_tmp = routes[:,:,i]
        routes_tmp[np.absolute(routes_tmp) < 1e-6] = 0.0
        print(np.array_str(routes_tmp, precision=2, suppress_small=True))

    # plot results

    x = []
    y = []
    for pt in msg.config:
        x += [pt.x]
        y += [pt.y]

    plt.plot(x, y, 'ro', markersize=12)
    plt.axis('scaled')
    plt.axis([min(x)-2.0, max(x)+2.0, min(y)-2.0, max(y)+2.0])
    for i in range(N):
        plt.annotate(str(i+1), (x[i]+0.6, y[i]+0.6), fontweight='bold')
    plt.show()


### Test 4
# TODO results inconsistant with SOCP!!!
def test4(margin=0.05,confidence=0.7):

    rrss.cm = rrss.channel_model(fetch_params=False, print_values=True, L0=-48.0)

    msg = RobustRoutingSOCPRequest()
    pt = Point()
    pt.x = 20.0
    pt.y = 0.0
    pt.z = 0.05
    msg.config += [copy.deepcopy(pt)]
    pt.x = -10.0
    pt.y = 17.32
    msg.config += [copy.deepcopy(pt)]
    pt.y = -17.32
    msg.config += [copy.deepcopy(pt)]
    pt.x = 5.0
    pt.y = 5.0
    pt.z = 1.83
    msg.config += [copy.deepcopy(pt)]
    N = len(msg.config)

    for pt in msg.config:
        print("(%6.2f, %6.2f, %6.2f)" % (pt.x, pt.y, pt.z))

    qos = QoS()
    qos.margin = margin
    qos.confidence = confidence
    qos.src = 1
    qos.dest = [2, 3]
    msg.qos += [copy.deepcopy(qos)]
    qos.src = 2
    qos.dest = [1, 3]
    msg.qos += [copy.deepcopy(qos)]
    qos.src = 3
    qos.dest = [1, 2]
    msg.qos += [copy.deepcopy(qos)]
    K = len(msg.qos)

    for i in range(0,K):
        print("flow %d:" % (i+1))
        print("  sources: %d" % msg.qos[i].src)
        print("  destinations: %s" % ', '.join(map(str, msg.qos[i].dest)))
        print("  margin = %.2f" % margin)
        print("  confidence = %.2f" % confidence)

    res = rrss.solveSOCP(msg)
    if res.status != 'optimal':
        print("solveSOCP returned with status: %s" % res.status)
        return

    routes = np.reshape(res.routes, (N,N,K), 'F')
    print("slack = %.4f" % res.slack)
    for i in range(K):
        print('alpha_ij_%d' % (i+1))
        routes_tmp = routes[:,:,i]
        routes_tmp[np.absolute(routes_tmp) < 1e-6] = 0.0
        print(np.array_str(routes_tmp, precision=3, suppress_small=True))


if __name__ == "__main__":
    if len(sys.argv) is not 3:
        test4()
    else:
        test4(float(sys.argv[1]), float(sys.argv[2]))
