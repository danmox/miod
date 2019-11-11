#!/usr/bin/env python

import numpy as np
from scipy import special, spatial, stats
import cvxpy as cp
import time
import rospy
from socp.srv import RobustRoutingSOCP, RobustRoutingSOCPResponse
from socp.msg import QoS
import pdb

###
### Robust Routing SOCP functions
###

def dBm2mW(dBm):
    return 10.0**(np.asarray(dBm)/10.0)

class channel_model():
    def __init__(self, fetch_params=None, print_values=None, N0=-70.0, n=2.52, L0=-53.0, a=0.2, b=6.0):
        if fetch_params is None or fetch_params is False:
            self.L0 = L0               # transmit power (dBm)
            self.n = n                 # decay rate
            self.N0 = N0               # noise at receiver (dBm)
            self.a = a                 # sigmoid parameter 1
            self.b = b                 # sigmoid parameter 2
        else:
            self.N0 = rospy.get_param("/N0")
            self.n = rospy.get_param("/n")
            self.L0 = rospy.get_param("/L0")
            self.a = rospy.get_param("/a")
            self.b = rospy.get_param("/b")
        self.PL0 = dBm2mW(self.L0) # transmit power (mW)
        self.PN0 = dBm2mW(self.N0) # noise at receiver (mW)
        if print_values is not None and print_values is True:
            print('L0 = %.3f' % self.L0)
            print('n  = %.3f' % self.n)
            print('N0 = %.3f' % self.N0)
            print('a  = %.3f' % self.a)
            print('b  = %.3f' % self.b)

    def predict(self,d):
        dist_mask = ~np.eye(d.shape[0], dtype=bool)
        P = np.zeros(d.shape)
        P[dist_mask] = dBm2mW(self.L0 - 10*self.n*np.log10(d[dist_mask]))
        rate = 1 - special.erfc(np.sqrt(P/self.PN0));
        var = (self.a*d/(self.b + d))**2;
        return rate, var

# create global instance of channel model
cm = channel_model()


def nodemarginconsts(qos, R, V):
    N = R.shape[0]
    K = len(qos)

    zero_vars = np.reshape(np.eye(N, dtype=bool), (N,N,1), 'F')
    zero_vars = np.repeat(zero_vars, K, axis=2)
    #for k in range(0,K):
    #    zero_vars[np.asarray(qos[k].dest)-1,:,k] = True

    A = np.zeros([N*K, N*N*K+1])
    B = np.zeros([N*K, N*N*K+1])

    idx = 0
    for k in range(0,K):
        for i in range(0,N):
            Aki = np.zeros([N,N])
            Bki = np.zeros([N,N])

            Aki[:,i] = np.sqrt(V[:,i])
            Aki[i,:] = np.sqrt(V[i,:])
            Aki[zero_vars[:,:,k]] = 0.0

            if not np.any(np.asarray(qos[k].dest)-1 == i):
                Bki[:,i] = -R[:,i]
                Bki[i,:] = R[i,:]
                Bki[zero_vars[:,:,k]] = 0.0;
            else:
                Bki[:,i] = R[:,i]
                #Bki[i,:] = -R[i,:]
                Bki[zero_vars[:,:,k]] = 0.0;

            A[idx, k*N*N:(k+1)*N*N] = np.reshape(Aki, (1,-1), 'F')
            B[idx, k*N*N:(k+1)*N*N] = np.reshape(Bki, (1, -1), 'F')
            B[idx, N*N*K] = -1

            idx += 1

    return A, B, zero_vars


# qos - a list of qos dicts
# x   - a 2xN np array of x,y positions
def rrsocpconf(qos, x):

    # node margin constraints

    d = spatial.distance_matrix(x.T, x.T)
    R,V = cm.predict(d)
    A, B, zero_vars = nodemarginconsts(qos, R, V)

    # margin and confidence vectors

    K = len(qos)
    N = x.shape[1]

    conf = np.ones([N,K])*np.asarray([qos[i].confidence for i in range(0,len(qos))])
    conf = np.reshape(conf, [-1,1], 'F')
    conf = stats.norm.ppf(conf)

    m_ik = np.zeros([N,K])
    for k in range(0,K):
        m_ik[np.hstack([qos[k].src, qos[k].dest])-1,k] = qos[k].margin
    m_ik = np.reshape(m_ik, (N*K), 'F')

    # form and solve SOCP

    # optimization variables
    slack, routes = cp.Variable((1)), cp.Variable((N*N*K))
    y = cp.hstack([routes, slack])

    # cone constraints
    cone_consts = []
    for i in range(A.shape[0]):
        cone_consts += [cp.SOC((cp.matmul(B[i,:],y) - m_ik[i])/conf[i], cp.matmul(cp.diag(A[i,:]),y))]

    # linear availability constraints
    routing_sum_mat = cp.reshape(routes[:N*N], (N,N)) # acts like np.reshape with 'F'
    for k in range(1,K): # summing over dim 3
        routing_sum_mat += cp.reshape(routes[k*N*N:(k+1)*N*N], (N,N))
    lin_consts = [cp.sum(routing_sum_mat,2) <= 1]
    lin_consts += [cp.sum(routing_sum_mat,1) <= 1]

    # CVX problem
    sign_consts = [0 <= routes, routes <= 1, 0 <= slack]
    zero_var_consts = [routes[np.reshape(zero_vars, (N*N*K), 'F')] == 0]
    constraints = cone_consts + lin_consts + sign_consts + zero_var_consts
    socp = cp.Problem(cp.Maximize(slack), constraints)
    socp.solve(warm_start=True)

    return slack.value, routes.value, socp.status

###
### ROS service functions
###

def solveSOCP(req):

    res = RobustRoutingSOCPResponse()

    # check for valid message
    if len(req.config) == 0 or len(req.qos) == 0:
        rospy.logwarn('[robust_routing_socp_server] invalid reqeust')
        return res

    # unpack the message

    x = np.zeros((2,len(req.config)))
    for i in range(len(req.config)):
        x[0,i] = req.config[i].x
        x[1,i] = req.config[i].y

    slack, routes, status = rrsocpconf(req.qos, x)
    if status == 'optimal':
        res.slack = slack
        res.routes = routes
        #N = len(req.config)
        #K = len(req.qos)
        #routes = np.reshape(res.routes, (N,N,K), 'F')
        #pdb.set_trace()

    res.status = status

    return res

def RobustRoutingSOCPServer():
    rospy.init_node('RobustRoutingSOCPServer')
    s = rospy.Service('RobustRoutingSOCP', RobustRoutingSOCP, solveSOCP)
    print "RobustRoutingSOCPServer ready"
    rospy.spin()

if __name__ == "__main__":
    cm = channel_model(True, True)
    RobustRoutingSOCPServer()
