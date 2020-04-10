import numpy as np
from math import pi
import random
import cvxpy as cp
from scipy.linalg import null_space
from socp.rr_socp import ChannelModel


class ConnectivityOpt:
    def __init__(self, x_task=None, x_comm=None,
                 print_values=False, n0=-70.0, n=2.52, l0=-53.0, a=0.2, b=6.0):
        self.cm = ChannelModel(print_values=print_values, n0=n0, n=n, l0=l0, a=a, b=b)
        self.x_task = x_task
        self.x_comm = x_comm
        if x_task is not None and x_comm is not None:
            self.config = np.vstack((self.x_task, self.x_comm))
            self.agent_count = self.config.shape[0]
            self.comm_count = self.x_comm.shape[0]
            self.comm_idcs = range(self.x_task.shape[0], self.agent_count)

    def connectivity(self):
        rate, _ = self.cm.predict(self.config)
        lap = np.diag(np.sum(rate, axis=1)) - rate
        v, _ = np.linalg.eigh(lap)
        return v[1]

    def set_team_config(self, x_task, x_comm):
        self.x_task = x_task
        self.x_comm = x_comm
        self.config = np.vstack((self.x_task, self.x_comm))
        self.agent_count = self.config.shape[0]
        self.comm_count = self.x_comm.shape[0]
        self.comm_idcs = range(self.x_task.shape[0], self.agent_count)

    # TODO use cp.parameter?
    def update_network_config(self, step_size):

        gamma = cp.Variable((1))
        x = cp.Variable((self.comm_count, 2))
        Aij = cp.Variable((self.agent_count, self.agent_count))
        Lij = cp.Variable((self.agent_count, self.agent_count))

        # relative closeness

        constraints = [cp.norm(x - self.config[self.comm_idcs], 1) <= step_size * self.comm_count]

        # linearized rate model

        for i in range(self.agent_count):
            for j in range(self.agent_count):
                if i == j:
                    constraints += [Aij[i,j] == 0.0]
                    continue
                xi = self.config[i,:]
                xj = self.config[j,:]
                Rxixj, _ = self.cm.predict_link(xi, xj)
                dRdxi = self.cm.derivative(xi, xj)
                dRdxj = self.cm.derivative(xj, xi)
                if i in self.comm_idcs and j in self.comm_idcs:
                    xi_var = x[self.comm_idcs.index(i),:]
                    xj_var = x[self.comm_idcs.index(j),:]
                    constraints += [Aij[i,j] == Rxixj + dRdxi.T * (xi_var - xi).T \
                                                      + dRdxj.T * (xj_var - xj).T]
                elif i in self.comm_idcs:
                    xi_var = x[self.comm_idcs.index(i),:]
                    constraints += [Aij[i,j] == Rxixj + dRdxi.T * (xi_var - xi).T]
                elif j in self.comm_idcs:
                    xj_var = x[self.comm_idcs.index(j),:]
                    constraints += [Aij[i,j] == Rxixj + dRdxj.T * (xj_var - xj).T]
                else:
                    constraints += [Aij[i,j] == Rxixj]

        # graph laplacian

        constraints += [Lij == cp.diag(cp.sum(Aij, axis=1)) - Aij]

        # 2nd smallest eigen value

        P = null_space(np.ones((1, self.agent_count)))
        constraints += [P.T * Lij * P >> gamma*np.eye(self.agent_count-1)]

        #
        # solve problem
        #

        prob = cp.Problem(cp.Maximize(gamma), constraints)
        prob.solve()

        if prob.status is not 'optimal':
            print(prob.status)

        self.x_comm = x.value
        self.config[self.comm_idcs] = self.x_comm
        return self.connectivity()
