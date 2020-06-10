import numpy as np
from math import pi
import random
import cvxpy as cp
import matplotlib.pyplot as plt
from scipy.linalg import null_space
from socp.rr_socp import ChannelModel
from socp.rr_socp_tests import plot_config


class ConnectivityOpt:
    def __init__(self, channel_model, x_task, x_comm):
        self.cm = channel_model
        self.x_task = x_task
        self.x_comm = x_comm
        self.config = np.vstack((self.x_task, self.x_comm))
        self.comm_count = self.x_comm.shape[0]
        self.agent_count = self.config.shape[0]
        self.comm_idcs = range(self.x_task.shape[0], self.agent_count)

    @classmethod
    def connectivity(cls, channel_model, x_task, x_comm):
        config = np.vstack((x_task, x_comm))
        rate, _ = channel_model.predict(config)
        lap = np.diag(np.sum(rate, axis=1)) - rate
        v, _ = np.linalg.eigh(lap)
        return v[1]

    def update_network_config(self, step_size):

        gamma = cp.Variable((1))
        x = cp.Variable((self.comm_count, 2))
        Aij = cp.Variable((self.agent_count, self.agent_count))
        Lij = cp.Variable((self.agent_count, self.agent_count))

        # relative closeness

        x_dist = x - self.config[self.comm_idcs]
        constraints = [-step_size <= x_dist, x_dist <= step_size]

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

        if prob.status != 'optimal':
            print(prob.status)

        self.x_comm = x.value
        self.config[self.comm_idcs] = self.x_comm
        return ConnectivityOpt.connectivity(self.cm, self.x_task, self.x_comm)

    # for use on a static task team only
    def maximize_connectivity(self, step_size=0.2, tol=1e-10, max_its=1000, viz=False):

        if viz:
            fig, axes = plt.subplots(1,2)
            l2_hist = np.zeros((1,))
            l2_hist[0] = ConnectivityOpt.connectivity(self.cm, self.x_task, self.x_comm)
            task_ids = set(range(self.agent_count)) - set(self.comm_idcs)

        update = 1.0
        lambda2_prev = 0.0
        it = 0
        while update > tol and it < 1000:
            lambda2 = self.update_network_config(step_size)
            if viz:
                l2_hist = np.append(l2_hist, [lambda2])
                plot_config(self.config, ax=axes[0], clear_axes=True, show=False,
                            task_ids=task_ids, title="it: {:3d}, l2 = {:.3f}".format(it, lambda2))
                axes[1].cla()
                axes[1].plot(range(0,it+2), l2_hist, 'r', linewidth=2)
                axes[1].set_title("update = {:.4e}".format(lambda2-lambda2_prev))
                plt.tight_layout()
                plt.pause(0.01)
            update = lambda2 - lambda2_prev
            lambda2_prev = lambda2
            it += 1

        if viz:
            plot_config(self.config, ax=axes[0], clear_axes=True, show=False,
                        task_ids=task_ids, title="total its: {:3d}".format(it-1))
            axes[1].cla()
            axes[1].plot(range(0,it+1), l2_hist, 'r', linewidth=2)
            axes[1].set_title("l2 = {:.3f}".format(lambda2))
            plt.tight_layout()
            plt.show()

        return lambda2
