import numpy as np
from math import pi, floor, log10
import random
import cvxpy as cp
import matplotlib.pyplot as plt
from scipy.linalg import null_space
from socp.channel_model import ChannelModel
from socp.rr_socp_tests import plot_config


def round_sf(x, significant_figures):
    if x != 0.0:
        return round(x, -int(floor(log10(abs(x))))+significant_figures-1)
    else:
        return 0.0


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

    def get_comm_config(self):
        return self.config[self.comm_idcs]

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
                    constraints += [Aij[i,j] == Rxixj + dRdxi.T @ (xi_var - xi).T \
                                                      + dRdxj.T @ (xj_var - xj).T]
                elif i in self.comm_idcs:
                    xi_var = x[self.comm_idcs.index(i),:]
                    constraints += [Aij[i,j] == Rxixj + dRdxi.T @ (xi_var - xi).T]
                elif j in self.comm_idcs:
                    xj_var = x[self.comm_idcs.index(j),:]
                    constraints += [Aij[i,j] == Rxixj + dRdxj.T @ (xj_var - xj).T]
                else:
                    constraints += [Aij[i,j] == Rxixj]

        # graph laplacian

        constraints += [Lij == cp.diag(cp.sum(Aij, axis=1)) - Aij]

        # 2nd smallest eigen value

        P = null_space(np.ones((1, self.agent_count)))
        constraints += [P.T @ Lij @ P >> gamma*np.eye(self.agent_count-1)]

        #
        # solve problem
        #

        prob = cp.Problem(cp.Maximize(gamma), constraints)
        prob.solve()

        # only update the configuration if a new optimum was found
        #
        # NOTE with agressive step sizes the result of the optimization may be
        # a configuration that has a worse value for connectivity; in this case
        # the network team configuration should not be updated at the calling
        # function should be notified that the optimization failed

        conn_prev = ConnectivityOpt.connectivity(self.cm, self.x_task, self.x_comm)

        if prob.status != 'optimal':
            return conn_prev, False

        conn_new  = ConnectivityOpt.connectivity(self.cm, self.x_task, x.value)

        if conn_new < conn_prev:
            return conn_prev, False

        self.x_comm = x.value
        self.config[self.comm_idcs] = self.x_comm
        return conn_new, True


    # for use on a static task team only
    def maximize_connectivity(self, step_size=2.0, tol=1e-6, max_its=1000, viz=False):

        if viz:
            fig, axes = plt.subplots(1,2)
            task_ids = set(range(self.agent_count)) - set(self.comm_idcs)

        # track lambda 2 over time for stopping crit and visualization
        l2_hist = np.zeros((1,))
        l2_hist[0] = ConnectivityOpt.connectivity(self.cm, self.x_task, self.x_comm)

        for it in range(max_its):
            lambda2, success = self.update_network_config(step_size)

            # the optimization failed, most likely due to an agressive step size
            if not success:
                step_size *= 0.5
                continue

            # check if change in lambda 2 has "flatlined"
            l2_hist = np.append(l2_hist, [lambda2])
            m = np.polyfit(range(l2_hist[-10::].shape[0]), l2_hist[-10::], 1)[0]

            if viz:
                plot_config(self.config, ax=axes[0], clear_axes=True, show=False,
                            task_ids=task_ids,
                            title=f'ss = {round_sf(step_size,2)}, m = {round_sf(m,2)}')
                axes[1].cla()
                axes[1].plot(l2_hist, 'r', linewidth=2)
                axes[1].set_title(f'it = {it}, update = {round_sf(np.diff(l2_hist[-2::])[0],2)}')
                plt.tight_layout()
                plt.pause(0.01)

            # stopping criterion
            if abs(m) < tol:
                break

        if viz:
            plt.show()

        return lambda2
