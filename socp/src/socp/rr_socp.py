#!/usr/bin/env python

import numpy as np
from scipy import special, spatial, stats
import cvxpy as cp


#
# Robust Routing SOCP functions
#

def dbm2mw(dbm):
    return 10.0 ** (np.asarray(dbm) / 10.0)


class ChannelModel:
    def __init__(self, print_values=True, n0=-70.0, n=2.52, l0=-53.0, a=0.2, b=6.0):
        self.L0 = l0                # transmit power (dBm)
        self.n = n                  # decay rate
        self.N0 = n0                # noise at receiver (dBm)
        self.a = a                  # sigmoid parameter 1
        self.b = b                  # sigmoid parameter 2
        self.PL0 = dbm2mw(self.L0)  # transmit power (mW)
        self.PN0 = dbm2mw(self.N0)  # noise at receiver (mW)
        if print_values is True:
            print('L0 = %.3f' % self.L0)
            print('n  = %.3f' % self.n)
            print('N0 = %.3f' % self.N0)
            print('a  = %.3f' % self.a)
            print('b  = %.3f' % self.b)

    def predict(self, x):
        """
        compute the expected channel rates and variances for each link in the
        network with node positions given by x

        Inputs:
          x: a Nx2 list of node positions [x y]

        Outputs:
          rate: matrix of expected channel rates between each pair of agents
          var: matrix of channel rate variances between each pair of agents
        """

        d = spatial.distance_matrix(x, x)
        dist_mask = ~np.eye(d.shape[0], dtype=bool)
        power = np.zeros(d.shape)
        power[dist_mask] = dbm2mw(self.L0 - 10 * self.n * np.log10(d[dist_mask]))
        rate = special.erf(np.sqrt(power / self.PN0))
        var = (self.a * d / (self.b + d)) ** 2
        return rate, var

    def predict_link(self, xi, xj):
        """
        compute the expected channel rate and variance of a single link

        Inputs:
          xi: 1x2 node position
          xj: 1x2 node position

        Outputs:
          rate: expected channel rate between xi, xj
          var: variance ("confidence") in expected channel rate between xi, xj
        """

        d = np.linalg.norm(xi - xj)
        power = dbm2mw(self.L0 - 10 * self.n * np.log10(d))
        rate = special.erf(np.sqrt(power / self.PN0))
        var = (self.a * d / (self.b + d)) ** 2
        return rate, var

    def derivative(self, xi, xj):
        """
        compute the derivative of channel rate function with respect to xi (note:
        the derivative with respect to xj can be found by swapping the inputs)

        Inputs:
          xi: [x y] node position
          xj: [x y] node position

        Outputs:
          der: 2x1 derivative of Rij w.r.t xi
        """

        xi = np.reshape(xi, (2,1))
        xj = np.reshape(xj, (2,1))

        dist = np.linalg.norm(xi - xj)
        if dist < 1e-6:
            return np.zeros((2,1))
        der = - 10.0**(self.L0/20) * self.n * np.sqrt(dist ** (-self.n) / self.PN0) \
            * np.exp(-10.0**(self.L0 / 10.0) * dist**(-self.n) / self.PN0) \
            / (np.sqrt(np.pi) * dist) * (xi - xj) / dist
        return der


class RobustRoutingSolver:
    def __init__(self, print_values=True, n0=-70.0, n=2.52, l0=-53.0, a=0.2, b=6.0):
        self.cm = ChannelModel(print_values=print_values, n0=n0, n=n, l0=l0, a=a, b=b)

    def solve_socp(self, flows, x, idx_to_id=None, reshape_routes=False):
        """
        solve the robust routing problem for the given network requirements and
        configuration

        Inputs:
          flows: a list of socp.Flow messages
          x: a Nx2 list of node positions [x y]
          reshape_routes: returns routing variables in NxNxK matrix form

        Outputs:
          slack: slack of the associated robust routing solution
          routes: optimal routing variables
          status: 'optimal' if a soln was found
        """

        N = x.shape[0]
        P = len(flows)

        assert x.shape[1] == 2

        # if no idx_to_id list is provided it is assumed idx == id
        # if idx_to_id is provided there must be a entry for each index
        if idx_to_id is None or len(idx_to_id) == 0:
            idx_to_id = range(N)
        else:
            assert len(idx_to_id) == N
        id_to_idx = {idx_to_id[i]: i for i in range(N)}

        # socp constraints
        a_mat, b_mat, zero_vars, conf, m_ik = self.cone_constraints(flows, x, id_to_idx)

        # optimization variables
        slack, routes = cp.Variable((1)), cp.Variable((N*N*P))
        y = cp.hstack([routes, slack])

        # cone constraints
        cone_consts = []
        for i in range(a_mat.shape[0]):
            cone_consts += [cp.SOC((cp.matmul(b_mat[i,:], y) - m_ik[i]) / conf[i], cp.matmul(cp.diag(a_mat[i,:]), y))]

        # linear availability constraints
        routing_sum_mat = cp.reshape(routes[:N*N], (N,N))  # acts like np.reshape with 'F'
        for k in range(1, P):  # summing over dim 3
            routing_sum_mat += cp.reshape(routes[k*N*N:(k+1)*N*N], (N,N))
        lin_consts = [cp.sum(routing_sum_mat, 2) <= 1]
        lin_consts += [cp.sum(routing_sum_mat, 1) <= 1]

        # solve program with CVX
        sign_consts = [0 <= routes, routes <= 1, 0 <= slack]
        zero_var_consts = [routes[np.reshape(zero_vars, (N*N*P), 'F')] == 0]
        constraints = cone_consts + lin_consts + sign_consts + zero_var_consts
        socp = cp.Problem(cp.Maximize(slack), constraints)
        socp.solve()

        if socp.status is 'optimal':
            qos_delivered = np.zeros((2*P))
            for k in range(P):
                idx = N*k + id_to_idx[qos[k].src]
                qos_delivered[2*k] = np.dot(b_mat[idx, :-1], routes.value)
                qos_delivered[2*k+1] = np.linalg.norm(a_mat[idx, :-1] * routes.value) * conf[idx]

            if reshape_routes:
                routing_vars = np.reshape(routes.value, (N,N,P), 'F')
            else:
                routing_vars = routes.value
            return slack.value[0], routing_vars, socp.status, qos_delivered

        return None, None, socp.status, None

    def cone_constraints(self, flows, x, id_to_idx):
        """
        compute the coefficient matrices and vectors of the 2nd order
        constraints of the robust routing problem.

        2nd order cone constraints of the form:
        ||A*y + b|| <= c^T*y + d
        are equivalent to the node margin constraints:
        (B*y - m_ik) / ||A*y|| >= I(eps)
        where y is the vector of optimization variables and I(eps) is the
        inverse normal cumulative distribution function.

        Inputs:
          flows: a list of socp.Flow messages
          x: a Nx2 list of node positions [x y]
          id_to_idx: a map between IDs in flows and indices in x

        Outputs:
          a_mat: variance coefficient matrix
          b_mat: mean coefficient matrix
          zero_vars: which optimization variables can safely be set to zero
          conf: the probabilistic confidence of each constraint
          m_ik: the required rate margin of each constraint
        """

        assert x.shape[1] == 2

        N = x.shape[0]
        P = len(flows)

        # predicted channel rates
        rate_mean, rate_var = self.cm.predict(x)

        # variables that should be zero
        zero_vars = np.reshape(np.eye(N, dtype=bool), (N,N,1), 'F')
        zero_vars = np.repeat(zero_vars, P, axis=2)

        # node margin constraints

        a_mat = np.zeros(((N-1)*P, N*N*P+1))
        b_mat = np.zeros(((N-1)*P, N*N*P+1))
        m_ik = np.zeros(((N-1)*P))

        idx = 0
        for k in range(P):
            for i in range(N):
                if i == id_to_idx[flows[k].dest]:
                    continue

                if i == id_to_idx[flows[k].src]:
                    m_ik[idx] = flows[k].rate

                aki = np.zeros((N,N))
                bki = np.zeros((N,N))

                aki[:,i] = np.sqrt(rate_var[:,i]) # incoming
                aki[i,:] = np.sqrt(rate_var[i,:]) # outgoing
                aki[zero_vars[:,:,k]] = 0.0

                bki[:,i] = -rate_mean[:,i] # incoming
                bki[i,:] =  rate_mean[i,:]  # outgoing
                bki[zero_vars[:,:,k]] = 0.0

                a_mat[idx, k*N*N:(k+1)*N*N] = np.reshape(aki, (1,-1), 'F')
                b_mat[idx, k*N*N:(k+1)*N*N] = np.reshape(bki, (1,-1), 'F')
                b_mat[idx, N*N*P] = -1

                idx += 1

        # probabilistic confidence requirements

        conf = stats.norm.ppf(np.repeat([f.qos for f in flows], N-1))

        return a_mat, b_mat, zero_vars, conf, m_ik
