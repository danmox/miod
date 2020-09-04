import numpy as np
from scipy import special, spatial, stats


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


class PiecewiseChannel(ChannelModel):
    def __init__(self, print_values=True, n0=-70.0, n=2.52, l0=-53.0, a=0.2, b=6.0,
                 transition_dist=16.4):
        ChannelModel.__init__(self, print_values, n0, n, l0, a, b)
        self.transition_dist = transition_dist

        # find the slope of R(xi, xj) at the cuttoff distance; this will serve
        # as the slope of the linear section that decays to zero
        dRdd = ChannelModel.derivative(self, np.asarray([transition_dist, 0.0]), np.zeros((2,)))
        self.m = dRdd[0].item() # by construction dR/dy = 0; dR/dx = slope (change in distance)

        # find the y-intercept of the linear portion
        trans_rate, _ = ChannelModel.predict_link(self, np.asarray([transition_dist, 0.0]), np.zeros((2,)))
        self.b = trans_rate - self.m * transition_dist

        self.cutoff_dist = - self.b / self.m # when the rate drops to zero

    def predict(self, x):
        rate, var = ChannelModel.predict(self, x)

        edm = spatial.distance_matrix(x, x)
        dist_mask =  edm > self.transition_dist
        rate[dist_mask] = np.maximum(self.m * edm[dist_mask] + self.b, np.zeros(edm[dist_mask].size))

        # TODO zero out variance when rate is zero?

        return rate, var

    def predict_link(self, xi, xj):
        rate, var = ChannelModel.predict_link(self, xi, xj)

        dist = np.linalg.norm(xi - xj)
        if dist > self.transition_dist:
            rate = max(self.m * dist + self.b, 0.0)

        # TODO zero out variance when rate hits zero?
        # if rate < 0.0:
        #     var = 0.0

        return rate, var

    def derivative(self, xi, xj):
        diff = xi - xj
        dist = np.linalg.norm(diff)

        if dist > self.cutoff_dist:
            return np.zeros((2,1))
        elif dist > self.transition_dist:
            return self.m / dist * diff
        else:
            return ChannelModel.derivative(self, xi, xj)


class LinearChannel:
    def __init__(self, print_values=False, max_range=30.0):
        self.max_range = max_range
        if print_values is True:
            print('max_range = %.3f' % self.max_range)

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

        dist = spatial.distance_matrix(x, x)
        rate = np.maximum(-1.0/self.max_range * dist + 1.0, 0.0)
        rate[np.eye(dist.shape[0], dtype=bool)] = 0.0
        var = np.zeros(dist.shape)
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

        rate = max(-1.0/self.max_range * np.linalg.norm(xi - xj) + 1.0, 0.0)
        var = 0.0
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

        diff = xi - xj
        dist = np.linalg.norm(xi - xj)
        if dist > self.max_range:
            return np.zeros((2,1))
        return -1.0 / self.max_range * diff / dist
