#!/usr/bin/env python

import rr_socp
import numpy as np
from socp.srv import RobustRoutingSOCP, RobustRoutingSOCPResponse
import rospy


class RRSOCPServer:
    def __init__(self, fetch_params=True, print_values=True, n0=-70.0, n=2.52, l0=-53.0, a=0.2, b=6.0):
        if fetch_params:
            n0 = rospy.get_param("/N0")
            n = rospy.get_param("/n")
            l0 = rospy.get_param("/L0")
            a = rospy.get_param("/a")
            b = rospy.get_param("/b")
            self.solver = rr_socp.RobustRoutingSolver(print_values=print_values, n0=n0, n=n, l0=l0, a=a, b=b)
        else:
            self.solver = rr_socp.RobustRoutingSolver(print_values=print_values, n0=n0, n=n, l0=l0, a=a, b=b)

    def solve_socp(self, req):
        res = RobustRoutingSOCPResponse()

        # check for valid request
        if len(req.config) == 0 or len(req.flows) == 0:
            rospy.logwarn('[robust_routing_socp_server] invalid reqeust')
            return res

        x = np.zeros((len(req.config), 2))
        for i in range(len(req.config)):
            x[i, 0] = req.config[i].x
            x[i, 1] = req.config[i].y

        slack, routes, status = self.solver.solve_socp(req.flows, x, req.idx_to_id)
        if status == 'optimal':
            res.obj_fcn = slack
            res.routes = routes
            res.qos = qos

        res.status = status

        return res


if __name__ == "__main__":
    rospy.init_node('robust_routing_socp_server')

    socp_server = RRSOCPServer(fetch_params=True, print_values=True)
    srv = rospy.Service('RobustRoutingSOCP', RobustRoutingSOCP, socp_server.solve_socp)

    rospy.loginfo("robust_routing_socp_server ready")
    rospy.spin()
