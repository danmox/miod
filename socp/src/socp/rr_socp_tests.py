import copy
import sys
import numpy as np
from . import channel_model
from . import rr_socp
from . import rr_socp_server
import time
from socp.srv import RobustRoutingSOCPRequest
from socp.msg import Flow
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import cm
from matplotlib.colors import Normalize


# helps the figures to be readable on hidpi screens
mpl.rcParams['figure.dpi'] = 200

def numpy_to_ros(np_config, z=0.):
    """
    convert a Nx2 numpy array of 2D node positions to a list of
    geometry_msgs.Points

    """
    ros_config = [Point(np_config[i,0], np_config[i,1], z) for i in range(np_config.shape[0])]
    return ros_config

def plot_config(config, ax=None, pause=None, clear_axes=False, show=True,
                title=None, ids=None, task_ids=None, routes=None, rates=None):
    """
    plot the 2D spatial configuration of the network

    Input:
      config - a list of geometry_msgs.Point msgs
      Optional Args:
        ax         - axes to plot on
        pause      - avoids blocking by continuing after a short pause
        clear_axes - clear ax before plotting
        show       - call plt.show()
        title      - string of text to set as title of figure
        ids        - as list of ids to use for agent labels
        task_ids   - ids of task agents
        routes     - draw lines denoting route usage between nodes
        rates      - draw lines denoting rates between agents
    """
    if type(config) is list:
        x = np.asarray([pt.x for pt in config])
        y = np.asarray([pt.y for pt in config])
    else:
        x = config[:,0]
        y = config[:,1]

    if ax is None:
        fig, ax = plt.subplots()

    if clear_axes:
        ax.cla()

    if ids is None:
        ids = range(len(x))
    elif type(ids) is not list:
        ids = list(ids)

    if task_ids is None:
        id_mask = np.asarray(len(ids)*[True], dtype=bool)
    else:
        id_mask = np.asarray([True if id in task_ids else False for id in ids], dtype=bool)

    # draw routes between each agent

    ax.axis('scaled')
    bbx = np.asarray([min(x), max(x), min(y), max(y)])
    window_scale = np.max(bbx[1::2] - bbx[0::2])
    ax.axis(bbx + np.asarray([-1, 1, -1, 1])*0.1*window_scale)

    if routes is not None:

        # form dict of all route lines to be plotted later
        cumulative_routes = np.sum(routes, 2)
        lines = []
        for i, j in [(i,j) for i in range(len(x)) for j in range(i+1,len(x))]:
            Pi = np.asarray([x[i], y[i]])
            Pj = np.asarray([x[j], y[j]])
            Aij = cumulative_routes[i,j]
            Aji = cumulative_routes[j,i]

            # ensures the arrows are oriented correctly
            if Pj[0] < Pi[0] or Pj[1] < Pi[1]:
                Pi, Pj = Pj, Pi
                Aij, Aji = Aji, Aij

            a1 = np.arctan2(Pj[1]-Pi[1], Pj[0]-Pi[0])
            a2 = np.arctan2(Pi[1]-Pj[1], Pi[0]-Pj[0])

            # line segment endpoints
            ds = np.pi / 16.0
            scale = 0.03 * window_scale
            l1 = np.zeros((2,2))
            l2 = np.zeros((2,2))
            l1[0,:] = Pi + scale*np.asarray([np.cos(a1+ds), np.sin(a1+ds)])
            l1[1,:] = Pj + scale*np.asarray([np.cos(a2-ds), np.sin(a2-ds)])
            l2[0,:] = Pi + scale*np.asarray([np.cos(a1-ds), np.sin(a1-ds)])
            l2[1,:] = Pj + scale*np.asarray([np.cos(a2+ds), np.sin(a2+ds)])

            # arrowhead endpoint
            ds = np.pi / 8.0
            scale = 0.04 * window_scale
            h1 = l1[0,:] + scale*np.asarray([np.cos(a1+ds), np.sin(a1+ds)])
            h2 = l2[1,:] + scale*np.asarray([np.cos(a2+ds), np.sin(a2+ds)])

            if Aji > 0.01:
                lines.append({'rate': Aji, 'line_x': l1[:,0], 'line_y': l1[:,1],
                              'arrow_x': [l1[0,0], h1[0]], 'arrow_y': [l1[0,1], h1[1]]})
            if Aij > 0.01:
                lines.append({'rate': Aij, 'line_x': l2[:,0], 'line_y': l2[:,1],
                              'arrow_x': [l2[1,0], h2[0]], 'arrow_y': [l2[1,1], h2[1]]})

        # plot lines by line weight: faintest on the bottom, boldest on top
        lw = 2
        cmap = cm.ScalarMappable(norm=Normalize(vmin=0.0, vmax=1.0, clip=True), cmap='YlOrBr')
        lines.sort(key=lambda line_dict: line_dict['rate'])
        for d in lines:
            ax.plot(d['line_x'], d['line_y'], lw=lw, c=cmap.to_rgba(d['rate']))
            ax.plot(d['arrow_x'], d['arrow_y'], lw=lw, c=cmap.to_rgba(d['rate']))

    if routes is None and rates is not None:
        cmap = cm.ScalarMappable(norm=Normalize(vmin=0.0, vmax=1.0, clip=True), cmap='cool')
        for i, j in [(i,j) for i in range(len(x)) for j in range(i+1,len(x))]:
            if rates[i,j] == 0.0:
                continue

            Pi = np.asarray([x[i], y[i]])
            Pj = np.asarray([x[j], y[j]])

            a1 = np.arctan2(Pj[1]-Pi[1], Pj[0]-Pi[0])
            a2 = np.arctan2(Pi[1]-Pj[1], Pi[0]-Pj[0])

            # line segment endpoints
            scale = 0.04 * window_scale
            line = np.zeros((2,2))
            line[0,:] = Pi + scale*np.asarray([np.cos(a1), np.sin(a1)])
            line[1,:] = Pj + scale*np.asarray([np.cos(a2), np.sin(a2)])

            ax.plot(line[:,0], line[:,1], lw=2, c=cmap.to_rgba(rates[i,j]))

    # plot agent positions as circles

    if len(x) != len(id_mask):
        import pdb;pdb.set_trace()
    ax.plot(x[id_mask], y[id_mask], 'ro', ms=16, fillstyle='none',
            markeredgewidth=2, label='task')
    ax.plot(x[~id_mask], y[~id_mask], 'bo', ms=16, fillstyle='none',
            markeredgewidth=2, label='network')
    ax.legend(markerscale=0.6)

    # add agent ID in middle of circle

    for i in range(len(x)):
        color= 'r' if id_mask[i] == True else 'b'
        ax.annotate(str(ids[i]), (x[i], y[i]-0.05), color=color,
                    horizontalalignment='center', verticalalignment='center')

    if title is not None:
        ax.set_title(title)

    if show:
        if pause is None:
            plt.show()
        else:
            plt.pause(pause)


def socp_info(routes, flow, config=None, solver=None, ids=None):
    """
    print information about the robust routing solution

    Input:
      routes: an NxNxK array of routing variables
      flow: an array of flow requirements with length(flow) == K
      config: (optional) the configuration of the team to plot
      solver: (optional) will print the channel rate, var if the config is provided
      ids: (optional) node ids to use instead of 1,...,n

    """
    assert len(flow) == routes.shape[2]
    n = routes.shape[0]
    if ids is not None:
        assert len(ids) == n
    else:
        ids = range(n)
    id_to_idx = {id: idx for id, idx in zip(ids, range(n))}
    idx_to_id = {idx: id for id, idx in zip(ids, range(n))}

    # TODO clean up printing
    for k in range(len(flow)):
        # flow info
        print('flow %d: %d -> %d, rate = %.2f, qos = %.2f:'\
              % (k+1, flow[k].src, flow[k].dest, flow[k].rate, flow[k].qos))
        # column header
        node_names = ['%6s' % i for i in ids]
        node_names[id_to_idx[flow[k].src]] = '%6s' % ('(s) ' + str(flow[k].src))
        node_names[id_to_idx[flow[k].dest]] = '%6s' % ('(d) ' + str(flow[k].dest))
        print('   %6s|%s|%5s' % (' ', ''.join(map(str, node_names)), 'Tx'))
        print('   %s' % ('-' * (6 * (n+2) + 1)))
        for i in range(n):
            num_str = ['%6.2f' % a if a > 0.01 else '%6s' % '-' for a in routes[i, :, k]]
            if sum(routes[i, :, k]) > 0.01:
                num_str += ['|%5.2f' % sum(routes[i, :, k])]
            else:
                num_str += ['|%5s' % '-']
            print('   %6s|%s' % (node_names[i], ''.join(num_str)))
        print('   %6s|%s|' % ('', '-' * (6*n)))
        rx_str = ['%6.2f' % sum(routes[:, i, k]) if sum(routes[:, i, k]) > 0.01 else '%6s' % '-' for i in range(n)]
        print('   %6s|%s' % ('Rx', ''.join(rx_str)))

    if config is not None and solver is not None:
        rate_mean, rate_var = solver.cm.predict(config)
        with np.printoptions(precision=3, suppress=True):
            print('rate_mean:')
            print(rate_mean)
            print('rate_var:')
            print(rate_var)


def speed_test():
    print('running speed test\n')

    patrol_rad = 20
    comm_rad = 8
    task_count = 3
    comm_count = 6

    rate = 0.10
    conf = 0.70
    ids = [0, 1, 2]
    flows = [Flow(rate, s, d, Flow.CONFIDENCE, conf) for s in ids for d in ids if d != s]

    # build team structure

    x_task = np.zeros((task_count, 2))
    x_task[:,0] = patrol_rad * np.cos(2.0*np.pi/task_count * np.arange(0,task_count))
    x_task[:,1] = patrol_rad * np.sin(2.0*np.pi/task_count * np.arange(0,task_count))
    x_comm = np.zeros((comm_count, 2))
    x_comm[1:,0] = comm_rad * np.cos(2.0*np.pi/(comm_count - 1) * np.arange(0,comm_count-1))
    x_comm[1:,1] = comm_rad * np.sin(2.0*np.pi/(comm_count - 1) * np.arange(0,comm_count-1))
    x = np.vstack([x_task, x_comm])

    # solve SOCP

    cm = channel_model.ChannelModel(print_values=True)
    rrsolver = rr_socp.RobustRoutingSolver(cm)
    for i in range(10):
        start = time.time()
        slack, routes, status = rrsolver.solve_socp(flows, x)
        print('found solution in {:.4f} second'.format(time.time() - start))


def simple_routing_test(rate=0.1, conf=0.7):
    print('running simple_routing_test()\n')
    msg = RobustRoutingSOCPRequest()

    x = [0.0, 30.0, 10.0, 20.0]
    y = [0.0, 0.0, 3.0, -3.0]
    task_ids = [0, 1]
    msg.config = [Point(x[i], y[i], 0.0) for i in range(len(x))]

    print('rate = {:.3f}, confidence = {:.3f}'.format(rate, conf))
    msg.flows = [Flow(rate, 1, 0, Flow.CONFIDENCE, conf)]
    msg.flows += [Flow(rate, 0, 1, Flow.CONFIDENCE, conf)]

    rrserver = rr_socp_server.RRSOCPServer(fetch_params=False)
    res = rrserver.solve_socp(msg)
    if res.status != 'optimal':
        print('solve_socp returned with status: {}'.format(res.status))
        return

    N = len(x)
    K = len(msg.flows)
    routes = np.reshape(res.routes, (N,N,K), 'F')
    print('slack = {:.4f}'.format(res.obj_fcn))
    socp_info(routes, msg.flows, msg.config)
    plot_config(msg.config, task_ids=task_ids, routes=routes)


def infeasible_test(rate=0.5, confidence=0.9):
    print('running simple_routing_test()\n')
    msg = RobustRoutingSOCPRequest()

    x = [0.0, 30.0, 10.0, 20.0]
    y = [0.0, 0.0, 3.0, -3.0]
    msg.config = [Point(x[i], y[i], 0.0) for i in range(len(x))]

    print('rate = {:.3f}, confidence = {:.3f}'.format(rate, confidence))
    msg.flows = [Flow(rate, 1, 0, Flow.CONFIDENCE, confidence)]

    rrserver = rr_socp_server.RRSOCPServer(fetch_params=False)
    res = rrserver.solve_socp(msg)
    if res.status != 'optimal':
        print('solve_socp returned with status: {}'.format(res.status))
        return

    N = len(x)
    K = len(msg.flows)
    routes = np.reshape(res.routes, (N,N,K), 'F')
    print('slack = {:.4f}'.format(res.obj_fcn))
    socp_info(routes, msg.flows, msg.config)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print('running all tests\n')
        speed_test()
        print('\n')
        simple_routing_test()
        print('\n')
        infeasible_test()
    else:
        arg = int(sys.argv[1])
        if arg == 1:
            speed_test()
        elif arg == 2:
            simple_routing_test()
        elif arg == 3:
            infeasible_test()
        else:
            print('unkown argument {}'.format(arg))
