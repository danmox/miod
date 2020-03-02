import json
import os
import subprocess
import tempfile
import time
import datetime
from math import ceil
from os.path import expanduser

import networkx as nx
from haversine import haversine
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.axes import Axes
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D, proj3d
import numpy as np
import re
import pexpect




def rsync(server, path, password, local_results_folder, timeout=30):
    """SSH'es to a host using the supplied credentials and executes a command.
    Throws an exception if the command doesn't return 0.
    bgrun: run command in the background"""

    fname = tempfile.mktemp()
    fout = open(fname, 'w')


    #ssh_cmd = "scp %s:%s" % (server, path)  +" ./results/"
    rsync = "rsync -avzh %s:%s" % (server, path)  + " " + local_results_folder
    print(rsync)
    child = pexpect.spawn(rsync, timeout=timeout)
    child.expect(['password: ', pexpect.EOF])
    child.sendline(password)
    child.expect(pexpect.EOF)
    child.logfile = fout
    child.close()
    fout.close()

    fin = open(fname, 'r')
    stdout = fin.read()
    print(stdout)
    fin.close()

    if child.exitstatus!=0:
        print(child.before)
        raise Exception(stdout)

    return stdout


def moving_average_inf(a, x_values, n=20, period=1) :
    mv_average_filter = np.zeros(n)
    j=0
    ret = np.zeros(len(a))
    prev_val = x_values[0]
    cur_per = 0
    for i in range(0,len(a)):
        if i!=0:
            cur_per = x_values[i]-prev_val
            prev_val = x_values[i]
        mv_average_filter[j] = a[i]
        if cur_per>period:
            print(cur_per)
            mv_average_filter[j] = period*1000
        j += 1
        j = j % n
        if i<=n:
            ret[i]=np.mean(mv_average_filter[:i+1])
        else:
            ret[i]=np.mean(mv_average_filter)
    return ret

def moving_average(a, n=20) :
    mv_average_filter = np.zeros(n)
    j=0
    ret = np.zeros(len(a))
    for i in range(0,len(a)):
        mv_average_filter[j] = a[i]
        j += 1
        j = j % n
        if i<=n:
            ret[i]=np.mean(mv_average_filter[:i+1])
        else:
            ret[i]=np.mean(mv_average_filter)

    return ret


save_archive = True
load_from_archive=False
load_local = False
archive_name = 'results/RRarchive2019-11-14 16:21.npz'
home = expanduser("~")
local_results_folder = home+"/workspace_aero/src/infrastructure-on-demand/routing_protocol/src/results/"

data = {}

if load_from_archive:
    f = np.load(archive_name, allow_pickle=True)
    data_full = f['arr_0']
    for i in data_full.item():
        data[i] = data_full.item()[i]

else:
    file_name = 'results/RR'
    local = None#'10.42.0.13'

    hosts = ['10.42.0.4','10.42.0.6', '10.42.0.7']#, '10.42.0.2', '10.42.0.10']
    passwd = ('1234567890\n').encode()
    files = {}
    for i in hosts:
        files[i] = file_name+"_"+str(i)+".npz"
        aero_id = i.split(".")[-1]
        server = "aero{}@{}".format(aero_id,i)
        path = "~/ws_intel/src/infrastructure-on-demand/routing_protocol/src/{}".format(files[i])
        if load_from_archive is not True:
            rsync(server,path,passwd,local_results_folder)

    if local:
        files[local] = file_name+"_"+local+".npz"

    for i in files:
        f = np.load(files[i], allow_pickle=True)
        data[i] = f['arr_0']

    if save_archive:
        now = datetime.datetime.now()
        archive = file_name+"archive"+now.strftime("%Y-%m-%d %H:%M")
        np.savez(archive,data)

print("Loading files complete, parsing...")
fig, tp_plot = plt.subplots()
fig2, tr_plot = plt.subplots()
fig3 = plt.figure()
fig3_prj = fig3.gca(projection='3d')
rt_plot = Axes3D(fig3)

fig4, tp_plot_ani = plt.subplots()
fig5, tr_plot_ani = plt.subplots()


colors = ['b', 'g', 'r', 'c', 'y']
colors_fixed = {}
j=0


tp_total = {}
tp_total_ma = {}
tp_total_time = {}

tr_total = {}
tr_total_ma = {}
tr_total_time = {}

rt_total = {}
rt_path_total = {}
rt_path_time = {}
pos_total = {}
rt_time_total = {}
pos_time_total = {}
max_time_rt  = 0
max_pos_x = [None, None]
max_pos_y = [None, None]
max_pos_z = [None, None]

for i in data:
    dat = data[i]
    #print(dat.item())
    print(dat.item()["tr"])
    start_time = dat.item()["start_time"]
    m_av_len = 20
    tr = dat.item()["tr"]
    if len(tr)>0:
        tp_ip = dat.item()['tp_ip']
        tr = np.transpose(np.array(tr, dtype=object))
        rt_path_total[i] = tr[1]
        print(tr)
        rt_path_time[i] = tr[0]-start_time
        y_ax = [sum(k) for k in tr[2]]
        tr_plot_ma = moving_average_inf(y_ax, tr[0]-start_time, n=m_av_len, period=1)
        tr_plot.plot(tr[0]-start_time, y_ax, color=colors[j % len(colors)], marker='.', linewidth=0, markersize=1,
                 label=('TX - {}, RX - {}'.format(i, tp_ip)))
        tr_plot.plot(tr[0]-start_time, tr_plot_ma, color=colors[j % len(colors)],
             label=('Mov. av., TX - {}, RX - {}'.format(i, tp_ip)))
        tr_total[i] = y_ax
        tr_total_ma[i] = tr_plot_ma
        tr_total_time[i] = tr[0]-start_time


    tp = dat.item()["tp"]
    if len(tp)>0:
        tp_ip = dat.item()['tp_ip']
        tp = np.transpose(tp)
        tp_plot_ma = moving_average(tp[1], m_av_len)
        tp_plot.plot(tp[0], tp[1], color=colors[j % len(colors)], marker='.', linewidth=0, markersize=1,
             label=('TX - {}, RX - {}'.format(i, tp_ip)))
        tp_plot.plot(tp[0], tp_plot_ma, color=colors[j % len(colors)],
             label=('Mov. av., TX - {}, RX - {}'.format(i, tp_ip)))
        tp_total[i] = tp[1]
        tp_total_ma[i] = tp_plot_ma
        tp_total_time[i] = tp[0]

    colors_fixed[i] = colors[j % len(colors)]
    j+=1


    rt = dat.item()["rt"]
    pos = dat.item()["pos"]
    if len(rt)>0:
        time_rt = np.array(rt).transpose()[0,:]
        rt_time_total[i] = np.array(time_rt,dtype=float) - start_time
        max_time_rt = max(max_time_rt, max(rt_time_total[i]))
        rt_total[i] = np.array(rt)[:,1:]
        print(i)
        print(rt_total[i])
    if len(pos)>0:
        time_pos = np.array(pos).transpose()[0,:]
        pos_time_total[i] = np.array(time_pos,dtype=float) - start_time
        pos_total[i] = np.array(pos)[:,1:]
        pos_total[i][:,2]=pos_total[i][:,2]
        if max_pos_x[0]!=None:
            max_pos_x[0] = min(max_pos_x[0], min(pos_total[i][:,0]))
        else:
            max_pos_x[0] = min(pos_total[i][:,0])
        if max_pos_x[1]!=None:
            max_pos_x[1] = max(max_pos_x[1], max(pos_total[i][:,0]))
        else:
            max_pos_x[1] = max(pos_total[i][:,0])

        if max_pos_y[0]!=None:
            max_pos_y[0] = min(max_pos_y[0], min(pos_total[i][:,1]))
        else:
            max_pos_y[0] = min(pos_total[i][:,1])
        if max_pos_y[1]!=None:
            max_pos_y[1] = max(max_pos_y[1], max(pos_total[i][:,1]))
        else:
            max_pos_y[1] = max(pos_total[i][:,1])

        if max_pos_z[0]!=None:
            max_pos_z[0] = min(max_pos_z[0], min(pos_total[i][:,2]))
        else:
            max_pos_z[0] = min(pos_total[i][:,2])
        if max_pos_z[1]!=None:
            max_pos_z[1] = max(max_pos_z[1], max(pos_total[i][:,2]))
        else:
            max_pos_z[1] = max(pos_total[i][:,2])

for i in pos_total:
    pos_total[i][:, 2] = pos_total[i][:,2]-max_pos_z[0]
if max_pos_z[0]!=None:
    max_pos_z[1]-=max_pos_z[0]
    max_pos_z[0] = 0




tr_plot.title.set_text('Delay')
tr_plot.set_xlabel("time, s")
tr_plot.set_ylabel("Delay, ms")
tr_plot.legend()
fig2.savefig('./results/delay.png')


tp_plot.title.set_text('Throughput')
tp_plot.set_xlabel("time, s")
tp_plot.set_ylabel("Throughput, Mbps")
tp_plot.legend()
fig.savefig('./results/throughput.png')


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

def dist_calc(pos1, pos2):
    d2d = haversine(pos1[0:2], pos2[0:2], unit='m')
    d3d = np.sqrt(abs(pos1[2] - pos2[2]) ** 2 +d2d**2)
    d3d = round(d3d,2)
    return d3d


def update_stats_tp(i):
    if len(fig4.texts)>0:
        fig4.texts.clear()
    textstr = "Time: {}".format(str(i))
    plt.gcf().text(0.02, 0.02, textstr, fontsize=14)
    for j in tp_total:
        tp_ip = data[j].item()['tp_ip']
        cur_time_tp = tp_total_time[j]
        cur_idx_tp = np.where(cur_time_tp < i)[0]

        if len(cur_idx_tp) > 0:
            tp_plot_ani.clear()
            tp_plot_ani.title.set_text('Throughput')
            tp_plot_ani.set_xlabel("time, s")
            tp_plot_ani.set_ylabel("Throughput, Mbps")
            cur_idx = cur_idx_tp[-1]
            tp_plot_ani.plot(tp_total_time[j][:cur_idx], tp_total[j][:cur_idx], color="r", marker='.', linewidth=0, markersize=1,
                         label=('TX - {}, RX - {}'.format(j, tp_ip)))
            tp_plot_ani.plot(tp_total_time[j][:cur_idx], tp_total_ma[j][:cur_idx], color="r",
                         label=('Mov. av., TX - {}, RX - {}'.format(j, tp_ip)))
            tp_plot_ani.legend()


def update_stats_tr(i):
    if len(fig5.texts)>0:
        fig5.texts.clear()
    textstr = "Time: {}".format(str(i))
    plt.gcf().text(0.02, 0.02, textstr, fontsize=14)

    for j in tr_total:
        tp_ip = data[j].item()['tp_ip']
        cur_time_tp = tr_total_time[j]
        cur_idx_tp = np.where(cur_time_tp < i)[0]

        if len(cur_idx_tp) > 0:
            tr_plot_ani.clear()
            tr_plot_ani.title.set_text('Delay, ms')
            tr_plot_ani.set_xlabel("time, s")
            tr_plot_ani.set_ylabel("Delay")
            cur_idx = cur_idx_tp[-1]
            tr_plot_ani.plot(tr_total_time[j][:cur_idx], tr_total[j][:cur_idx], color="r", marker='.', linewidth=0, markersize=1,
                         label=('TX - {}, RX - {}'.format(j, tp_ip)))
            tr_plot_ani.plot(tr_total_time[j][:cur_idx], tr_total_ma[j][:cur_idx], color="r",
                         label=('Mov. av., TX - {}, RX - {}'.format(j, tp_ip)))
            tr_plot_ani.legend()





def update_graph(i):
    fixed_edges = []
    path_edges  = {}
    pos = {}
    for j in pos_total:
        cur_pos_time = pos_time_total[j]
        cur_idx_pos = np.where(cur_pos_time < i)[0]
        if len(cur_idx_pos) > 0:
            cur_idx_pos = cur_idx_pos[-1]
            pos[j] = pos_total[j][cur_idx_pos]
    for j in rt_path_total:
        cur_time_path = rt_path_time[j]
        cur_idx_path  = np.where(cur_time_path<i)[0]
        if len(cur_idx_path)>0:
            #print(cur_idx_path)
            #print(rt_path_total[j][cur_idx_path])
            path_edges[j] = rt_path_total[j][cur_idx_path[-1]]
    for j in rt_total:
        cur_time = rt_time_total[j]
        cur_pos_time = pos_time_total[j]
        cur_idx_list = np.where(cur_time<i)[0]
        if len(cur_idx_list)>0:
            cur_idx = cur_idx_list[-1]
            m = rt_total[j][cur_idx][-1]
            if (j,m) not in fixed_edges and m!=j:
                fixed_edges.append([j,m])
    #print(i)
    #print(network_map)
    #print(fixed_positions)
    #print(fixed_nodes)


    rt_plot.clear()
    rt_plot.title.set_text('Network layout')
    rt_plot.set_xlabel("lat")
    rt_plot.set_ylabel("lon")
    rt_plot.set_zlabel("alt, m")
    rt_plot.set_xlim(max_pos_x)
    rt_plot.set_ylim(max_pos_y)
    rt_plot.set_zlim(max_pos_z)

    #rt_plot.legend()
    if len(fig3.texts)>0:
        fig3.texts.clear()

    textstr = "Time: {}".format(str(i))
    plt.gcf().text(0.02, 0.02, textstr, fontsize=14)
    if len(pos.keys()) > 0:
        #positions = nx.spring_layout(network_map, pos=fixed_positions, fixed=fixed_nodes)
        #nx.draw_networkx(network_map, positions, with_labels=True)
        for m in pos:
            rt_plot.scatter(pos[m][0],pos[m][1],pos[m][2], color = colors_fixed[m])
            rt_plot.text(pos[m][0],pos[m][1],pos[m][2], '%s' % (str(m)), size=10, zorder=1, color='k')
        # for j in fixed_edges:
        #     x = [pos[j[0]][0], pos[j[1]][0]]
        #     y = [pos[j[0]][1], pos[j[1]][1]]
        #     z = [pos[j[0]][2], pos[j[1]][2]]
        #     rt_plot.plot(x, y, z, c='black')
        #     a = Arrow3D(x, y,
        #                 z, mutation_scale=5,
        #                 lw=1, arrowstyle="-|>", color="k")
        #     rt_plot.add_artist(a)
        #     label = str(dist_calc(pos[j[0]], pos[j[1]]))+"m"
        #     rt_plot.text(np.mean(x), np.mean(y), np.mean(z), label,size=10, zorder=1, color='k')
        for k,j in path_edges.items():
            for m in range(0,len(j)-1):
                if j[m] in pos.keys() and j[m+1] in pos.keys():
                    x = [pos[j[m]][0], pos[j[m+1]][0]]
                    y = [pos[j[m]][1], pos[j[m+1]][1]]
                    z = [pos[j[m]][2], pos[j[m+1]][2]]
                    rt_plot.plot(x, y, z, c=colors_fixed[k])
                    a = Arrow3D(x, y,
                                z, mutation_scale=10,
                                lw=2, arrowstyle="-|>", color=colors_fixed[k])
                    rt_plot.add_artist(a)





#ani = FuncAnimation(fig3, update_graph, frames=range(0,ceil(max_time_rt)), interval=1000)

#ani1 = FuncAnimation(fig4, update_stats_tp, frames=range(0,ceil(max_time_rt)), interval=1000)

#ani2 = FuncAnimation(fig5, update_stats_tr, frames=range(0,ceil(max_time_rt)), interval=1000)

#ani2.save('./results/delay_routing.mp4')
#ani1.save('./results/throughput_routing.mp4')
#ani.save('./results/positions_routing.mp4')
plt.show()