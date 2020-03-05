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


save_archive = False
load_from_archive=True
load_local = False
archive_name_1 = 'results/RRarchive2020-03-05 15:36.npz'
archive_name_2 = 'results/RRarchive2020-03-05 16:08.npz'

home = expanduser("~")
local_results_folder = home+"/workspace_aero/src/infrastructure-on-demand/routing_protocol/src/results/"

data_1 = {}

f = np.load(archive_name_1, allow_pickle=True)
data_full_1 = f['arr_0']
for i in data_full_1.item():
    data_1[i] = data_full_1.item()[i]

data_2 = {}

f = np.load(archive_name_2, allow_pickle=True)
data_full_2 = f['arr_0']
for i in data_full_2.item():
    data_2[i] = data_full_2.item()[i]



print("Loading files complete, parsing...")
fig, tp_plot = plt.subplots()
fig2, tr_plot = plt.subplots()


colors = ['b', 'g', 'r', 'c', 'y']
colors_fixed = {}


tp_total = {}
tp_total_ma = {}
tp_total_time = {}

tr_total = {}
tr_total_ma = {}
tr_total_time = {}

rt_total = {}
rt_path_total = {}
rt_path_time = {}

j=0
for i in data_1:
    dat = data_1[i]
    #print(dat.item())
    print(dat.item()["delay"])
    start_time = dat.item()["start_time"]
    m_av_len = 20
    tr = dat.item()["delay"]
    if len(tr)>0:
        tp_ip = dat.item()['tp_ip']
        tr = np.transpose(np.array(tr, dtype=object))
        rt_path_total[i] = tr[1]
        print(tr)
        rt_path_time[i] = tr[0]-start_time
        y_ax = tr[1]
        tr_plot_ma = moving_average_inf(y_ax, tr[0]-start_time, n=m_av_len, period=10)
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

tp_total = {}
tp_total_ma = {}
tp_total_time = {}

tr_total = {}
tr_total_ma = {}
tr_total_time = {}

rt_total = {}
rt_path_total = {}
rt_path_time = {}

for i in data_2:
    dat = data_2[i]
    #print(dat.item())
    print(dat.item()["tr"])
    start_time = dat.item()["start_time"]
    m_av_len = 20
    tr = dat.item()["delay"]
    if len(tr)>0:
        tp_ip = dat.item()['tp_ip']
        tr = np.transpose(np.array(tr, dtype=object))
        rt_path_total[i] = tr[1]
        print(tr)
        rt_path_time[i] = tr[0]-start_time
        y_ax = tr[1]
        tr_plot_ma = moving_average_inf(y_ax, tr[0]-start_time, n=m_av_len, period=10)
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

tr_plot.title.set_text('Delay')
tr_plot.set_xlabel("time, s")
tr_plot.set_ylabel("Delay, ms")
tr_plot.legend()
fig2.savefig('./results/delay_comp.png')


tp_plot.title.set_text('Throughput')
tp_plot.set_xlabel("time, s")
tp_plot.set_ylabel("Throughput, Mbps")
tp_plot.legend()
fig.savefig('./results/throughput_comp.png')


plt.show()