import json
import os
import subprocess
import tempfile
import time
import datetime
from os.path import expanduser

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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
archive_name = 'results/RRarchive2019-10-14 18:06.npz'
home = expanduser("~")
local_results_folder = home+"/workspace_aero/src/intel_aero/routing_protocol/src/results/"

data = {}

if load_from_archive:
    f = np.load(archive_name, allow_pickle=True)
    data_full = f['arr_0']
    for i in data_full.item():
        data[i] = data_full.item()[i]
else:
    file_name = 'results/RR'
    local = '10.42.0.13'

    hosts = ['10.42.0.7']
    passwd = ('1234567890\n').encode()
    files = {}
    for i in hosts:
        files[i] = file_name+"_"+str(i)+".npz"
        aero_id = i.split(".")[-1]
        server = "aero{}@{}".format(aero_id,i)
        path = "~/ws_intel/src/intel_aero/routing_protocol/src/{}".format(files[i])
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

colors = ['b', 'g', 'r', 'c', 'y']
j=0
for i in data:
    dat = data[i]
    print(dat.item())
    start_time = dat.item()["start_time"]
    m_av_len = 20
    tr = dat.item()["tr"]
    if len(tr)>0:
        tp_ip = dat.item()['tp_ip']
        tr = np.transpose(tr)
        y_ax = [sum(k) for k in tr[2]]
        tr_plot_ma = moving_average(y_ax, m_av_len)
        tr_plot.plot(tr[0]-start_time, y_ax, color=colors[j % len(colors)], marker='.', linewidth=0, markersize=1,
                 label=('TX - {}, RX - {}'.format(i, tp_ip)))
        tr_plot.plot(tr[0]-start_time, tr_plot_ma, color=colors[j % len(colors)],
             label=('Mov. av., TX - {}, RX - {}'.format(i, tp_ip)))

    tp = dat.item()["tp"]
    if len(tp)>0:
        tp_ip = dat.item()['tp_ip']
        tp = np.transpose(tp)
        tp_plot_ma = moving_average(tp[1], m_av_len)
        tp_plot.plot(tp[0], tp[1], color=colors[j % len(colors)], marker='.', linewidth=0, markersize=1,
             label=('TX - {}, RX - {}'.format(i, tp_ip)))
        tp_plot.plot(tp[0], tp_plot_ma, color=colors[j % len(colors)],
             label=('Mov. av., TX - {}, RX - {}'.format(i, tp_ip)))

    j+=1

tr_plot.title.set_text('Delay')
tr_plot.set_xlabel("time, s")
tr_plot.set_ylabel("Delay, ms")
tr_plot.legend()

tp_plot.title.set_text('Throughput')
tp_plot.set_xlabel("time, s")
tp_plot.set_ylabel("Throughput, Mbps")
tp_plot.legend()


plt.show()
