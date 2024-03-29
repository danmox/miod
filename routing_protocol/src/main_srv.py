import json
import random
import re
import select
import subprocess
import sys
import threading
from copy import deepcopy
from time import sleep, time
import os
from os.path import expanduser

import pexpect
from matplotlib import pyplot as plt
import matplotlib
import networkx as nx


import netifaces as ni
import numpy as np

import socket
from socket import *
import pyric.pyw as pyw  # iw functionality
import argparse
##defining vairables


class Params:
    SERVER = None  # Standard loopback interface address (localhost)
    BROADCAST = '10.42.0.255'
    SUBNET='10.42.0.0/24'
    RESPONSIVENESS_TIMEOUT = 1
    HOST = None
    MAC = None
    PORT = 54545  # Port to listen on (non-privileged ports are > 1023)
    PORT_PING = 54546
    WIFI_IF = "wlp1s0" #'wlx9cefd5fc63ef' 'wlp1s0'
    period = 1  # routing table update frequency in s
    rt_update_period = 0.5 # probabilistic routing table update period, s
    cs=None #initialize socket with None
    cs_ping=None
    sim_run=True

    routing_table = {}
    rt_tables_ids = []
    file_name_rt = './routing_tables_upd.txt'

    update_source = "ROS"
    snr_stats = {}
    tp_stats = {}
    traceroute_stats = {}
    routing = True
    dynamic_statistics_parsing = False
    statistics_collection = True
    dynamic_statistics_upd = 2 #dynamic statistics update timer
    tp_update_period = 1 # throughput query interval
    TP_IP = None

    home = expanduser("~")
    results_folder = home + '/ws_intel/src/intel_aero/routing_protocol/src/results/'
    final_stats = {}
    final_stats["start_time"] = time()
    final_stats["delay"] = []
    final_stats["ws"] = []
    final_stats["tp"] = []
    final_stats["tr"] = []
    final_stats["rt"] = []
    final_stats["pos"] = []
    final_stats["tp_ip"] = None


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

def global_rt_update_thread():
    #cmdline = ["roslaunch" , 'test_routing', 'launch_sub.launch']
    cmdline = "roslaunch routing_protocol launch_sub.launch"
    cur_wireless = pexpect.spawn(cmdline, timeout=None)#, logfile=sys.stdout.buffer)
    #cur_wireless = subprocess.Popen(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
    while Params.sim_run is True:
        if Params.update_source == "ROS":
            #for line in iter(cur_wireless.stdout.readline, b''):
                ready = select.select([cur_wireless], [], [], 2)
                if ready[0]:
                    line = cur_wireless.readline()
                else:
                    continue
                if line:
                    line_readable = line.decode()
                    line_readable = str(line_readable).strip("'<>() ").replace('\'', '\"')
                    #print(line_readable)
                    try:
                        json.loads(line_readable)
                    except json.decoder.JSONDecodeError:
                        #print("skipping line ---------------------------------------------------------")
                        continue
                    data = json.loads(line_readable)
                    update = {}
                    for entry in data:
                        ips = [i for i in entry if type(i)==str]
                        probs = [i for i in entry if type(i)!=str]
                        if str(ips[0]) in update.keys():
                            update[str(ips[0])][str(ips[1])]=[ips[2:], probs]
                        else:
                            update[str(ips[0])]={str(ips[1]):[ips[2:],probs]}
                    Params.routing_table = update
                #sleep(0.5)
        else:
            with open(Params.file_name_rt, 'rt') as myfile:
                update = {}
                for myline in myfile:
                    line = myline.rstrip('\n')
                    ips = np.array(re.findall(r"(([\d]{1,3}\.){3}[\d]{1,3})", line))
                    ips = list(ips.transpose()[0])
                    probs = re.findall(r"([\d],[\d]+)", line)
                    if str(ips[0]) in update.keys():
                        update[str(ips[0])][str(ips[1])] = [ips[2:], probs]
                    else:
                        update[str(ips[0])] = {str(ips[1]): [ips[2:], probs]}
                # print(update)
                Params.routing_table = update
    cur_wireless.terminate()

def pos_update_thread():
    #cmdline = ["roslaunch" , 'test_routing', 'launch_sub.launch']
    cmdline = "roslaunch routing_protocol launch_pos_sub.launch"
    cur_pos = pexpect.spawn(cmdline, timeout=None)#, logfile=sys.stdout.buffer)
    #cur_wireless = subprocess.Popen(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
    while Params.sim_run is True:
        ready = select.select([cur_pos], [], [], 2)
        if ready[0]:
            line = cur_pos.readline()
        else:
            continue
        if line:
            line_readable = line.decode()
            line_readable = str(line_readable).strip("'<>() ").replace('\'', '\"')
            try:
                json.loads(line_readable)
            except json.decoder.JSONDecodeError:
                #print("skipping line ---------------------------------------------------------")
                continue
            data = json.loads(line_readable) #lat lon alt
            time_stamp = time()
            data.insert(0,time_stamp)
            Params.final_stats["pos"].append(data)
    cur_pos.terminate()


def local_rt_update_thread():
    while Params.sim_run is True:
        if Params.HOST in Params.routing_table.keys():
            new_rt = Params.routing_table[Params.HOST]
            for src in new_rt.keys():
                dest = new_rt[src][0][0]
                if dest != Params.HOST:
                    if not src in Params.rt_tables_ids:
                        Params.rt_tables_ids.append(src)
                        subprocess.run(["sudo", "ip", "rule", "add", "from", str(src), "table", str(Params.rt_tables_ids.index(src)+1), "prio", "2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                    subprocess.run(["sudo", "ip", "route", "del", dest, "table", str(Params.rt_tables_ids.index(src)+1)],stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    if len(new_rt[src][0])>2:
                        cur_prob=0
                        coin=random.uniform(0, sum(new_rt[src][1]))
                        for i in range(0, len(new_rt[src][1])):
                            cur_prob+=new_rt[src][1][i]
                            if coin<=cur_prob:
                                if Params.statistics_collection:
                                    Params.final_stats["rt"].append([time(), src, dest, new_rt[src][0][i + 1]])
                                subprocess.run(
                                    ["sudo", "ip", "route", "add", dest, "via", new_rt[src][0][i+1], "dev", Params.WIFI_IF, "table",str(Params.rt_tables_ids.index(src)+1)],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                                print("changing route: from {} to {} is now via {}".format(str(src),dest,new_rt[src][0][i+1]))

                    elif len(new_rt[src][0])==2:
                        if Params.statistics_collection:
                            Params.final_stats["rt"].append([time(), src, dest, new_rt[src][0][1]])
                        subprocess.run(["sudo", "ip", "route", "add", dest,"via", new_rt[src][0][1], "dev", Params.WIFI_IF, "table", str(Params.rt_tables_ids.index(src)+1)] ,stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        print("changing route: from {} to {} is now via {}".format(str(src), dest, new_rt[src][0][1]))

        sleep(Params.rt_update_period)

def status_send_update_thread():
    seq_number=0
    while Params.sim_run is True:
        #print(Params.routing_table)
        if Params.routing_table:
            update = [Params.HOST,seq_number, Params.routing_table]
            msg = json.dumps(update)
            Params.cs.sendto(msg.encode(), (Params.BROADCAST, Params.PORT))
            seq_number += 1
        sleep(Params.period)

def receive_ping_thread():
    while Params.sim_run is True:
        ready = select.select([Params.cs_ping], [], [], 2)
        if ready[0]:
            data_init, cur_src = Params.cs_ping.recvfrom(4096)
        else:
            continue
        data = data_init.decode()
        data = json.loads(data)
        #print(["received ping msg from:", cur_src])
        if cur_src[0]!=Params.HOST:
            if data==cur_src[0]:
                msg = [Params.HOST, Params.MAC]
                msg = json.dumps(msg)
                Params.cs_ping.sendto(msg.encode(), (Params.BROADCAST, Params.PORT_PING))


def measurement_thread_throughput_cli():
    if Params.TP_IP!=None:
        cmdline = ["iperf3", "-c", Params.TP_IP, "-i", str(Params.tp_update_period), "-t", "1"]
        while Params.sim_run is True:
            try:
                subprocess.check_output(cmdline)
                break
            except subprocess.CalledProcessError:
                print("retrying connectetion")
                sleep(1)
        print("connection to {} succesfull".format(str(Params.TP_IP)))
        #cmdline = ["iperf3", "-c", Params.TP_IP, "-i", str(Params.tp_update_period), "-t", "60000"]
        cmdline = "iperf3 -c {}  -i {}  -t 60000".format(Params.TP_IP, str(Params.tp_update_period))

        #cur_tp = subprocess.Popen(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
        cur_tp = pexpect.spawn(cmdline,timeout=None)
        Params.tp_stats[Params.HOST, Params.TP_IP] = []
        start_time = Params.final_stats["start_time"]
        while Params.sim_run is True:
                line = cur_tp.readline()
                line_readable = line.decode()
                end_of_meas = re.search(r"- - - - - - - - - - - - - - - - - - - - - - - - -", line_readable)
                if end_of_meas != None:
                    break
                time_stamp = re.search(r"\d{1,5}\.\d{2}", line_readable)
                throughput_mbps = re.search(r"(?<=Bytes).*(?=Mbits/sec)", line_readable)
                throughput_kbps = re.search(r"(?<=Bytes).*(?=Kbits/sec)", line_readable)
                throughput_bps = re.search(r"(?<=Bytes).*(?=bits/sec)", line_readable)
                if time_stamp != None:
                    tp = None
                    #time_stamp= time_stamp.group(0)
                    time_stamp =  time()-start_time
                    if throughput_mbps != None:
                        tp = float(throughput_mbps.group(0))
                        Params.tp_stats[Params.HOST, Params.TP_IP].append([float(time_stamp), tp])
                    elif throughput_kbps != None:
                        tp = 1e-3 * float(throughput_kbps.group(0))
                        Params.tp_stats[Params.HOST, Params.TP_IP].append([float(time_stamp), tp])
                    elif throughput_bps != None:
                        tp =  1e-6 * float(throughput_bps.group(0))
                        Params.tp_stats[Params.HOST, Params.TP_IP].append([float(time_stamp), tp])

                    if Params.statistics_collection:
                        if tp != None:
                            Params.final_stats["tp"].append([float(time_stamp), tp])
                #sleep(0.5)

def measurement_thread_throughput_srv():
    cmdline = ["iperf3", "-s"]
    cur_wireless = subprocess.Popen(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
    while Params.sim_run is True:
        sleep(0.1)
    cur_wireless.terminate()

def measurement_thread_delay():
    if Params.TP_IP!=None:
        while Params.sim_run is True:
            cmdline = ["fping", Params.TP_IP, "-c1", "-t", str(Params.tp_update_period * 1000)]
            while Params.sim_run is True:
                try:
                    subprocess.check_output(cmdline)
                    break
                except subprocess.CalledProcessError:
                    print("retrying fping")
                    sleep(1)
            cur_wireless = subprocess.run(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
            cur_wireless_update = cur_wireless.stdout.decode()
            delays = re.search(r"bytes,\s(.*?)\sms", cur_wireless_update)
            if delays is not None:
                delays = float(delays.group(1))
                print(delays)
                if Params.statistics_collection:
                    Params.final_stats["delay"].append([time(), delays])
            sleep(Params.tp_update_period)



def measurement_thread_traceroute():
    if Params.TP_IP!=None:
        cmdline = ["sudo", "traceroute", Params.TP_IP]
        while Params.sim_run is True:
            try:
                subprocess.check_output(cmdline)
                break
            except subprocess.CalledProcessError:
                print("retrying traceroute")
                sleep(1)
        print("traceroute to {} succesfull".format(str(Params.TP_IP)))
        start_time = Params.final_stats["start_time"]
        Params.traceroute_stats[Params.HOST, Params.TP_IP] = []
        while Params.sim_run is True:
            cmdline = ["sudo", "traceroute", Params.TP_IP]
            cur_wireless = subprocess.run(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
            cur_wireless_update = cur_wireless.stdout.decode()
            ips = np.array(re.findall(r"((?<=\d\s\s)([\d]{1,3}\.){3}[\d]{1,3})", cur_wireless_update))
            if len(ips) > 0:
                ips2 = ips.transpose()[0]
                if len(ips2) > 0:
                    ips2 = list(ips2)
                    ips2.insert(0, Params.HOST)
                    Params.traceroute_stats[Params.HOST, Params.TP_IP].append([time() - start_time, ips2])
                    if Params.statistics_collection:
                        Params.final_stats["tr"].append([time(), ips2])
            sleep(Params.rt_update_period)



def dynamic_statistics_parsing_thread():
    fig= plt.figure(figsize=(21,7))
    ax1 = fig.add_subplot(131)
    ax2 = fig.add_subplot(132)
    ax3 = fig.add_subplot(133)
    colors = ['b', 'g', 'r', 'c', 'y']
    plt.ion()
    fig.canvas.draw()
    while Params.sim_run is True:
        #fig_snr.canvas.flush_events()
        #plt.clf()
        data_traceroute = deepcopy(Params.traceroute_stats)
        data_tp = deepcopy(Params.tp_stats)
        if len(data_tp)>0:
            ax1.clear()
            ax1.title.set_text('Throughput')
            max_tick = 0
            m_av_len = 20
            j = 0
            for i in data_tp.keys():
                if len(data_tp[i])>0:
                    tp_plot = np.transpose(data_tp[i])
                    tp_plot_ma = moving_average(tp_plot[1],m_av_len)
                    ax1.plot(tp_plot[0],tp_plot[1], color=colors[j%len(colors)], marker='.',linewidth=0, markersize=1, label=('TX - {}, RX - {}'.format(i[0], i[1])))
                    ax1.plot(tp_plot[0],tp_plot_ma, color=colors[j%len(colors)], label=('Mov. av., TX - {}, RX - {}'.format(i[0], i[1])))

                    if len(tp_plot[0])>1:
                        if max(tp_plot[0])>max_tick:
                            max_tick = max(tp_plot[0])
                            ax1.set_xticks(np.arange(0, max_tick, step=max_tick/10))
                j+=1
            ax1.legend()
            ax1.set_xlabel("time, s")
            ax1.set_ylabel("Throughput, Mbps")


        network_map = nx.MultiGraph()
        fixed_positions = {}
        edge_labels = {}
        fixed_nodes = []
        cur_time = time()
        pos = [0,0]
        plt.tight_layout()
        snr_stats = deepcopy(Params.snr_stats)
        for i in snr_stats.keys():
            time_stamp = snr_stats[i][1]
            if cur_time-time_stamp<Params.dynamic_statistics_upd:
                if i not in network_map.nodes:
                    network_map.add_node(i)
                    fixed_positions[i] = [pos[0],pos[1]]
                    fixed_nodes.append(i)
                    pos[0]+=1
                for j in snr_stats[i][0]:
                    if j not in network_map.nodes:
                        network_map.add_node(j)
                        fixed_positions[j] = [pos[0],pos[1]]
                        fixed_nodes.append(j)
                        pos[1] += 2
                    if (i,j) not in network_map.edges:
                        network_map.add_edge(i,j)
                        #network_map.add_edges_from([(i,j, dict(route=282))])
                        edge_labels[(i,j)]=str(snr_stats[i][0][j][0])+" dbm"

        if len(network_map.nodes)>0:

            ax3.clear()
            ax3.title.set_text('Network_layout, TX - {}, RX - {}'.format(Params.HOST, Params.TP_IP))
            plt.xlabel("Pos x")
            plt.ylabel("Pos y")
            plt.xticks([-5, 5])
            plt.xticks([-5, 5])
            positions = nx.spring_layout(network_map, pos=fixed_positions, fixed=fixed_nodes)
            nx.draw_networkx(network_map, positions, with_labels=True)
        fig.canvas.draw()
        sleep(0.1)
    plt.savefig("./results/test_run.png")
    plt.close()




def parse_wireless_status(msg,source):
    time_stamp = time()
    if Params.dynamic_statistics_parsing is True:
        Params.snr_stats[source] = (msg,time_stamp)

def parse_throughput_status(msg,src):
    tp = msg[0]
    #print(tp)
    dst = msg[1]
    if (src,dst) in Params.tp_stats.keys():
        Params.tp_stats[src, dst].extend(tp)
    else:
        Params.tp_stats[src, dst] = tp

def parse_traceroute(msg,src):
    tr = msg[0]
    # print(tp)
    dst = msg[1]
    if (src,dst) in Params.traceroute_stats.keys():
        Params.traceroute_stats[src, dst].extend(tr)
    else:
        Params.traceroute_stats[src, dst]=tr

def status_receive_update_thread():
    seq_num={}
    while Params.sim_run is True:
        ready = select.select([Params.cs], [], [], 2)
        if ready[0]:
            data_init, cur_src = Params.cs.recvfrom(4096)
        else:
            continue
        data = data_init.decode()
        data = json.loads(data)
        source = data[0]
        seq_num_upd = data[1]
        status = data[2]
        if source!=Params.HOST:
            if source in seq_num.keys():
                if seq_num_upd>seq_num[source]:
                    seq_num[source] = seq_num_upd
                    if "ws" in status:
                        parse_wireless_status(status["ws"],source)
                    if "tp" in status:
                        parse_throughput_status(status["tp"], source)
                    if "tr" in status:
                        parse_traceroute(status["tr"], source)

            else:
                seq_num[source]=seq_num_upd
                if "ws" in status:
                    parse_wireless_status(status["ws"], source)
                if "tp" in status:
                    parse_throughput_status(status["tp"], source)
                if "tr" in status:
                    parse_traceroute(status["tr"], source)

def remove_network():
    for i in range(len(Params.rt_tables_ids)):
        subprocess.call(["sudo", "ip", "rule", "del", "table", str(i + 1)])
        subprocess.call(["sudo", "ip", "route", "flush", "table", str(i + 1)])

def main():
    parser = argparse.ArgumentParser(description='Server part of the RR message distribution protocol')
    parser.add_argument("--dest_ip", help="Destination ip for the throughput test",
                        type=str, default=None)
    parser.add_argument("--interface", help="Wireless interface name",
                        type=str, default=None)

    args, unknown = parser.parse_known_args()
    if args.dest_ip!="None":
        Params.TP_IP=args.dest_ip
        Params.final_stats["tp_ip"] = args.dest_ip

    if args.interface != None:
        Params.WIFI_IF = args.interface

    cs = socket(AF_INET, SOCK_DGRAM)
    Params.HOST = ni.ifaddresses(Params.WIFI_IF)[ni.AF_INET][0]['addr']
    Params.SERVER = Params.HOST
    w0 = pyw.getcard(Params.WIFI_IF)
    Params.MAC = pyw.macget(w0)


    print(['Srv IP:', Params.HOST])
    print([Params.BROADCAST, Params.PORT])
    cs.bind((Params.BROADCAST, Params.PORT))
    cs.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    cs.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    Params.cs = cs

    cs_ping = socket(AF_INET, SOCK_DGRAM)
    cs_ping.bind((Params.BROADCAST, Params.PORT_PING))
    cs_ping.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    cs_ping.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    Params.cs_ping = cs_ping

    threads = [threading.Thread(target=status_send_update_thread),
               threading.Thread(target=local_rt_update_thread),
               threading.Thread(target=status_receive_update_thread),
    ]
    if Params.statistics_collection is True:
        threads.append(threading.Thread(target=measurement_thread_throughput_srv))
        threads.append(threading.Thread(target=measurement_thread_throughput_cli))
        threads.append(threading.Thread(target=receive_ping_thread))
        threads.append(threading.Thread(target=measurement_thread_traceroute))
        threads.append(threading.Thread(target=pos_update_thread))
        threads.append(threading.Thread(target=measurement_thread_delay))

    if Params.routing is True:
        threads.append(threading.Thread(target=global_rt_update_thread))

    if Params.dynamic_statistics_parsing is True:
        threads.append(threading.Thread(target=dynamic_statistics_parsing_thread))

    try:
        for t in threads:
            t.start()
        while Params.sim_run is True:
            r, __, __ = select.select([sys.stdin, ], [], [], Params.RESPONSIVENESS_TIMEOUT)
            if r:
                raise KeyboardInterrupt
            else:
                continue
        raise KeyboardInterrupt
    except KeyboardInterrupt:
        Params.sim_run = False
        for t in threads:
            t.join()
        print("Threads successfully closed")
        Params.cs.close()
    finally:
        remove_network()
        if Params.statistics_collection:
            if os.path.exists(Params.results_folder) is True:
                print("saving results to {}".format(Params.results_folder))
                results_file_new = Params.results_folder + "RR_" + str(Params.HOST)
            else:
                print("Folder {} does not exist. Trying to save results locally".format(Params.results_folder))
                dir = "./results/"
                if os.path.exists(dir) is not True:
                    os.mkdir(dir)
                results_file_new = './results/' + "RR_" + str(Params.HOST)

            np.savez(results_file_new, Params.final_stats)
            print("results saved in file {}".format(results_file_new))



main()
