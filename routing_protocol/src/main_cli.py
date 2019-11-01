import argparse
import datetime
import json
import os
import random
import select
import subprocess
import sys
import threading
from time import sleep, time
import re

import getch
import netifaces as ni
import numpy

import socket
from socket import *
from os.path import expanduser

import pexpect
import pyric.pyw as pyw  # iw functionality
import numpy as np




#print(cur_wireless)

class Params:
    SERVER = '10.42.0.13'  # Standard loopback interface address (localhost)
    BROADCAST = '10.42.0.255'
    SUBNET='10.42.0.0/24'
    PORT = 54545  # Port to listen on (non-privileged ports are > 1023)
    PORT_PING = 54546  # Port to listen on for ping msgs
    WIFI_IF = 'wlp1s0'
    HOST=None
    MAC = None

    RESPONSIVENESS_TIMEOUT = 2
    inactivity_timer = 2000 #inactivity ms after which neighbor is considered dead
    statistics_collection = True
    period = 1  # status update frequency in s
    rt_update_period=0.5 #update frequency of routing table
    ping_period = 5 #arp update freq
    cs = None #initialize socket with None
    cs_ping = None
    sim_run=True
    mac_neigh=[] #list of MAC adr. of the neighbors
    ip_neigh=[] #list of neighbors IPs
    routing_table={}
    rt_tables_ids = []

    TP_IP = None
    tp_update_period = 1  # throughput query interval
    tp_stats = []
    traceroute_stats = []

    home = expanduser("~")
    results_folder = home+'/ws_intel/src/intel_aero/routing_protocol/src/results/'
    final_stats = {}
    final_stats["start_time"] = time()
    final_stats["ws"] = []
    final_stats["tp"] = []
    final_stats["tr"] = []
    final_stats["tp_ip"] = None


def unique(sequence):
    seen = set()
    return [x for x in sequence if not (x in seen or seen.add(x))]

### Threads definition

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
        cur_tp = pexpect.spawn(cmdline, timeout=None)
        start_time = Params.final_stats["start_time"]
        while Params.sim_run is True:
            #for line in iter(cur_tp.stdout.readline, b''):
                line = cur_tp.readline()
                line_readable = line.decode()
                end_of_meas = re.search(r"- - - - - - - - - - - - - - - - - - - - - - - - -", line_readable)
                if end_of_meas != None:
                    break
                time_stamp = re.search(r"\d{1,5}\.\d{2}", line_readable)
                throughput_mbps = re.search(r"(?<=Bytes).*(?=Mbits/sec)", line_readable)
                throughput_kbps = re.search(r"(?<=Bytes).*(?=Kbits/sec)", line_readable)
                throughput_bps = re.search(r"(?<=Bytes).*(?=bits/sec)", line_readable)

                tp = None
                if time_stamp != None:
                    #time_stamp = time_stamp.group(0)
                    time_stamp = time() - start_time

                    if throughput_mbps != None:
                        tp = float(throughput_mbps.group(0))
                        Params.tp_stats.append([float(time_stamp), tp])
                    elif throughput_kbps != None:
                        tp = 1e-3 * float(throughput_kbps.group(0))
                        Params.tp_stats.append([float(time_stamp), tp])
                    elif throughput_bps != None:
                        tp = 1e-6 * float(throughput_bps.group(0))
                        Params.tp_stats.append([float(time_stamp), tp])
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

def measurement_thread_traceroute():
    if Params.TP_IP!=None:
        cmdline = ["sudo", "traceroute", "-T", Params.TP_IP]
        while Params.sim_run is True:
            try:
                subprocess.check_output(cmdline)
                break
            except subprocess.CalledProcessError:
                print("retrying traceroute")
                sleep(1)
        print("traceroute to {} succesfull".format(str(Params.TP_IP)))
        start_time = Params.final_stats["start_time"]
        while Params.sim_run is True:
            cmdline = ["sudo", "traceroute", "-T", Params.TP_IP]
            cur_wireless = subprocess.run(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
            cur_wireless_update = cur_wireless.stdout.decode()
            ips = np.array(re.findall(r"((?<=\d\s\s)([\d]{1,3}\.){3}[\d]{1,3})", cur_wireless_update))
            delays = np.array(re.findall(r"((?<=\)\s\s)[\d]{1,4}\.[\d]{1,3})", cur_wireless_update))
            if len(ips)>0:
                ips2 = ips.transpose()[0]
                if len(ips2)>0:
                    ips2 = list(ips2)
                    ips2.insert(0,Params.HOST)
                    delays = list(np.around(delays.astype(float),1))
                    Params.traceroute_stats.append([time()-start_time,ips2,delays])
                    if Params.statistics_collection:
                        Params.final_stats["tr"].append([time(),ips2,delays])
            sleep(Params.rt_update_period)


def status_send_update_thread():
    seq_number=0
    while Params.sim_run is True:
        wireless_status = dict()
        # wireless_status structure is  {ip_addr1:signal(dbm),tx_rate,rx_rate (Mbps), ip_addr2...}
        cur_wireless = subprocess.run(['iw', 'dev', Params.WIFI_IF, 'station', 'dump'], stdout=subprocess.PIPE,stderr=None)
        cur_wireless_update = cur_wireless.stdout.decode()
        cur_wireless_update = re.sub("[^a-zA-Z'\d:\n -.]+", '', cur_wireless_update)
        signal_level = re.findall(r"(?<=signal:\s\s)[-+]?\d+", cur_wireless_update)
        station_mac = re.findall(r"((?<=Station\s)([a-f\d]{1,2}:){5}[a-f\d]{1,2})", cur_wireless_update)
        station_mac = np.transpose(station_mac)[0,:]
        tx_bitrate = re.findall(r"(?<=tx\sbitrate:)[-+]?\d+.\d+", cur_wireless_update)
        rx_bitrate = re.findall(r"(?<=rx\sbitrate:)[-+]?\d+.\d+", cur_wireless_update)
        inactive_time = re.findall(r"(?<=inactive\stime:)\d+", cur_wireless_update)
        del_idx=[]
        for i in range(0,len(inactive_time)):
            if int(inactive_time[i])>Params.inactivity_timer:
                del_idx.append(i)
        signal_level = list(np.delete(signal_level,del_idx))
        station_mac = list(np.delete(station_mac,del_idx))
        tx_bitrate = list(np.delete(tx_bitrate,del_idx))
        rx_bitrate = list(np.delete(rx_bitrate,del_idx))

        mac_neigh = Params.mac_neigh
        ip_neigh = Params.ip_neigh
        #print(mac_neigh)
        #print(station_mac)
        status={}
        if mac_neigh and len(mac_neigh)==len(ip_neigh)==len(station_mac):
            ip_sorted = [ip_neigh[station_mac.index(i)] for i in mac_neigh if i in station_mac]
            ip_sorted = unique(ip_sorted)
            for i in range(0, len(ip_sorted)):
                wireless_status[ip_sorted[i]] = [float(signal_level[i]), float(tx_bitrate[i]), float(rx_bitrate[i])]
            status["ws"] = wireless_status
            Params.final_stats["ws"].append([time(), wireless_status])

            #print(wireless_status)

        throughput_status = Params.tp_stats
        traceroute_status = Params.traceroute_stats
        if Params.TP_IP != None and len(Params.tp_stats)>0:
            status["tp"] = [throughput_status, Params.TP_IP]
            Params.tp_stats = []
        if len(Params.traceroute_stats)>0:
            status["tr"] = [traceroute_status, Params.TP_IP]
            Params.traceroute_stats = []
        if len(wireless_status.keys())>0 or len(throughput_status)>0 or len(traceroute_status)>0:
            msg = [Params.HOST, seq_number, status]
            msg = json.dumps(msg)
            Params.cs.sendto(msg.encode(), (Params.BROADCAST, Params.PORT))
            seq_number+=1
        sleep(Params.period)


def rt_update_thread():
    while Params.sim_run is True:
        new_rt = Params.routing_table
        for src in new_rt.keys():
            if not src in Params.rt_tables_ids:
                Params.rt_tables_ids.append(src)
                #print(str(Params.rt_tables_ids.index(src)))
                subprocess.run(
                    ["sudo", "ip", "rule", "add", "from", str(src), "table", str(Params.rt_tables_ids.index(src) + 1),
                     "prio", "2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            dest = new_rt[src][0][0]
            subprocess.run(["sudo", "ip", "route", "del", dest, "table", str(Params.rt_tables_ids.index(src) + 1)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if len(new_rt[src][0]) > 2:
                cur_prob = 0
                coin = random.uniform(0, 1)
                for i in range(0, len(new_rt[src][1])):
                    cur_prob += new_rt[src][1][i]
                    if coin <= cur_prob:
                        subprocess.run(
                            ["sudo", "ip", "route", "add", dest, "via", new_rt[src][0][i + 1], "dev", Params.WIFI_IF,
                             "table", str(Params.rt_tables_ids.index(src) + 1)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        print(
                            "changing route: from {} to {} is now via {}".format(str(src), dest, new_rt[src][0][i + 1]))


            else:
                subprocess.run(
                    ["sudo", "ip", "route", "add", dest, "via", new_rt[src][0][1], "dev", Params.WIFI_IF, "table",
                     str(Params.rt_tables_ids.index(src) + 1)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print("changing route: from {} to {} is now via {}".format(str(src), dest, new_rt[src][0][1]))

        sleep(Params.rt_update_period)

def parse_server_msg(msg):
    if Params.HOST in msg.keys():
        Params.routing_table = msg[Params.HOST]

def status_receive_update_thread():
    seq_num={}
    while Params.sim_run is True:
        ready = select.select([Params.cs], [], [], 2)
        if ready[0]:
            data_init, cur_src = Params.cs.recvfrom(4096)
        else:
            continue
        #print(source)
        #all messages should have a shape of [sender, seq_num, data]
        data = data_init.decode()
        data = json.loads(data)
        source  = data[0]
        seq_num_upd = data[1]
        msg = data[2]
        #print(["routing table", Params.routing_table])
        if source!=Params.HOST:
            #print(["received msg from:", cur_src])
            if source in seq_num.keys():
                    #check for sequence number - we do not want to retransmit the msg too many times
                if seq_num_upd>seq_num[source]:
                    print(["retransmitting msg from:", cur_src])
                    print(["seq number:", seq_num_upd])
                    seq_num[source] = seq_num_upd
                    if source == Params.SERVER:
                        parse_server_msg(msg)
                    Params.cs.sendto(data_init, (Params.BROADCAST, Params.PORT))
            else:
                seq_num[source]=seq_num_upd
                #print(["retransmitting first msg from:", cur_src])
                #print(["seq number:", seq_num_upd])
                if source == Params.SERVER:
                    parse_server_msg(msg)
                Params.cs.sendto(data_init, (Params.BROADCAST, Params.PORT))


def send_ping_thread():
    while Params.sim_run is True:
        msg = Params.HOST
        msg = json.dumps(msg)
        Params.mac_neigh = []
        Params.ip_neigh = []
        Params.cs_ping.sendto(msg.encode(), (Params.BROADCAST, Params.PORT_PING))
        sleep(Params.ping_period)

def receive_ping_thread():
    while Params.sim_run is True:
        ready = select.select([Params.cs_ping], [], [], 2)
        if ready[0]:
            data_init, cur_src = Params.cs_ping.recvfrom(4096)
        else:
            continue
        data = data_init.decode()
        data = json.loads(data)
        if cur_src[0]!=Params.HOST:
            #print(["received ping msg from:", cur_src])
            if data==cur_src[0]:
                msg = [Params.HOST, Params.MAC]
                msg = json.dumps(msg)
                #print(["sending ping ack to:", cur_src])
                Params.cs_ping.sendto(msg.encode(), (Params.BROADCAST, Params.PORT_PING))
            else:
                if isinstance(data,list):
                    if data[0] not in Params.ip_neigh:
                        Params.mac_neigh.append(data[1])
                        Params.ip_neigh.append(data[0])




def remove_network():
    for i in range(len(Params.rt_tables_ids)):
        subprocess.call(["sudo", "ip", "rule", "del", "table", str(i + 1)])
        subprocess.call(["sudo", "ip", "route", "flush", "table", str(i + 1)])


###Main
def main():
    parser = argparse.ArgumentParser(description='Cli part of the RR message distribution protocol')
    parser.add_argument("--dest_ip", help="Destination ip for the throughput test",
                        type=str, default=None)
    parser.add_argument("--srv", help="Server ip",
                        type=str, default=None)
    parser.add_argument("--interface", help="Wireless interface name",
                        type=str, default=None)

    args, unknown = parser.parse_known_args()

    Params.TP_IP=args.dest_ip
    Params.final_stats["tp_ip"] = args.dest_ip
    if args.srv != None:
        Params.SERVER = args.srv
    if args.interface != None:
        Params.WIFI_IF = args.interface

    cs = socket(AF_INET, SOCK_DGRAM)
    Params.HOST = ni.ifaddresses(Params.WIFI_IF)[ni.AF_INET][0]['addr']
    w0 = pyw.getcard(Params.WIFI_IF)
    Params.MAC = pyw.macget(w0)
    print(['Node IP:', Params.HOST])
    cs.bind((Params.BROADCAST, Params.PORT))
    cs.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    cs.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    Params.cs = cs

    cs_ping = socket(AF_INET, SOCK_DGRAM)
    cs_ping.bind((Params.BROADCAST, Params.PORT_PING))
    cs_ping.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    cs_ping.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    Params.cs_ping = cs_ping

    threads = [
               threading.Thread(target=status_receive_update_thread),
               threading.Thread(target=rt_update_thread),
               threading.Thread(target=send_ping_thread),
    ]

    if Params.statistics_collection is True:
        threads.append(threading.Thread(target=status_send_update_thread))
        threads.append(threading.Thread(target=measurement_thread_throughput_cli))
        threads.append(threading.Thread(target=measurement_thread_throughput_srv))
        threads.append(threading.Thread(target=measurement_thread_traceroute))
        threads.append(threading.Thread(target=send_ping_thread))
        threads.append(threading.Thread(target=receive_ping_thread))

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
        Params.cs.close()
        print("Threads successfully closed")
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
                results_file_new = './results/'+ "RR_" + str(Params.HOST)

            np.savez(results_file_new, Params.final_stats)
            print("results saved in file {}".format(results_file_new))

main()