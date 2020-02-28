#!/usr/bin/python
import fcntl
import json
import os
import re
import select
import subprocess
import sys
from time import sleep


#cmdline = ['python2', '/home/mishanya/workspace_aero/test_routing/src/rt_update_sub.py']
#cmdline = ['roslaunch', 'test_routing', 'launch_sub.launch']
import pexpect

cmdline = ["iperf3", "-c", "10.42.0.7", "-i", "2", "-t", "1"]

while True:
     try:
         subprocess.check_output(cmdline)
         break
     except subprocess.CalledProcessError:
         print("retrying connectetion")
         sleep(1)
print("connection to {} succesfull".format("10.42.0.7"))
#cur_wireless = subprocess.Popen(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#p1 = pexpect.spawn(cmdline,timeout=None,logfile=sys.stdout.buffer)
cmdline = "iperf3 -c 10.42.0.7 -i 2 -t 400"

p = pexpect.spawn(cmdline,timeout=None, logfile=sys.stdout.buffer)
#print(p.expect(b'Connecting to host 10.42.0.7, port 5201'))
#while True:
    #print(p.eof())
    #line = p.readline()
    #a = True
#    print(p.expect(b'\r\n'))
    #a = p.expect(b'iperf3: error - unable to connect to server: Connection refused\r\n')


#    sleep(0.1)


#y=select.poll()
#y.register(cur_wireless.stdout,select.POLLIN)
line = p.readline()
try:
    while line:
        #line = p.readline()
        #print(line)
        #line_readable = line.decode()
        print(line)
        line=p.readline()
    #a = cur_wireless.stdout
    #print(a)
    #while True:
        #if cur_wireless.poll():
        #    print (cur_wireless.stdout.readline())
        #else:
        #    print("nothing here")
        #sleep(0.1)
    # for line in iter(cur_wireless.stdout.readline, b''):
    #     line_readable = line.decode()
    #     line_readable = str(line_readable).strip("'<>() ").replace('\'', '\"')
    #     data = json.loads(line_readable)
    #     print(data)
        #cur_wireless_update = json.loads(cur_wireless.stdout.decode('utf-8'))
        #signal_level = re.findall(r"(?<=signal:\s\s)[-+]?\d+", cur_wireless_update)
        sleep(1)
except Exception as e:
    print(e)
    print("ololo")
finally:
    p.terminate()
p.terminate()

    #cur_wireless.terminate()
#cur_wireless.terminate()