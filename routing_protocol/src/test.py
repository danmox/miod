#!/usr/bin/python
import subprocess
import re
import numpy as np
cmdline = ["fping", '10.42.0.13', '-c', '1', '-t', '1500']
cur_wireless = subprocess.run(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
cur_wireless_update = cur_wireless.stdout.decode()
print(cur_wireless_update)
delays = re.search(r"bytes,\s(.*?)\sms", cur_wireless_update)
if delays is not None:
    print(delays)
    print(float(delays.group(0)))