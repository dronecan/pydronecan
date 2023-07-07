#!/usr/bin/env python3
'''
bridge between two CAN interfaces
this can be used to bridge a hardware device with SLCAN to a SITL vcan0 environment
'''

import dronecan
import time
import sys
import threading

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='CAN bridge - connect two CAN interfaces')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("port1", default=None, type=str, help="first interface")
parser.add_argument("port2", default=None, type=str, help="second interface")
args = parser.parse_args()

try:
    d1 = dronecan.driver.make_driver(args.port1, bitrate=args.bitrate)
except Exception as ex:
    print("Unable to connect to %s - %s" % (args.port1, ex))
    sys.exit(1)
print("Connected to %s" % args.port1)

try:
    d2 = dronecan.driver.make_driver(args.port2, bitrate=args.bitrate)
except Exception as ex:
    print("Unable to connect to %s - %s" % (args.port2, ex))
    sys.exit(1)
print("Connected to %s" % args.port2)

class BridgeThread(object):
    def __init__(self, d1, d2, name):
        self.d1 = d1
        self.d2 = d2
        self.count = 0
        self.thd = threading.Thread(target=self.loop, name=name)
        self.thd.start()

    def loop(self):
        while True:
            try:
                frame = self.d1.receive(timeout=0.1)
            except dronecan.driver.common.TransferError:
                continue
            if frame:
                self.count += 1
                try:
                    self.d2.send_frame(frame)
                except dronecan.driver.common.TxQueueFullError:
                    pass

t1 = BridgeThread(d1, d2, "d1->d2")
t2 = BridgeThread(d2, d1, "d2->d1")

last_print = time.time()

last_d1 = 0
last_d2 = 0

while True:
    time.sleep(1)
    now = time.time()
    dt = now - last_print
    c1 = t1.count
    c2 = t2.count
    print("%.3f/%.3f pkts/sec" % ((c1-last_d1)/dt, (c2-last_d2)/dt))
    last_print = now
    last_d1 = c1
    last_d2 = c2
