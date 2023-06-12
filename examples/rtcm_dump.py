#!/usr/bin/env python3
'''
decode RTCMStream data from a DroneCAN URI
'''

import dronecan, time
from dronecan import uavcan
from MAVProxy.modules.lib import rtcm3

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='Fix2 gap example')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("port", default=None, type=str, help="serial port")
args = parser.parse_args()

# Initializing a DroneCAN node instance.
node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

# Initializing a node monitor
node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

rtcm_file = open("rtcm.dat", "wb")
decoder = rtcm3.RTCM3()

def handle_RTCMStream(msg):
    data = msg.message.data.to_bytes()
    for b in data:
        if decoder.read(chr(b)):
            print("packet len %u ID %u" % (len(decoder.get_packet()), decoder.get_packet_ID()))
    rtcm_file.write(data)
    rtcm_file.flush()

# callback for printing ESC status message to stdout in human-readable YAML format.
node.add_handler(dronecan.uavcan.equipment.gnss.RTCMStream, handle_RTCMStream)

while True:
    try:
        node.spin()
    except Exception as ex:
        pass
