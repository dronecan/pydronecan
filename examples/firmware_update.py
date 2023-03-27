#!/bin/python3

import dronecan, time, math
from argparse import ArgumentParser

import sys
import os

# import logging
# logging.basicConfig(level=logging.DEBUG)

parser = ArgumentParser(description='dump all DroneCAN messages')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("--dna-server", action='store_true', default=True, help="run DNA server")
parser.add_argument("--port", default='/dev/ttyACM0', type=str, help="serial port")
parser.add_argument("--app-firmware", default=None, type=str, help="serial port")

args = parser.parse_args()

firmware_path = args.app_firmware

if not os.path.exists(firmware_path):
    print('Please provide a valid firmware file path\n\t--app-firmware <path>')
    print(firmware_path)
    sys.exit(1)

# We need to symlink the firmware file path because of 40 character limit
file_path = '/tmp/fw.uavcan.bin'
if os.path.islink(file_path):
    os.unlink(file_path)
os.symlink(firmware_path, file_path)

# Ensure it is readable
try:
    with open(file_path, 'rb') as f:
        f.read(100)
except Exception as ex:
    print('Could not read file')
    sys.exit(1)

# Create the CAN Node
global node
print('Starting server as Node ID', args.node_id)
node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

if args.dna_server:
    dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(node, node_monitor)


# Waiting for at least one other node to appear online
print('Waiting for node to come online')
while len(node_monitor.get_all_node_id()) < 1:
    node.spin(timeout=1)

target_node_id = int(list(node_monitor.get_all_node_id())[0])
print("Discovered node: " + str(target_node_id))

# Set up the file server
file_server = dronecan.app.file_server.FileServer(node, lookup_paths=file_path)

update_started = False
update_complete = False

def on_node_status(e):
    global update_started
    global update_complete

    if e.transfer.source_node_id == target_node_id and e.message.mode == e.message.MODE_SOFTWARE_UPDATE \
    and e.message.health < e.message.HEALTH_ERROR:
        if not update_started:
            print('Performing update')
            update_started = True;
    else:
        if update_started:
            print('Update complete')
            update_complete = True;


def on_response(e):
    if e is not None:
        print('Firmware update response:', e.response)
        if e.response.error != e.response.ERROR_IN_PROGRESS:
            # NOTE: the timing here is fickle, we can't be too early
            # because we are trying to catch the bootloader
            node.defer(4, request_update)


def request_update():
    global update_started

    if not update_started:
        print('REQUESTING UPDATE')
        request = dronecan.uavcan.protocol.file.BeginFirmwareUpdate.Request(
                    source_node_id=node.node_id,
                    image_file_remote_path=dronecan.uavcan.protocol.file.Path(path=file_path))
        node.request(request, target_node_id, on_response, priority=30)


node_status_handle = node.add_handler(dronecan.uavcan.protocol.NodeStatus, on_node_status)

request_update()

while not update_started or not update_complete:
    try:
        node.spin(0.1)

    except KeyboardInterrupt:
        sys.exit(0)
