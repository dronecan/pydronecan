#!/usr/bin/env python3
'''
 hobbywing ESC control program, for testing hobbywing ESC messages
'''

from ast import arguments
import dronecan, time, math, binascii, sys
from multiprocessing import freeze_support

# get command line arguments
from argparse import ArgumentParser

def handle_msg(msg):
    if msg is not None:
        print('REPLY: ', dronecan.to_yaml(msg))
    else:
        print("No reply")
    sys.exit(0)

def wait_online():
    '''wait for some nodes to come online'''
    print('Waiting for other nodes to become online...')
    while len(node_monitor.get_all_node_id()) < 1:
        try:
            node.spin(timeout=1)
        except Exception as ex:
            print(ex)
    print("done")

def command_XCSetBaud(arguments):
    '''set CAN baudrate'''
    req = dronecan.com.xckj.esc.SetBaud.Request()
    bmap = {
        250000:4,
        500000:5,
        800000:6,
        1000000:7,
    }
    if len(arguments) < 1:
        print("Usage: SetBaud BAUDRATE")
        return
    baudrate = int(arguments[0])
    if not baudrate in bmap:
        print("Valid baudrates: ", ','.join(bmap.keys()))
        return
    req.baud = bmap[baudrate]
    node.request(req, args.target_node_id, handle_msg)

def handle_OperateId(msg):
    '''handle OperateId reply'''
    r = msg.message
    if len(r.payload) >= 3:
        if r.payload[0] == 3:
            print("ESC[%u] NodeID=%d ThrottleNum=%d" % (msg.transfer.source_node_id, r.payload[1], r.payload[2]))

def command_OperateId(msg):
    '''Set ESC IDs'''
    req = dronecan.com.xckj.esc.OperateId()
    if len(arguments) == 3:
        print("Usage: OperateId OPTION")
        return
    req.payload[0] = int(arguments[0])
    req.payload[1] = int(arguments[1])
    req.payload[2] = int(arguments[2])
    node.add_handler(dronecan.com.xckj.esc.OperateId, handle_OperateId)
    node.broadcast(req)

def command_ThrotGroup1(arguments):
    '''send RawCommand1-4'''
    req = dronecan.com.xckj.esc.ThrotGroup1()
    if len(arguments) < 1:
        print("Usage: RawCommand THROTTLES..")
        return
    req.command = [ int(a) for a in arguments ]

    # callback for printing ESC status
    node.add_handler(dronecan.com.xckj.esc.AutoUpMsg1, lambda msg: print(dronecan.to_yaml(msg)))
    node.add_handler(dronecan.com.xckj.esc.AutoUpMsg2, lambda msg: print(dronecan.to_yaml(msg)))

    while True:
        node.broadcast(req)
        node.spin(0.02)

def command_ThrotGroup2(arguments):
    '''send RawCommand5-8'''
    req = dronecan.com.xckj.esc.ThrotGroup2()
    if len(arguments) < 1:
        print("Usage: RawCommand THROTTLES..")
        return
    req.command = [ int(a) for a in arguments ]

    # callback for printing ESC status
    node.add_handler(dronecan.com.xckj.esc.AutoUpMsg1, lambda msg: print(dronecan.to_yaml(msg)))
    node.add_handler(dronecan.com.xckj.esc.AutoUpMsg2, lambda msg: print(dronecan.to_yaml(msg)))

    while True:
        node.broadcast(req)
        node.spin(0.02)

def command_CmdControl(arguments):
    '''send cmd'''
    req = dronecan.com.xckj.esc.CmdControl()
    bmap = {
        1: req.NODECMD_DIS_ALL_UP,
        2: req.NODECMD_DIS_UP,
        3: req.NODECMD_TRIG_HB,
        4: req.NODECMD_EN_ALLUP,
        5: req.NODECMD_RST,
    }
    if len(arguments) < 1:
        print("Usage: CmdControl NODECMD")
        return
    nodecmd = int(arguments[0])
    if not nodecmd in bmap:
        print("Valid nodecmd: ", ','.join(bmap.keys()))
        return
    req.NodeCmd = bmap[nodecmd]
    req.CmdNodeId = int(arguments[1])
    req.reserved = int(0)
    node.broadcast(req)

commands = {
    "XC_SetBaud":command_XCSetBaud,    
    "OperateId":command_OperateId,
    "ThrotGroup1":command_ThrotGroup1,
    "ThrotGroup2":command_ThrotGroup2,
    "CmdControl":command_CmdControl,
}

if __name__ == "__main__":
    freeze_support()
    parser = ArgumentParser(description='XC-ESC Technology control example')
    parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
    parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
    parser.add_argument("--target-node-id", default=16, type=int, help="CAN node ID")
    parser.add_argument("--signing-key", default=None, help="MAVLink2 signing key for mavcan")
    parser.add_argument("port", default=None, type=str, help="serial port")
    parser.add_argument("command", help="command")
    parser.add_argument("args", nargs='*', help="command arguments")
    args = parser.parse_args()

    
    if not args.command in commands:
        clist = ','.join(commands.keys())
        print(f"Invalid command '{args.command}' - must be one of {clist}")
        sys.exit(1)

        # Initializing a DroneCAN node instance.
    node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

    if args.signing_key is not None:
        node.can_driver.set_signing_passphrase(args.signing_key)

    # Initializing a node monitor, so we can see what nodes are online
    node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

        
    wait_online()

    commands[args.command](args.args)

    # run for 2 seconds
    startt = time.time()
    while time.time() - startt < 2:
        try:
            node.spin(0.1)
        except Exception as ex:
            pass
