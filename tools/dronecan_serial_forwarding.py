import dronecan
import time 
from functools import partial
import socket
import errno
import logging
import struct
from argparse import ArgumentParser
import threading

parser = ArgumentParser(description='Serial forwarder')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("--baudrate", default=115200, type=int, help="Serial baud rate")
parser.add_argument("--device", "-d", default=None, type=str, help="Serial device")
parser.add_argument("--port", default=2001, type=int, help="Listen Port")
parser.add_argument("--node-id", default=None, nargs="+", type=int, help="Node ID")
global args
args = parser.parse_args()

if args.node_id is None:
    print("No node ID specified")
    exit(1)

if args.device is None:
    print("No device specified")
    exit(1)
    
class serialForwarding():
    def __init__(self, node, nodeid, instance):
        self.sock = None
        self.listen_sock = None
        self.addr = None
        self.num_rx_bytes = 0
        self.num_tx_bytes = 0
        self.node = node
        self.node_id = nodeid
        self.i = instance
        try:
            self.device = args.device
        except Exception as e:
            print(e)
            return
        self.baudrate = args.baudrate
        self.port = args.port + instance
        print(self.device, self.port)
        self.state = "disconnected"
        self.tunnel = None
        self.target_dev = -1
        self.ublox_handling = False
        self.restart_listen()
        timer = threading.Timer(0.01, self.check_connection)
        timer.start()

    def restart_listen(self):
        '''stop and restart listening socket'''
        if self.listen_sock is not None:
            self.listen_sock.close()
        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
        self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen_sock.bind(('', int(self.port)))
        self.listen_sock.setblocking(False)
        self.listen_sock.listen(1)
        self.state = "disconnected"

    def process_socket(self):
        '''process data from the socket'''
        while True:
            try:
                buf = self.sock.recv(120)
            except socket.error as ex:
                if ex.errno not in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                    print("ucenter socket fail")
                    self.close_socket()
                return
            except Exception as e:
                print(e)
                self.close_socket()
                return

            if buf is None or len(buf) == 0:
                break
            self.tunnel.write(buf)
            self.num_tx_bytes += len(buf)
            print("     " * self.i + "port:" + str(args.port + self.i) + " tx_bytes: %u" % self.num_tx_bytes)

    def process_tunnel(self):
        '''process data from the tunnel'''
        while True:
            if self.tunnel is None:
                return
            buf = self.tunnel.read(120)
            if buf is None or len(buf) == 0:
                break
            try:
                self.sock.send(buf)
            except Exception as e:
                print(e)
                self.close_socket()
                return

            self.num_rx_bytes += len(buf)
            print("     " * self.i + "port:" + str(args.port + self.i) + " rx_bytes: %u" % self.num_rx_bytes)

    def close_socket(self):
        '''close the socket on errors'''
        print("Closing socket")
        self.state = "disconnected"
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        if self.tunnel is not None:
            self.tunnel.close()
            self.tunnel = None

    def check_connection(self):
        '''called at 100Hz to process data'''
        timer = threading.Timer(0.01, self.check_connection)
        timer.start()
        if self.sock is not None:
            self.process_socket()
            self.process_tunnel()

        if self.sock is None:
            try:
                sock, self.addr = self.listen_sock.accept()
            except Exception as e:
                if e.errno not in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                    print("ucenter listen fail")
                    self.restart_listen()
                    return
                return
            self.sock = sock
            self.sock.setblocking(False)
            self.state = "connection from %s:%u" % (self.addr[0], self.addr[1])
            self.num_rx_bytes = 0
            self.num_tx_bytes = 0
            if self.tunnel is not None:
                self.tunnel.close()
            target_node = self.node_id
            self.tunnel = dronecan.DroneCANSerial(self.device, target_node, self.target_dev,
                                                node=self.node, baudrate=self.baudrate)

            print("ucenter connection from %s" % str(self.addr))

def getNode():
    # Trying to start the node on the specified interface
    try:
        node_info = dronecan.uavcan.protocol.GetNodeInfo.Response()
        node_info.name = 'org.dronecan.gui_tool'
        node_info.software_version.major = 1
        node_info.software_version.minor = 2
        kwargs = {
            "baudrate": args.baudrate,
            "bitrate": args.bitrate,
            "bus_number": 1,
            "mavlink_target_system": 0,
            "mavlink_signing_key": None,
        }
        if args.device is None:
            raise Exception('No device specified')
        node = dronecan.make_node(can_device_name=args.device, node_info=node_info, mode=dronecan.uavcan.protocol.NodeStatus().MODE_OPERATIONAL,
                                **kwargs)
        node.node_id = 127
        # Making sure the interface is alright
        node.spin(0.1)
    except dronecan.transport.TransferError as ex:
        # allow unrecognized messages on startup:
        print('DroneCAN Transfer Error occurred on startup')
        print(ex)
        exit(1)
    except Exception as ex:
        print('DroneCAN node init failed')
        print('Fatal error', 'Could not initialize DroneCAN node')
        print(ex)
        exit(1)
    else:
        return node

if __name__ == "__main__":
    try:
        node = getNode()
        for i in range(len(args.node_id)):
            target=serialForwarding(node, args.node_id[i], i)
    except Exception as e:
        print(e)
