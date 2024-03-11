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

# protocol constants
PREAMBLE1 = 0xb5
PREAMBLE2 = 0x62

CLASS_ACK = 0x05
CLASS_CFG = 0x06

MSG_CFG_PRT = 0x00

MSG_ACK_NACK = 0x00
MSG_ACK_ACK = 0x01

# class UBloxError(Exception):
#     '''Ublox error class'''
#     def __init__(self, msg):
#         Exception.__init__(self, msg)
#         self.message = msg

# class UBloxDescriptor:
#     '''class used to describe the layout of a UBlox message'''
#     def __init__(self, name, msg_format, fields=[], count_field=None, format2=None, fields2=None): 
#         '''
#         (CLASS_CFG, MSG_CFG_PRT)    : UBloxDescriptor('CFG_PRT',
#                                                     '<B,<BHIIHHHH',
#                                                     ['portID', 'reserved0', 'txReady', 'mode', 'baudRate', 'inProtoMask', 
#                                                     'outProtoMask', 'reserved4', 'reserved5']),
#         (CLASS_ACK, MSG_ACK_ACK)    : UBloxDescriptor('ACK_ACK',
#                                                     '<BB', 
#                                                     ['clsID', 'msgID']),
#         (CLASS_ACK, MSG_ACK_NACK)   : UBloxDescriptor('ACK_NACK',
#                                                     '<BB', 
#                                                     ['clsID', 'msgID']),
#         '''
#         self.name = name
#         self.msg_format = msg_format
#         self.fields = fields
#         self.count_field = count_field
#         self.format2 = format2
#         self.fields2 = fields2

#     def ArrayParse(self, field):
#         '''parse an array descriptor'''
#         arridx = field.find('[')
#         if arridx == -1:
#             return (field, -1)
#         alen = int(field[arridx+1:-1])
#         fieldname = field[:arridx]
#         return (fieldname, alen)

#     def unpack(self, msg):
#         '''unpack a UBloxMessage, creating the .fields and ._recs attributes in msg'''
#         msg._fields = {}

#         # unpack main message blocks. A comm
#         formats = self.msg_format.split(',')
#         buf = msg._buf[6:-2]
#         count = 0
#         msg._recs = []
#         fields = self.fields[:]
#         for fmt in formats:
#             size1 = struct.calcsize(fmt)
#             if size1 > len(buf):
#                 raise UBloxError("%s INVALID_SIZE1: %u>%u fmt='%s'" % (self.name, size1, len(buf), fmt))
#             f1 = list(struct.unpack(fmt, buf[:size1])) # unpack the message
#             i = 0
#             while i < len(f1):
#                 field = fields.pop(0)
#                 (fieldname, alen) = self.ArrayParse(field) 
#                 if alen == -1:
#                     msg._fields[fieldname] = f1[i]
#                     if self.count_field == fieldname: 
#                         count = int(f1[i]) 
#                     i += 1
#                 else:
#                     msg._fields[fieldname] = [0]*alen
#                     for a in range(alen):
#                         msg._fields[fieldname][a] = f1[i]
#                         i += 1
#             buf = buf[size1:]
#             if len(buf) == 0:
#                 break

#         if self.count_field == '_remaining':
#             count = len(buf) / struct.calcsize(self.format2)

#         if count == 0:
#             msg._unpacked = True
#             #if len(buf) != 0:
#             #    raise UBloxError("EXTRA_BYTES=%u" % len(buf))
#             return

#         size2 = struct.calcsize(self.format2)
#         for _ in range(count):
#             r = UBloxAttrDict()
#             if size2 > len(buf):
#                 raise UBloxError("INVALID_SIZE=%u, " % len(buf))
#             f2 = list(struct.unpack(self.format2, buf[:size2]))
#             for i in range(len(self.fields2)):
#                 r[self.fields2[i]] = f2[i]
#             buf = buf[size2:]
#             msg._recs.append(r)
#         if len(buf) != 0:
#             raise UBloxError("EXTRA_BYTES=%u" % len(buf))
#         msg._unpacked = True

#     def pack(self, msg, msg_class=None, msg_id=None):
#         '''pack a UBloxMessage from the .fields and ._recs attributes in msg'''
#         f1 = []
#         if msg_class is None:
#             msg_class = msg.msg_class()
#         if msg_id is None:
#             msg_id = msg.msg_id()
#         msg._buf = ''

#         fields = self.fields[:]
#         for f in fields:
#             (fieldname, alen) = self.ArrayParse(f)
#             if not fieldname in msg._fields:
#                 break
#             if alen == -1:
#                 f1.append(msg._fields[fieldname])
#             else:
#                 for a in range(alen):
#                     f1.append(msg._fields[fieldname][a])                    
#         try:
#             # try full length message
#             if self.msg_format.find(',') != -1:
#                 fmt = self.msg_format.replace(',', '')
#                 msg._buf = struct.pack(fmt, *tuple(f1))
#             else:
#                 raise Exception
#         except Exception as e:
#             # try without optional part
#             fmt = self.msg_format.split(',')[0]
#             msg._buf = struct.pack(fmt, *tuple(f1))

#         length = len(msg._buf)
#         if msg._recs: 
#             length += len(msg._recs) * struct.calcsize(self.format2)
#         header = struct.pack('<BBBBH', PREAMBLE1, PREAMBLE2, msg_class, msg_id, length) # pack the header
#         msg._buf = header + msg._buf

#         for r in msg._recs: # pack the records
#             f2 = []
#             for f in self.fields2:
#                 f2.append(r[f])
#             msg._buf += struct.pack(self.format2, *tuple(f2))            
#         msg._buf += struct.pack('<BB', *msg.checksum(data=msg._buf[2:]))

#     def format(self, msg):
#         '''return a formatted string for a message'''
#         if not msg._unpacked:
#             self.unpack(msg)
#         ret = self.name + ': '
#         for f in self.fields:
#             (fieldname, alen) = self.ArrayParse(f)
#             if not fieldname in msg._fields:
#                 continue
#             v = msg._fields[fieldname]
#             if isinstance(v, list):
#                 ret += '%s=[' % fieldname
#                 for a in range(alen):
#                     ret += '%s, ' % v[a]
#                 ret = ret[:-2] + '], '
#             elif isinstance(v, str):
#                 ret += '%s="%s", ' % (f, v.rstrip(' \0'))
#             else:
#                 ret += '%s=%s, ' % (f, v)
#         for r in msg._recs:
#             ret += '[ '
#             for f in self.fields2:
#                 v = r[f]
#                 ret += '%s=%s, ' % (f, v)
#             ret = ret[:-2] + ' ], '
#         return ret[:-2]

# class UBloxMessage:
#     '''UBlox message class - holds a UBX binary message'''
#     def __init__(self, msg_class=None, msg_id=None, payload=None):
#         self._buf = bytearray()
#         self._fields = {}
#         self._recs = []
#         self._unpacked = False
#         self.debug_level = 0
#         if msg_class is not None and msg_id is not None and payload is not None:
#             self.pack_message(msg_class, msg_id, payload)

#     def __str__(self):
#         '''format a message as a string'''
#         return self.name()

#     def __getattr__(self, name):
#         '''allow access to message fields'''
#         try:
#             return self._fields[name]
#         except KeyError:
#             if name == 'recs':
#                 return self._recs
#             raise AttributeError(name)

#     def __setattr__(self, name, value):
#         '''allow access to message fields'''
#         if name.startswith('_'):
#             self.__dict__[name] = value
#         else:
#             self._fields[name] = value 

#     def have_field(self, name):
#         '''return True if a message contains the given field'''
#         return name in self._fields

#     def debug(self, level, msg):
#         '''write a debug message'''
#         if self.debug_level >= level:
#             print(msg)

#     def name(self):
#         '''return the short string name for a message'''
#         return "UBX(0x%02x,0x%02x,len=%u)" % (self.msg_class(), self.msg_id(), self.msg_length())

#     def msg_class(self):
#         '''return the message class'''
#         return self._buf[2]

#     def msg_id(self):
#         '''return the message id within the class'''
#         return self._buf[3]

#     def msg_type(self):
#         '''return the message type tuple (class, id)'''
#         return (self.msg_class(), self.msg_id())

#     def msg_length(self):
#         '''return the payload length'''
#         (payload_length,) = struct.unpack('<H', self._buf[4:6])
#         return payload_length

#     def valid_so_far(self):
#         '''check if the message is valid so far'''
#         if len(self._buf) > 0 and self._buf[0] != PREAMBLE1:
#             return False
#         if len(self._buf) > 1 and self._buf[1] != PREAMBLE2:
#             return False
#         needed = self.needed_bytes()
#         if needed > 1000:
#             return False
#         if needed == 0 and not self.valid():
#             return False
#         return True

#     def add(self, buf):
#         '''add some bytes to a message'''
#         self._buf += buf
#         while not self.valid_so_far() and len(self._buf) > 0:
#             '''handle corrupted streams'''
#             self._buf = self._buf[1:]
#         if self.needed_bytes() < 0:
#             self._buf = bytearray()

#     def checksum(self, data=None):
#         '''return a checksum tuple for a message'''
#         if data is None:
#             data = self._buf[2:-2]
#         cs = 0
#         ck_a = 0
#         ck_b = 0
#         for i in data:
#             ck_a = (ck_a + i) & 0xFF
#             ck_b = (ck_b + ck_a) & 0xFF
#         return (ck_a, ck_b)

#     def valid_checksum(self):
#         '''check if the checksum is OK'''
#         (ck_a, ck_b) = self.checksum()
#         d = self._buf[2:-2]
#         (ck_a2, ck_b2) = struct.unpack('<BB', self._buf[-2:])
#         return ck_a == ck_a2 and ck_b == ck_b2 

#     def needed_bytes(self):
#         '''return number of bytes still needed'''
#         if len(self._buf) < 6:
#             return 8 - len(self._buf)
#         return self.msg_length() + 8 - len(self._buf)

#     def valid(self):
#         '''check if a message is valid'''
#         return len(self._buf) >= 8 and self.needed_bytes() == 0 and self.valid_checksum()

#     def raw(self):
#         '''return the raw bytes'''
#         return self._buf

#     def pack_message(self, msg_class, msg_id, payload):
#         self._buf = struct.pack('<BBBBH', PREAMBLE1, PREAMBLE2, msg_class, msg_id, len(payload))
#         self._buf += payload
#         (ck_a, ck_b) = self.checksum(self._buf[2:])
#         self._buf += struct.pack('<BB', ck_a, ck_b)

# # list of supported message types.
# msg_types = {
#     (CLASS_CFG, MSG_CFG_PRT)    : UBloxDescriptor('CFG_PRT',
#                                                   '<B,<BHIIHHHH',
#                                                   ['portID', 'reserved0', 'txReady', 'mode', 'baudRate', 'inProtoMask', 
#                                                    'outProtoMask', 'reserved4', 'reserved5']),
#     (CLASS_ACK, MSG_ACK_ACK)    : UBloxDescriptor('ACK_ACK',
#                                                   '<BB', 
#                                                   ['clsID', 'msgID']),
#     (CLASS_ACK, MSG_ACK_NACK)   : UBloxDescriptor('ACK_NACK',
#                                                   '<BB', 
#                                                   ['clsID', 'msgID']),
# }

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