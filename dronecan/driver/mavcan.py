#
# Copyright (C) 2022 DroneCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
'''
 driver for CAN over MAVLink, using MAV_CMD_CAN_FORWARD and CAN_FRAME messages
'''

import os
import sys
import time
import multiprocessing
from logging import getLogger
from .common import DriverError, CANFrame, AbstractDriver
from pymavlink import mavutil

try:
    import queue
except ImportError:
    # noinspection PyPep8Naming,PyUnresolvedReferences
    import Queue as queue

if 'darwin' in sys.platform:
    RX_QUEUE_SIZE = 32767   # http://stackoverflow.com/questions/5900985/multiprocessing-queue-maxsize-limit-is-32767
else:
    RX_QUEUE_SIZE = 1000000
TX_QUEUE_SIZE = 1000

logger = getLogger(__name__)

def io_process(url, bus, tx_queue, rx_queue):
    os.environ['MAVLINK20'] = '1'

    target_system = 1
    target_component = 0
    last_enable = time.time()
    conn = None

    def connect():
        nonlocal conn
        conn = mavutil.mavlink_connection(url, source_system=250, source_component=mavutil.mavlink.MAV_COMP_ID_MAVCAN)
        if conn is None:
            raise DriverError('unable to connect to %s' % url)

    def reconnect():
        while True:
            try:
                time.sleep(1)
                logger.info('reconnecting to %s' % url)
                connect()
                return
            except Exception:
                continue

    def enable_can_forward():
        last_enable = time.time()
        conn.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_CAN_FORWARD,
            0,
            bus+1,
            0,
            0,
            0,
            0,
            0,
            0)

    connect()
    enable_can_forward()

    while True:
        while not tx_queue.empty():
            frame = tx_queue.get()
            message_id = frame.id
            if frame.extended:
                message_id |= 1<<31
            message = frame.data
            mlen = len(message)
            if mlen < 8:
                message += bytearray([0]*(8-mlen))
            try:
                conn.mav.can_frame_send(
                    target_system,
                    target_component,
                    bus,
                    mlen,
                    message_id,
                    message)
            except Exception as ex:
                print(ex)
            if time.time() - last_enable > 1:
                enable_can_forward()

        try:
            m = conn.recv_match(type='CAN_FRAME',blocking=True,timeout=0.005)
        except Exception as ex:
            reconnect()
            continue
        if m is None:
            continue
        is_extended = (m.id & (1<<31)) != 0
        canid = m.id & 0x1FFFFFFF
        frame = CANFrame(canid, m.data[:m.len], is_extended)
        rx_queue.put_nowait(frame)


# MAVLink CAN driver
#
class MAVCAN(AbstractDriver):
    """
    Driver for MAVLink CAN bus adapters, using CAN_FRAME MAVLink packets
    """

    def __init__(self, url, **kwargs):
        super(MAVCAN, self).__init__()
        self.bus = kwargs.get('bus_number', 1) - 1

        self.rx_queue = multiprocessing.Queue(maxsize=RX_QUEUE_SIZE)
        self.tx_queue = multiprocessing.Queue(maxsize=TX_QUEUE_SIZE)

        self.proc = multiprocessing.Process(target=io_process, name='mavcan_io_process',args=(url, self.bus, self.tx_queue, self.rx_queue))
        self.proc.daemon = True
        self.proc.start()

    def close(self):
        pass

    def __del__(self):
        self.close()

    def receive(self, timeout=None):
        tstart = time.time()
        while True:
            try:
                frame = self.rx_queue.get(block=0)
            except queue.Empty:
                frame = None
            if frame is not None:
                self._rx_hook(frame)
                return frame
            if timeout is not None:
                timeout = max(timeout, 0.001)
                if time.time() >= tstart + timeout:
                    return

    def send(self, message_id, message, extended=False):
        frame = CANFrame(message_id, message, extended)
        self._tx_hook(frame)
        self.tx_queue.put_nowait(frame)

    def is_mavlink_port(device_name):
        '''check if a device is sending mavlink'''
        os.environ['MAVLINK20'] = '1'
        conn = mavutil.mavlink_connection(device_name, source_system=250, source_component=mavutil.mavlink.MAV_COMP_ID_MAVCAN)
        if not conn:
            return False
        m = conn.recv_match(blocking=True, type=['HEARTBEAT','ATTITUDE', 'SYS_STATUS'], timeout=1.1)
        return m is not None
