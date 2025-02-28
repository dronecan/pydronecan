#
# Copyright (C) 2023 DroneCAN Development Team  <dronecan.org>
#
# This software is distributed under the terms of the MIT License.
#
'''
 driver for CAN over Serial

 packet format is:
   - MAGIC 0x2934 16 bit
   - 16 bit CRC CRC-16-CCITT (over all bytes after CRC)
   - 16 bit flags (where 6 MSB bits are used for data length)
   - 32 bit message ID
   - data[]

 all data is little endian

 standard URLs
   sercan:/dev/ttyACM0
   sercan:COM1
'''

SERCAN_MAGIC = 0x2934
SERCAN_FLAG_CANFD = 0x0001
SERCAN_LENGTH_MASK = 0xFC00  # 6 MSB bits for length
SERCAN_LENGTH_SHIFT = 10     # Shift to get/set length in flags

import os
import sys
import time
import multiprocessing
import struct
import errno
from logging import getLogger
import dronecan.dsdl.common as common
from .common import DriverError, TxQueueFullError, CANFrame, AbstractDriver

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

# If PySerial isn't available, we can't support SLCAN
try:
    import serial
except ImportError:
    serial = None
    logger.info("Cannot import PySerial; SLCAN will not be available.")


def io_process(device_name, baudrate, timeout, tx_queue, rx_queue, exit_queue, parent_pid):
    """Process for handling serial I/O operations"""
    
    serial_port = None
    rx_buffer = bytearray()
    last_reconnect_attempt = 0
    reconnect_interval = 1.0  # seconds between reconnection attempts
    exit_proc = False
    
    def connect():
        """Try to connect to the serial port"""
        nonlocal serial_port
        try:
            if serial_port is not None:
                try:
                    serial_port.close()
                except Exception:
                    pass
            # we rather strangely set the baudrate initially to 1200, then change to the desired
            # baudrate. This works around a kernel bug on some Linux kernels where the baudrate
            # is not set correctly
            serial_port = serial.Serial(
                port=device_name,
                baudrate=1200,
                timeout=0,
                dsrdtr=False, rtscts=False, xonxoff=False,
                exclusive=True
            )
            try:
                serial_port.setBaudrate(baudrate)
            except Exception:
                # for pySerial 3.0, which doesn't have setBaudrate()
                serial_port.baudrate = baudrate
            logger.info(f"SerialCAN: Connected to {device_name}")
            return True
        except serial.SerialException as ex:
            logger.warning(f"SerialCAN: Failed to connect to {device_name}: {ex}")
            serial_port = None
            return False
    
    def process_rx_buffer():
        """Process received data in the buffer"""
        nonlocal rx_buffer
        
        # Need at least header size (10 bytes) to check for a complete packet
        while len(rx_buffer) >= 10:
            # Check for magic number
            magic, = struct.unpack("<H", rx_buffer[0:2])
            if magic != SERCAN_MAGIC:
                # No magic number at start, remove first byte and continue
                rx_buffer.pop(0)
                continue
            
            # Extract header
            magic, crc16, flags, message_id = struct.unpack("<HHHI", rx_buffer[0:10])
            
            # Extract data length from flags
            data_len = (flags & SERCAN_LENGTH_MASK) >> SERCAN_LENGTH_SHIFT
            
            # Check if we have the complete packet
            if len(rx_buffer) < 10 + data_len:
                # Not enough data yet
                return
            
            # Get the packet body (flags, message_id, data)
            body = rx_buffer[4:10+data_len]
            
            # Verify CRC
            if crc16 != common.crc16_from_bytes(body):
                # CRC error, remove first byte and continue
                logger.debug("SerialCAN: CRC error")
                rx_buffer.pop(0)
                continue
            
            # Extract data
            data = rx_buffer[10:10+data_len]
            
            # Remove the processed packet from the buffer
            rx_buffer = rx_buffer[10+data_len:]
            
            # Create and queue frame
            is_extended = (message_id & (1<<31)) != 0
            is_canfd = flags & SERCAN_FLAG_CANFD != 0
            canid = message_id & 0x1FFFFFFF
            frame = CANFrame(canid, bytes(data), is_extended, 
                            ts_monotonic=time.monotonic(), 
                            ts_real=time.time(), 
                            canfd=is_canfd)
            
            rx_queue.put_nowait(frame)
    
    # Initial connection attempt
    connect()
    
    # Main loop
    while True:
        # Check for exit command
        if not exit_queue.empty() and exit_queue.get() == "QUIT":
            if serial_port is not None:
                serial_port.close()
            return
        
        # Check if parent process is alive
        if os.getppid() != parent_pid:
            # Ensure we die when parent dies
            if serial_port is not None:
                serial_port.close()
            return
        
        # Check if we need to reconnect
        if serial_port is None:
            current_time = time.monotonic()
            if current_time - last_reconnect_attempt >= reconnect_interval:
                last_reconnect_attempt = current_time
                connect()
            time.sleep(0.1)  # Don't consume CPU while waiting to reconnect
            continue
        
        # Process transmit queue
        while not tx_queue.empty():
            try:
                if (not exit_queue.empty() and exit_queue.get() == "QUIT") or exit_proc:
                    serial_port.close()
                    return
                frame = tx_queue.get()
                # Prepare message flags with embedded data length
                data_len = len(frame.data)
                flags = SERCAN_FLAG_CANFD if frame.canfd else 0
                flags |= (data_len << SERCAN_LENGTH_SHIFT) & SERCAN_LENGTH_MASK
                
                # Prepare message ID
                message_id = frame.id
                if frame.extended:
                    message_id |= 1<<31
                
                # Construct packet body
                body = struct.pack("<HI", flags, message_id) + frame.data
                
                # Add header with CRC
                hdr = struct.pack("<HH", SERCAN_MAGIC, common.crc16_from_bytes(body))
                
                # Complete packet
                pkt = hdr + body
                
                # Send packet
                serial_port.write(bytes(pkt))
                serial_port.flush()
            except queue.Empty:
                break
            except (serial.SerialException, OSError) as ex:
                logger.warning(f"SerialCAN: TX error: {ex}")
                try:
                    serial_port.close()
                except Exception:
                    pass
                serial_port = None
                rx_buffer.clear()  # Clear buffer on disconnect
                break
        
        # Read from serial port
        if serial_port is not None:
            try:
                if serial_port.in_waiting:
                    data = serial_port.read(serial_port.in_waiting)
                    rx_buffer.extend(data)
                    process_rx_buffer()
            except (serial.SerialException, OSError) as ex:
                logger.warning(f"SerialCAN: RX error: {ex}")
                try:
                    serial_port.close()
                except Exception:
                    pass
                serial_port = None
                rx_buffer.clear()  # Clear buffer on disconnect


class sercan(AbstractDriver):
    """
    Driver for Serial CAN
    """
    
    @staticmethod
    def is_sercan_port(device_name, baudrate=115200, timeout=1.0):
        """
        Check if the given serial port is a SerialCAN port by looking for 3 valid packets
        
        :param device_name: Serial port name
        :param baudrate: Baud rate to use
        :param timeout: Detection timeout in seconds
        :return: True if it's a SerialCAN port, False otherwise
        """
        try:
            # Try to open the serial port
            port = serial.Serial(
                port=device_name,
                baudrate=1200,
                timeout=0,
                dsrdtr=False, rtscts=False, xonxoff=False,
                exclusive=True
            )
            try:
                port.setBaudrate(baudrate)
            except Exception:
                # for pySerial 3.0, which doesn't have setBaudrate()
                port.baudrate = baudrate
            # Wait for data
            end_time = time.monotonic() + timeout
            buffer = bytearray()
            valid_packets_found = 0
            required_packets = 3
            
            while time.monotonic() < end_time:
                if port.in_waiting:
                    data = port.read(port.in_waiting)
                    buffer.extend(data)
                    
                    # Process the buffer for valid packets
                    while len(buffer) >= 10:  # Minimum header size
                        # Check for magic number
                        if buffer[0] != (SERCAN_MAGIC & 0xFF) or buffer[1] != (SERCAN_MAGIC >> 8):
                            buffer.pop(0)
                            continue
                            
                        # Extract header
                        magic, crc16, flags, message_id = struct.unpack("<HHHI", buffer[0:10])
                        
                        # Extract data length from flags
                        data_len = (flags & SERCAN_LENGTH_MASK) >> SERCAN_LENGTH_SHIFT
                        
                        # Check if we have the complete packet
                        if len(buffer) < 10 + data_len:
                            # Not enough data yet
                            break
                        
                        # Get the packet body (flags, message_id, data)
                        body = buffer[4:10+data_len]
                        
                        # Verify CRC
                        if crc16 == common.crc16_from_bytes(body):
                            # Valid packet found
                            valid_packets_found += 1
                            logger.debug(f"SerialCAN: Valid packet found ({valid_packets_found}/{required_packets})")
                            
                            if valid_packets_found >= required_packets:
                                port.close()
                                return True
                        
                        # Remove the processed packet from the buffer
                        buffer = buffer[10+data_len:]
                
                time.sleep(0.01)
                
            port.close()
            return False
            
        except serial.SerialException:
            return False
    
    def __init__(self, device_name, **kwargs):
        """
        :param device_name: Serial port name, e.g. '/dev/ttyACM0' or 'COM1'
        :param kwargs: Additional arguments for serial port configuration
        """
        super(sercan, self).__init__()
        
        # Store parameters for connection
        self._device_name = device_name
        self._baudrate = kwargs.get('baudrate', 115200)
        self._timeout = kwargs.get('timeout', 0.1)
        
        # Create queues for communication
        self.rx_queue = multiprocessing.Queue(maxsize=RX_QUEUE_SIZE)
        self.tx_queue = multiprocessing.Queue(maxsize=TX_QUEUE_SIZE)
        self.exit_queue = multiprocessing.Queue(maxsize=1)
        
        # Start I/O process
        self.proc = multiprocessing.Process(
            target=io_process,
            name='sercan_io_process',
            args=(self._device_name, self._baudrate, self._timeout, 
                  self.tx_queue, self.rx_queue, self.exit_queue, os.getpid())
        )
        self.proc.daemon = True
        self.proc.start()
    
    def close(self):
        """Close the driver and terminate the I/O process"""
        if hasattr(self, 'proc') and self.proc is not None:
            self.exit_queue.put_nowait("QUIT")
            self.proc.join(timeout=1.0)
            self.proc = None
    
    def __del__(self):
        """Ensure resources are properly released"""
        self.close()
    
    def receive(self, timeout=None):
        """
        Receive a CAN frame
        
        :param timeout: Timeout in seconds, None for indefinite
        :return: Received CANFrame or None if timeout occurred
        """
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
    
    def send_frame(self, frame):
        """
        Send a CAN frame
        
        :param frame: CANFrame to send
        :raises: TxQueueFullError if send queue is full
        """
        try:
            self._tx_hook(frame)
            self.tx_queue.put_nowait(frame)
        except queue.Full:
            raise TxQueueFullError()
