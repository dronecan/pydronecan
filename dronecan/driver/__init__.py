#
# Copyright (C) 2014-2016  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

from __future__ import division, absolute_import, print_function, unicode_literals
import sys
from .python_can import PythonCAN
from .slcan import SLCAN
try:
    from .mavcan import MAVCAN
    have_mavcan = True
except ImportError:
    have_mavcan = False
from .common import DriverError, CANFrame

if sys.platform.startswith('linux'):
    from .socketcan import SocketCAN
else:
    SocketCAN = None

__all__ = ['make_driver', 'DriverError', 'CANFrame']

def is_mavlink_port(device_name):
    '''check if a port is sending mavlink'''
    if not have_mavcan:
        return False
    return MAVCAN.is_mavlink_port(device_name)

def make_driver(device_name, **kwargs):
    """Creates an instance of CAN driver.
    The right driver class will be selected automatically based on the device_name.
    :param device_name: This parameter is used to select driver class. E.g. "/dev/ttyACM0", "COM9", "can0".
    :param kwargs: Passed directly to the constructor.
    """
    windows_com_port = device_name.replace('\\', '').replace('.', '').lower().startswith('com')
    unix_tty = device_name.startswith('/dev/')

    if device_name.startswith("mavcan:"):
        if not have_mavcan:
            raise DriverError('MAVCAN is not available, ensure pymavlink is installed')
        return MAVCAN(device_name[7:], **kwargs)
    elif device_name.startswith("slcan:"):
        return SLCAN(device_name[6:], **kwargs)
    elif windows_com_port or unix_tty:
        if is_mavlink_port(device_name):
            return MAVCAN(device_name, **kwargs)
        else:
            return SLCAN(device_name, **kwargs)
    elif PythonCAN is not None:
        return PythonCAN(device_name, **kwargs)
    elif SocketCAN is not None:
        return SocketCAN(device_name, **kwargs)
    else:
        raise DriverError('Unrecognized device name: %r' % device_name)
