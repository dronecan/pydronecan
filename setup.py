#!/usr/bin/env python
#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

import os
import sys
from setuptools import setup

__version__ = None
VERSION_FILE = os.path.join(os.path.dirname(__file__), 'dronecan', 'version.py')
exec(open(VERSION_FILE).read())         # Adds __version__ to globals


args = dict(
    name='dronecan',
    version=__version__,
    description='Python implementation of the DroneCAN protocol stack',
    packages=[
        'dronecan',
        'dronecan.dsdl',
        'dronecan.driver',
        'dronecan.app',
    ],
    package_data={
        'dronecan': [os.path.join(root[len('dronecan/'):], fname)
                   for root, dirs, files in os.walk('dronecan/dsdl_files')
                   for fname in files if fname.endswith('.dronecan')]
    },
    author='Pavel Kirienko, Ben Dyer',
    author_email='dronecan.devel@gmail.com',
    url='https://github.com/dronecan/pydronecan',
    license='MIT',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Topic :: Software Development :: Libraries',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python',
    ],
    keywords=''
)

if sys.version_info[0] < 3:
    args['install_requires'] = ['monotonic']

setup(**args)
