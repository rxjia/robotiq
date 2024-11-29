# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$
#
# Modifed from the orginal comModbusTcp by Kelsey Hawkins @ Georgia Tech


"""@package docstring
Module comModbusRtu: defines a class which communicates with Robotiq Grippers using the Modbus RTU protocol.

The module depends on pymodbus (http://code.google.com/p/pymodbus/) for the Modbus RTU client.
"""

import pymodbus

if pymodbus.__version__ == "3.6.9":
    from robotiq_modbus_rtu.comModbusRtu_369 import communication
elif pymodbus.__version__ == "2.1.0":
    from robotiq_modbus_rtu.comModbusRtu_210 import communication
else:
    from robotiq_modbus_rtu.comModbusRtu_210 import communication
    print(f"Warn!!!: This code was developed with pymodbus version 2.1.0, but you are using version {pymodbus.__version__}")
