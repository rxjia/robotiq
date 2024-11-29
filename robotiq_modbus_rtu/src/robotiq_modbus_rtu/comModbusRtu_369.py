import pymodbus

if pymodbus.__version__ != "3.6.9":
    print(
        f"Warning: This code was developed with pymodbus version 3.6.9, but you are using version {pymodbus.__version__}"
    )

from math import ceil

from pymodbus import (
    ExceptionResponse,
    Framer,
    ModbusException,
    pymodbus_apply_logging_config,
)
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse

# activate debugging
# pymodbus_apply_logging_config("DEBUG")


class communication:

    def __init__(self):
        self.client = None
        self.device = None

    def connectToDevice(self, device):
        """Connection to the client - the method takes the IP address (as a string, e.g. '192.168.1.11') as an argument."""
        self.device = device
        self.client = ModbusClient(
            device,
            framer=Framer.RTU,
            baudrate=115200,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.2,
            retries=3,
            # handle_local_echo=False,
        )
        if not self.client.connect():
            print("Unable to connect to {}".format(device))
            return False
        return True

    def disconnectFromDevice(self):
        """Close connection"""
        self.client.close()

    def connect(self):
        if self.client is None:
            print("Client is not initialized")
            return False
        if not self.client.connect():
            print("Unable to connect to {}".format(self.device))
            return False
        return True

    def sendCommand(self, data):
        """Send a command to the Gripper - the method takes a list of uint8 as an argument. The meaning of each variable depends on the Gripper model (see support.robotiq.com for more details)"""

        if not self.connect():
            return

        # make sure data has an even number of elements
        if len(data) % 2 == 1:
            data.append(0)

        # Initiate message as an empty list
        message = []

        # Fill message by combining two bytes in one register
        for i in range(0, len(data) // 2):
            message.append((data[2 * i] << 8) + data[2 * i + 1])

        # To do!: Implement try/except
        self.client.write_registers(0x03E8, message, slave=0x0009)

    def getStatus(self, numBytes):
        """Sends a request to read, wait for the response and returns the Gripper status. The method gets the number of bytes to read as an argument"""

        if not self.connect():
            return None

        numRegs = int(ceil(numBytes / 2.0))

        # Get status from the device
        try:
            rr = self.client.read_holding_registers(0x07D0, count=numRegs, slave=0x0009)
        except ModbusException as exc:
            print(f"Received ModbusException({exc}) from library")
            self.client.close()
            return None

        if rr.isError():
            print(f"Received exception from device ({rr})")
            # THIS IS NOT A PYTHON EXCEPTION, but a valid modbus message
            self.client.close()
            return None

        if isinstance(rr, ReadHoldingRegistersResponse):
            # Instantiate output as an empty list
            output = []

            # Fill the output with the bytes in the appropriate order
            for i in range(0, numRegs):
                output.append((rr.getRegister(i) & 0xFF00) >> 8)
                output.append(rr.getRegister(i) & 0x00FF)

            # Output the result
            return output
        else:
            print(f"Warn! Received response: {rr}")
            self.client.close()
            return None
