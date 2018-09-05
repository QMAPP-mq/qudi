# -*- coding: utf-8 -*-

"""
This file contains the Qudi hardware file to control a Elektro Automatik PS2000 series powersupply.

The following powersupplies are supported:
* EA-PS 2042-06B
* EA-PS 2042-10B
* EA-PS 2042-20B
* EA-PS 2084-03B
* EA-PS 2084-05B
* EA-PS 2084-10B
* EA-PS 2342-10B

Qudi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Qudi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Qudi. If not, see <http://www.gnu.org/licenses/>.

This module was developed from ps2000, written originally by marcj71.
Have a look in:
    https://github.com/marcj71/ps2000.py
A fork of that project supporting an additional model was then used, 
written by xolotl90, have a look in:
    https://github.com/xolotl90/ps2000/blob/master/ps2000.py

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import os
import sys
import struct
import serial

from core.module import Base, ConfigOption
from interface.powersupply_interface import PowersupplyInterface

class ps2000(Base, PowersupplyInterface):

    """ stabel: Matt van Breugel, Lachlan Rogers
    This is the hardware module for communicating with Elektro Automatik PS2000 series powersupplys
    over USB.

    Example configuration (Windows):
    ```
        my_powersupply:
            module.Class 'powersupply.ea_ps2000.ps2000'
            port: 'COM3'
    ```

    Example configuration (Linux):
    ```
        my_powersupply:
            module.Class 'powersupply.ea_ps2000.ps2000'
            port: '/dev/ttyACM0''
    ```
    """

    # set verbose to True to see all bytes
    _verbose = False

    # defines
    _PS_QUERY = 0x40
    _PS_SEND = 0xc0

    # nominal values, required for all voltage and current calculations
    _u_nom = 0
    _i_nom = 0

    # fetch port information
    _port = ConfigOption('port', missing='error')

    # fetch number of channels
    _channels = ConfigOption('channels', default=1, missing='warn')

    # fetch if triple output information
    _triple = ConfigOption('port', default=False, missing='warn')

    # open port upon initialization
    def on_activate(self):
        """ Initialise the hardware module.

        :return int error code (0:OK, -1:error)
        """
        # set timeout to 0.06s to guarantee minimum interval time of 50ms
        self.ser_dev = serial.Serial(self._port, timeout=0.06, baudrate=115200, parity=serial.PARITY_ODD)
        self._u_nom = self.get_nominal_voltage()
        self._i_nom = self.get_nominal_current()

        # take control of the output channels
        for channel in range(0, self._channels):
            self.set_remote(node=channel)

        return 0

    # close the door behind you
    def on_deactivate(self):
        """ Deactivate the hardware module

        :return int error code (0:OK, -1:error)
        """
        # relinquish control of the output channels
        for channel in range(0, self._channels):
            self.set_local(node=channel)

        self.ser_dev.close()
        return 0

    #
    # internal methods ##################################################
    #

    # construct telegram
    @staticmethod
    def _construct(type, node, obj, data):
        """ Construct telegram byte packet

        :param type :
        :param node :
        :param obj  :
        :param data :

        :return telegram : telegram byte packet
        """
        telegram = bytearray()
        telegram.append(0x30 + type)  # SD (start delimiter)
        telegram.append(node)  # DN (device node)
        telegram.append(obj)  # OBJ (object)
        if len(data) > 0:  # DATA
            telegram.extend(data)
            telegram[0] += len(data) - 1  # update length

        cs = 0
        for b in telegram:
            cs += b
        telegram.append(cs >> 8)  # CS0
        telegram.append(cs & 0xff)  # CS1 (checksum)

        return telegram

    # compare checksum with header and data in response from device
    @staticmethod
    def _check_checksum(ans):
        """ Compare checksum with header and data in response from device

        :param ans :

        :return bool error : (True:OK, False:error)
        """
        cs = 0
        for b in ans[0:-2]:
            cs += b
        if (ans[-2] != (cs >> 8)) or (ans[-1] != (cs & 0xff)):
            print('ERROR: checksum mismatch')
            sys.exit(1)
        else:
            return True

    # check for errors in response from device
    @staticmethod
    def _check_error(ans):
        """ Check for errors in response from device

        Will print error code is present

        :param ans :

        :return bool error : (True:OK, False:error)
        """
        if ans[2] != 0xff:
            return False

        if ans[3] == 0x00:
            # this is used as an acknowledge
            return False
        elif ans[3] == 0x03:
            print('ERROR: checksum incorrect')
        elif ans[3] == 0x04:
            print('ERROR: start delimiter incorrect')
        elif ans[3] == 0x05:
            print('ERROR: wrong address for output')
        elif ans[3] == 0x07:
            print('ERROR: object not defined')
        elif ans[3] == 0x08:
            print('ERROR: object length incorrect')
        elif ans[3] == 0x09:
            print('ERROR: access denied')
        elif ans[3] == 0x0f:
            print('ERROR: device is locked')
        elif ans[3] == 0x30:
            print('ERROR: upper limit exceeded')
        elif ans[3] == 0x31:
            print('ERROR: lower limt exceeded')

        print('answer: ', end='')
        for b in ans:
            print('%02x ' % (b), end='')
        print()
        sys.exit(1)

    # send one telegram, receive and check one response
    def _transfer(self, type, node, obj, data):
        """ Send one telegram, receive and check one response

        :param type :
        :param node :
        :param obj  :
        :param data :

        :return ans :
        """
        telegram = self._construct(type, node, obj, data)
        if self._verbose:
            print('* telegram: ', end='')
            for b in telegram:
                print('%02x ' % (b), end='')
            print()

        # send telegram
        self.ser_dev.write(telegram)

        # receive response (always ask for more than the longest answer)
        ans = self.ser_dev.read(100)

        if self._verbose:
            print('* answer:   ', end='')
            for b in ans:
                print('%02x ' % (b), end='')
            print()

        # if the answer is too short, the checksum may be missing
        if len(ans) < 5:
            print('ERROR: short answer (%d bytes received)' % len(ans))
            sys.exit(1)

        # check answer
        self._check_checksum(ans)
        self._check_error(ans)

        return ans

    # get a binary object
    def _get_binary(self, obj, node):
        """ Get a binary object

        :param obj :

        :return ans :
        """
        ans = self._transfer(self._PS_QUERY, node, obj, '')

        return ans[3:-2]

    # set a binary object
    def _set_binary(self, obj, mask, data, node):
        """ Set a binary object

        :param obj  :
        :param mask :
        :param data :

        :return ans :
        """
        ans = self._transfer(self._PS_SEND, node, obj, [mask, data])

        return ans[3:-2]

    # get a string-type object
    def _get_string(self, obj, node):
        """ Get a string-type object

        :param obj :

        :return ans :
        """
        ans = self._transfer(self._PS_QUERY, node, obj, '')

        return ans[3:-3].decode('ascii')

    # get a float-type object
    def _get_float(self, obj, node):
        """ Get a float-type object

        :param obj :

        :return ans :
        """
        ans = self._transfer(self._PS_QUERY, node, obj, '')

        return struct.unpack('>f', ans[3:-2])[0]

    # get an integer object
    def _get_integer(self, obj, node):
        """ Get an integer object

        :param obj :

        :return ans :
        """
        ans = self._transfer(self._PS_QUERY, node, obj, '')

        return (ans[3] << 8) + ans[4]

    # set an integer object
    def _set_integer(self, obj, data, node):
        """ Set an integer object

        :param obj  :
        :param data :

        :return ans :
        """
        ans = self._transfer(self._PS_SEND, node, obj, [data >> 8, data & 0xff])

        return (ans[3] << 8) + ans[4]

    #
    # public functions ##################################################
    #

    # object 0
    def get_type(self):
        return self._get_string(0, node=0)

    # object 1
    def get_serial(self):
        return self._get_string(1, node=0)

    # object 2
    def get_nominal_voltage(self, node=0):
        return self._get_float(2, node)

    # object 3
    def get_nominal_current(self, node=0):
        return self._get_float(3, node)

    # object 4
    def get_nominal_power(self, node=0):
        return self._get_float(4, node)

    # object 6
    def get_article(self, node=0):
        return self._get_string(6, node)

    # object 8
    def get_manufacturer(self, node=0):
        return self._get_string(8, node)

    # object 9
    def get_version(self, node=0):
        return self._get_string(9, node)

    # object 19
    def get_device_class(self, node=0):
        return self._get_integer(19, node)

    # object 38
    def get_OVP_threshold(self, node=0):
        return self._get_integer(38, node)

    def set_OVP_threshold(self, u, node=0):
        return self._set_integer(38, u, node)

    # object 39
    def get_OCP_threshold(self, node=0):
        return self._get_integer(39, node)

    def set_OCP_threshold(self, i, node=0):
        return self._set_integer(39, i, node)

    # object 50
    def get_voltage_setpoint(self, node=0):
        v = self._get_integer(50, node)
        return self._u_nom * v / 25600

    def set_voltage(self, u, node=0):
        return self._set_integer(50, int(round((u * 25600.0) / self._u_nom)), node)

    # object 51
    def get_current_setpoint(self, node=0):
        i = self._get_integer(50, node)
        return self._i_nom * i / 25600

    def set_current(self, i, node=0):
        return self._set_integer(51, int(round((i * 25600.0) / self._i_nom)), node)

    # object 54
    def _get_control(self, node=0):
        return self._get_binary(54, node)

    def _set_control(self, mask, data, node=0):
        ans = self._set_binary(54, mask, data, node)

        # return True if command was acknowledged ("error 0")
        return ans[0] == 0xff and ans[1] == 0x00

    def set_remote(self, remote=True, node=0):
        if remote:
            return self._set_control(0x10, 0x10, node)
        else:
            return self._set_control(0x10, 0x00, node)

    def set_local(self, local=True, node=0):
        return self.set_remote(not local, node)

    def set_output_on(self, on=True, node=0):
        if on:
            return self._set_control(0x01, 0x01, node)
        else:
            return self._set_control(0x01, 0x00, node)

    def set_output_off(self, off=True, node=0):
        return self.set_output_on(not off, node)

    # object 71
    def get_actual(self, print_state=False, node=0):
        ans = self._get_binary(71, node)

        actual = dict()
        actual['remote'] = True if ans[0] & 0x03 else False
        actual['local'] = not actual['remote']
        actual['on'] = True if ans[1] & 0x01 else False
        actual['CC'] = True if ans[1] & 0x06 else False
        actual['CV'] = not actual['CC']
        if self._triple:
        	actual['tracking'] = True if ans[1] & 0x08 else False
        actual['OVP'] = True if ans[1] & 0x10 else False
        actual['OCP'] = True if ans[1] & 0x20 else False
        actual['OPP'] = True if ans[1] & 0x40 else False
        actual['OTP'] = True if ans[1] & 0x80 else False
        actual['v'] = self._u_nom * ((ans[2] << 8) + ans[3]) / 25600
        actual['i'] = self._i_nom * ((ans[4] << 8) + ans[5]) / 25600

        if print_state:
            print("Get_actual for node %i" % node)
            if actual['remote']:
                print('remote')
            else:
                print('local')

            if actual['on']:
                print('output on')
            else:
                print('output off')

            if actual['CC']:
                print('constant current')
            else:
                print('constant voltage')

            # for dual/triple output only
            if self._triple:
                if actual['tracking']:
                    print('tracking on')
                else:
                    print('tracking off')

            if actual['OVP']:
                print('over-voltage protection active')
            else:
                print('over-voltage protection inactive')

            if actual['OCP']:
                print('over-current protection active')
            else:
                print('over-current protection active')

            if actual['OPP']:
                print('over-power protection active')
            else:
                print('over-power protection inactive')

            if actual['OTP']:
                print('over-temperature protection active')
            else:
                print('over-temperature protection inactive')

            print('actual voltage %fV' % actual['v'])
            print('actual current %fA' % actual['i'])

        return actual

#
# user example script ###############################################
#

# ps = ps2000()  # add your port here if the default does not work for you
# print('type    ' + ps.get_type())
# print('serial  ' + ps.get_serial())
# print('article ' + ps.get_article())
# print('manuf   ' + ps.get_manufacturer())
# print('version ' + ps.get_version())
# print('nom. voltage %f' % ps.get_nominal_voltage())
# print('nom. current %f' % ps.get_nominal_current())
# print('nom. power   %f' % ps.get_nominal_power())
# print('class        0x%04x' % ps.get_device_class())
# print('OVP          0x%04x' % ps.get_OVP_threshold())
# print('OCP          %d' % ps.get_OCP_threshold())
# print('control      0x%04x' % ps.set_remote())
# ps._verbose = True
# print('output       0x%04x' % ps.set_output_on())
# ps.get_actual(True, 0)
# ps.get_actual(True, 1)
# print('set voltage      %f %f' % (ps.set_voltage(12.34, node=0), ps.get_voltage_setpoint(0)))
# print('set voltage      %f %f' % (ps.set_voltage(15.51, node=1), ps.get_voltage_setpoint(1)))
# ps.get_actual(True, 0)
# ps.get_actual(True, 1)
# ps.close()
