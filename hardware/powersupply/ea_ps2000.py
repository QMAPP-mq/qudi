# -*- coding: utf-8 -*-

"""
This file contains the Qudi hardware file to control a Elektro Automatik PS2000 series powersupply.

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

This module was developed from PyAPT, written originally by marcj71.
Have a look in:
    https://github.com/marcj71/ps2000.py

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import os
import sys
import struct
import serial

class ps2000(Base, PowersupplyInterface):

    # set verbose to True to see all bytes
    _verbose = False

    # defines
    _PS_QUERY = 0x40
    _PS_SEND  = 0xc0

    # nominal values, required for all voltage and current calculations
    _u_nom = 0
    _i_nom = 0

    # open port upon initialization
    def on_activate(self): # , port)
        # TODO make more general
        port = 'COM3'
        # set timeout to 0.06s to guarantee minimum interval time of 50ms
        self.ser_dev = serial.Serial(port, timeout=0.06, baudrate=115200, parity=serial.PARITY_ODD)
        self._u_nom = self.get_nominal_voltage()
        self._i_nom = self.get_nominal_current()

    # close the door behind you
    def on_deactivate(self):
        self.ser_dev.close()

    # construct telegram
    def _construct(self, type, node, obj, data):
        telegram = bytearray()
        telegram.append(0x30 + type)	# SD (start delimiter)
        telegram.append(node)		# DN (device node)
        telegram.append(obj)		# OBJ (object)
        if len(data) > 0:		# DATA
            telegram.extend(data)
            telegram[0] += len(data) - 1	# update length

        cs = 0
        for b in telegram:
            cs += b
        telegram.append(cs >> 8)	# CS0
        telegram.append(cs & 0xff)	# CS1 (checksum)
        
        return telegram

    # compare checksum with header and data in response from device
    def _check_checksum(self, ans):
        cs = 0
        for b in ans[0:-2]:
            cs += b
        if (ans[-2] != (cs >> 8)) or (ans[-1] != (cs & 0xff)):
            print('ERROR: checksum mismatch')
            sys.exit(1)
            return False
        else:
            return True

    # check for errors in response from device
    def _check_error(self, ans):
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
        return True

    # send one telegram, receive and check one response
    def _transfer(self, type, node, obj, data):
        telegram = self._construct(type, 0, obj, data)
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
    def _get_binary(self, obj):
        ans = self._transfer(self._PS_QUERY, 0, obj, '')

        return ans[3:-2]

    # set a binary object
    def _set_binary(self, obj, mask, data):
        ans = self._transfer(self._PS_SEND, 0, obj, [mask, data])

        return ans[3:-2]

    # get a string-type object
    def _get_string(self, obj):
        ans = self._transfer(self._PS_QUERY, 0, obj, '')

        return ans[3:-3].decode('ascii')

    # get a float-type object
    def _get_float(self, obj):
        ans = self._transfer(self._PS_QUERY, 0, obj, '')

        return struct.unpack('>f', ans[3:-2])[0]

    # get an integer object
    def _get_integer(self, obj):
        ans = self._transfer(self._PS_QUERY, 0, obj, '')

        return (ans[3] << 8) + ans[4]

    # set an integer object
    def _set_integer(self, obj, data):
        ans = self._transfer(self._PS_SEND, 0, obj, [data >> 8, data & 0xff])

        return (ans[3] << 8) + ans[4]

    #
    # public functions ##################################################
    #

    # object 0
    def get_type(self):
        return self._get_string(0)
        
    # object 1
    def serial(self):
        return self._get_string(1)

    # object 2
    def nominal_voltage(self):
        return self._get_float(2)

    # object 3
    def nominal_current(self):
        return self._get_float(3)

    # object 4
    def nominal_power(self):
        return self._get_float(4)

    # object 6
    def article(self):
        return self._get_string(6)

    # object 8
    def manufacturer(self):
        return self._get_string(8)

    # object 9
    def version(self):
        return self._get_string(9)

    # object 19
    def device_class(self):
        return self._get_integer(19)

    # object 38
    @property
    def OVP_threshold(self):
        return self._get_integer(38)

    @OVP_threshold.setter
    def OVP_threshold(self, u):
        return self._set_integer(38, u)

    # # object 38
    # def get_OVP_threshold(self):
    #     return self._get_integer(38)

    # def set_OVP_threshold(self, u):
    #     return self._set_integer(38, u)

    # object 39
    @property
    def OCP_threshold(self):
        return self._get_integer(39)

    @OCP_threshold.setter
    def OCP_threshold(self, i):
        return self._set_integer(39, i)

    # # object 39
    # def get_OCP_threshold(self):
    #     return self._get_integer(39)

    # def set_OCP_threshold(self, i):
    #     return self._set_integer(39, i)

    # object 50
    @property
    def voltage_setpoint(self):
        v = self._get_integer(50)
        return self._u_nom * v / 25600

    @voltage_setpoint.setter
    def voltage_setpoint(self, u):
        return self._set_integer(50, int(round((u * 25600.0) / self._u_nom)))

    # # object 50
    # def get_voltage_setpoint(self):
    #     v = self._get_integer(50)
    #     return self._u_nom * v / 25600

    # def set_voltage(self, u):
    #     return self._set_integer(50, int(round((u * 25600.0) / self._u_nom)))

    # object 51
    @property
    def current_setpoint(self):
        i = self._get_integer(50)
        return self._i_nom * i / 25600

    @current_setpoint.setter
    def current_setpoint(self, i):
        return self._set_integer(51, int(round((i * 25600.0) / self._i_nom)))

    # # object 51
    # def get_current_setpoint(self):
    #     i = self._get_integer(50)
    #     return self._i_nom * i / 25600

    # def set_current(self, i):
    #     return self._set_integer(51, int(round((i * 25600.0) / self._i_nom)))

    # object 54
    def _get_control(self):
        return self._get_binary(54)

    def _set_control(self, mask, data):
        ans = self._set_binary(54, mask, data)

        # return True if command was acknowledged ("error 0")
        return ans[0] == 0xff and ans[1] == 0x00

    def set_remote(self, remote=True):
        if remote:
            return self._set_control(0x10, 0x10)
        else:
            return self._set_control(0x10, 0x00)

    def set_local(self, local=True):
        return self.set_remote(not local)

    def set_output_on(self, on=True):
        if on:
            return self._set_control(0x01, 0x01)
        else:
            return self._set_control(0x01, 0x00)

    def set_output_off(self, off=True):
        return self.set_output_on(not off)

    # object 71
    def get_actual(self, print_state = False):
        ans = self._get_binary(71)

        actual = dict()
        actual['remote']   = True if ans[0] & 0x03 else False
        actual['local']    = not actual['remote']
        actual['on']       = True if ans[1] & 0x01 else False
        actual['CC']       = True if ans[1] & 0x06 else False
        actual['CV']       = not actual['CC']
    #	actual['tracking'] = True if ans[1] & 0x08 else False
        actual['OVP']      = True if ans[1] & 0x10 else False
        actual['OCP']      = True if ans[1] & 0x20 else False
        actual['OPP']      = True if ans[1] & 0x40 else False
        actual['OTP']      = True if ans[1] & 0x80 else False
        actual['v']        = self._u_nom * ((ans[2] << 8) + ans[3]) / 25600
        actual['i']        = self._i_nom * ((ans[4] << 8) + ans[5]) / 25600

        if print_state:
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
    #		if actual['tracking']:
    #			print('tracking on')
    #		else:
    #			print('tracking off')

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
