# -*- coding: utf-8 -*-
"""
Interface file for lab powersupplies.

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

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

from enum import Enum
import abc
from core.util.interfaces import InterfaceMetaclass


class PowersupplyInterface(metaclass=InterfaceMetaclass):
    _modtype = 'PowersupplyInterface'
    _modclass = 'interface'

    # object 2
    def get_nominal_voltage(self):
        return self._get_float(2)

    # object 3
    def get_nominal_current(self):
        return self._get_float(3)

    # object 4
    def get_nominal_power(self):
        return self._get_float(4)

    # object 38
    def get_OVP_threshold(self):
        return self._get_integer(38)

    def set_OVP_threshold(self, u):
        return self._set_integer(38, u)

    # object 39
    def get_OCP_threshold(self):
        return self._get_integer(39)

    def set_OCP_threshold(self, i):
        return self._set_integer(39, i)

    # object 50
    def get_voltage_setpoint(self):
        v = self._get_integer(50)
        return self.u_nom * v / 25600

    def set_voltage(self, u):
        return self._set_integer(50, int(round((u * 25600.0) / self.u_nom)))

    # object 51
    def get_current_setpoint(self):
        i = self._get_integer(50)
        return self.i_nom * i / 25600

    def set_current(self, i):
        return self._set_integer(51, int(round((i * 25600.0) / self.i_nom)))

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
        actual['v']        = self.u_nom * ((ans[2] << 8) + ans[3]) / 25600
        actual['i']        = self.i_nom * ((ans[4] << 8) + ans[5]) / 25600

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
    