# -*- coding: utf-8 -*-

"""
This file contains the hardware control for New Focus piezo screws.

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

import time
import ctypes
import os

# TODO: ensure these are part of the qudi environment
import logging
import asyncio
import timeit

from collections import OrderedDict

from core.module import Base, ConfigOption
from interface.motor_interface import MotorInterface

# thirdparty code
from hardware.motor.newfocus8742.newfocus8742.usb import NewFocus8742USB as USB
from hardware.motor.newfocus8742.newfocus8742.tcp import NewFocus8742TCP as TCP
from hardware.motor.newfocus8742.newfocus8742.sim import NewFocus8742Sim as Sim


class PiezoScrewsNF(Base, MotorInterface):

    """unstable: Matt van Breugel, Lachlan Rogers
    This is the hardware module for communicating with New Focus piezo screws.

    This module has been developed for the New Focus picomotor controller model 8742
    but probably works with any New Focus controller with a comparible command set.
    """
    _modclass = 'PiezoScrewsNF'
    _modtype = 'hardware'

    _communication_method = ConfigOption('communication_over', missing='error')

    logging.basicConfig(level=logging.INFO)
    loop = asyncio.get_event_loop()
    loop.set_debug(False)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        @return: error code
        """
        self.dev = self._startup()

        if self.dev == 1:
            return 1

        self.log.info(self._error_message())

        self._print_config()
        self._set_initial_config()
        self._print_config()

        # TODO: check for more than 1 device connected

        return 0

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        @return: error code
        """
        self.dev.stop()
        self.dev.abort()
        return 0

    def get_constraints(self):
        """ Retrieve the hardware constrains from the motor device.

        @return dict: dict with constraints for the sequence generation and GUI

        Provides all the constraints for the xyz stage  and rot stage (like total
        movement, velocity, ...)
        Each constraint is a tuple of the form
            (min_value, max_value, stepsize)
        """
        # TODO: read this from config

        constraints = OrderedDict()

        axis0 = {}
        axis0['label'] = 'x'
        axis0['pos_min'] = 0.0
        axis0['pos_max'] = 300.0

        axis1 = {}
        axis1['label'] = 'y'
        axis1['pos_min'] = 0.0
        axis1['pos_max'] = 300.0

        axis2 = {}
        axis2['label'] = 'z'
        axis2['pos_min'] = 0.0
        axis2['pos_max'] = 300.0

        # assign the parameter container for x to a name which will identify it
        constraints[axis0['label']] = axis0
        constraints[axis1['label']] = axis1
        constraints[axis2['label']] = axis2

        return constraints

    def move_rel(self, param_dict):
        """Moves stage in given direction (relative movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.


        @return dict pos: dictionary with the current magnet position
        """

         # TODO: there must be a better way to do this

        axis_numbers = []

        for axis_label in param_list:
            if 'x' in axis_label:
                axis_numbers.append(0)
            if 'y' in axis_label:
                axis_numbers.append(1)
            if 'z' in axis_label:
                axis_numbers.append(2)

        for axis in axis_numbers:
            if axis == 0:
                param_dict['x'] = self.dev.set_relative(axis, param_dict['x'])
                # await dev.finish(axis)
                self._finish(axis)
            elif axis == 1:
                param_dict['y'] = self.dev.set_relative(axis, param_dict['y'])
                # await dev.finish(axis)
                self._finish(axis)
            elif axis == 2:
                param_dict['z'] = self.dev.set_relative(axis, param_dict['z'])
                # await dev.finish(axis)
                self._finish(axis)

        return self.get_position()

    def move_abs(self, param_dict):
        """Moves stage to absolute position

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.
                                The values for the axes are in meter,
                                the value for the rotation is in degrees.

        @return dict pos: dictionary with the current axis position
        """

        # TODO: there must be a better way to do this

        axis_numbers = []

        for axis_label in param_list:
            if 'x' in axis_label:
                axis_numbers.append(0)
            if 'y' in axis_label:
                axis_numbers.append(1)
            if 'z' in axis_label:
                axis_numbers.append(2)

        for axis in axis_numbers:
            if axis == 0:
                param_dict['x'] = self.dev.set_position(axis, param_dict['x'])
                # await dev.finish(axis)
                self._finish(axis)
            elif axis == 1:
                param_dict['y'] = self.dev.set_position(axis, param_dict['y'])
                # await dev.finish(axis)
                self._finish(axis)
            elif axis == 2:
                param_dict['z'] = self.dev.set_position(axis, param_dict['z'])
                # await dev.finish(axis)
                self._finish(axis)

        return self.get_position()

    def abort(self):
        """Stops movement of the stage

        @return int: error code (0:OK, -1:error)
        """
        self.dev.stop()
        self.dev.abort()
        return 0

    def get_pos(self, param_list=None):
        """ Gets current position of the stage arms

        @param list param_list: optional, if a specific position of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then the positions of
                                all axes are returned.

        @return dict: with keys being the axis labels and item the current
                      position.
        """
        # TODO: there must be a better way to do this

        axis_numbers = []

        if param_list:
            for axis_label in param_list:
                if 'x' in axis_label:
                    axis_numbers.append(0)
                if 'y' in axis_label:
                    axis_numbers.append(1)
                if 'z' in axis_label:
                    axis_numbers.append(2)
        else:
            axis_numbers.append(0)
            axis_numbers.append(1)
            axis_numbers.append(2)

        param_dict = {}

        for axis in axis_numbers:
            if axis == 0:
                # param_dict['x'] = self.dev.get_possition(axis)
                param_dict['x'] = self.dev.position(axis)
            elif axis == 1:
                # param_dict['y'] = self.dev.get_possition(axis)
                param_dict['x'] = self.dev.position(axis)
            elif axis == 2:
                # param_dict['z'] = self.dev.get_possition(axis)
                param_dict['x'] = self.dev.position(axis)

        return param_dict

    def get_status(self, param_list=None):
        """ Get the status of the position

        @param list param_list: optional, if a specific status of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                status is asked.

        @return dict: with the axis label as key and the status number as item.
        The meaning of the return value is:
        Bit 0: Ready Bit 1: On target Bit 2: Reference drive active Bit 3: Joystick ON
        Bit 4: Macro running Bit 5: Motor OFF Bit 6: Brake ON Bit 7: Drive current active
        """
        self._print_config()

    def calibrate(self, param_list=None):
        """ Calibrates the stage.

        @param dict param_list: param_list: optional, if a specific calibration
                                of an axis is desired, then the labels of the
                                needed axis should be passed in the param_list.
                                If nothing is passed, then all connected axis
                                will be calibrated.

        After calibration the stage moves to home position which will be the
        zero point for the passed axis.

        @return dict pos: dictionary with the current position of the axis
        """
        pos = {}

        return pos

    def get_velocity(self, param_list=None):
        """ Gets the current velocity for all connected axes in m/s.

        @param list param_list: optional, if a specific velocity of an axis
                                    is desired, then the labels of the needed
                                    axis should be passed as the param_list.
                                    If nothing is passed, then from each axis the
                                    velocity is asked.

        @return dict : with the axis label as key and the velocity as item.
        """

    def set_velocity(self, param_dict):
        """ Write new value for velocity in m/s.

        @param dict param_dict: dictionary, which passes all the relevant
                                    parameters, which should be changed. Usage:
                                     {'axis_label': <the-velocity-value>}.
                                     'axis_label' must correspond to a label given
                                     to one of the axis.

        @return dict param_dict2: dictionary with the updated axis velocity
        """

        # TODO: there must be a better way to do this

        axis_numbers = []

        for axis_label in param_list:
            if 'x' in axis_label:
                axis_numbers.append(0)
            if 'y' in axis_label:
                axis_numbers.append(1)
            if 'z' in axis_label:
                axis_numbers.append(2)

        for axis in axis_numbers:
            if axis == 0:
                param_dict['x'] = self.dev.set_velocity(axis, param_dict['x'])
            elif axis == 1:
                param_dict['y'] = self.dev.set_velocity(axis, param_dict['y'])
            elif axis == 2:
                param_dict['z'] = self.dev.set_velocity(axis, param_dict['z'])

########################## internal methods ##################################

    def _do_move_abs(self, axis, move):
        """internal method for the absolute move in meter

        @param axis string: name of the axis that should be moved

        @param float move: desired position in meter

        @return str axis: axis which is moved
                move float: absolute position to move to
        """
        # TODO: implement this
        constraints = self.get_constraints()
        #self.log.info(axis + 'MA{0}'.format(int(move*1e8)))
        if not(constraints[axis]['pos_min'] <= move <= constraints[axis]['pos_max']):
            self.log.warning('Cannot make the movement of the axis "{0}"'
                             'since the border [{1},{2}] would be crossed! Ignore command!'
                             ''.format(axis, constraints[axis]['pos_min'], constraints[axis]['pos_max']))
        else:
            self._write_xyz(axis, 'MA{0}'.format(int(move * 1e7)))  # 1e7 to convert meter to SI units
            #self._write_xyz(axis, 'MP')
        return axis, move

    def _set_servo_state(self, to_state):
        """internal method enabling / disabling the servos

        @param bool to_state: desired state of the servos
        """
        # this is an open-loop device
        return 1

########################## extra internal methods ###############################################################

    async def _print_config(self):
        """ internal method to print current stage config

        @param instance dev: the connected device
        """
        print(await self.dev.error_message())
        print(await self.dev.get_velocity(1))
        print(await self.dev.get_velocity(1))
        print(await self.dev.error_code())
        print(await self.dev.identify())
        print(await self.dev.get_home(1))
        print(await self.dev.get_position(1))
        print(await self.dev.get_relative(1))

    async def _set_initial_config(self):
        """ set the initial device configuration, values from newfocus8742

        @param instance dev: the connected device
        """
        self.set_velocity({'x':2000, 'y':2000})

        # audably know if the stage is connected
        self.move_rel({'x':10})
        await dev.finish(1)
        self.move_rel({'x':-10})
        await dev.finish(1)

    async def _startup(self):
        """
        """
        if not self._communication_method:
            self.log.warning('No communication method specified in the config, I cannot continue.')
            return 1
        
        if 'usb' in _communication_method.lower():
            dev = await USB.connect()
            return dev
        elif 'tcp' in _communication_method.lower():
            _tcp_addy = ConfigOption('tcp_addy', missing='error')
            if _tcp_addy:
                dev = await TCP.connect(_tcp_addy)
                return dev
            else:
                self.log.error('No TCP address specified in the config, I cannot contnue.')
                return 1
        elif 'sim' in _communication_method.lower():
            dev = await Sim.connect()
            return dev
        else:
            self.log.error('Invalid communication method specified in config.')
            return 1

    async def _finish(self, axis):
        """
        """
        await self.dev.finish(axis)

    async def _error_message(self):
        """
        """
        return await self.dev.error_message()

########################## required start-up functions from newfocus8742 ###############################################################

    async def k(self):
        for i in range(100):
            await self.dev.error_code()

    async def dump(self):
        for i in range(4):
            for cmd in "AC DH MD PA PR QM TP VA".split():
                print(1 + i, cmd, await self.dev.ask(cmd + "?", 1 + i))
        for cmd in ("SA SC SD TB TE VE ZZ "
                    "GATEWAY HOSTNAME IPADDR IPMODE MACADDR NETMASK "
                    ).split():
            print(cmd, await self.dev.ask(cmd + "?"))

    # unused
    async def test(self):
        print(dev)
        m = 2
        self.dev.do("VA", m, 2000)
        self.dev.do("AC", m, 100000)
        for i in range(100):
            self.dev.do("PR", m, 100)
            while not int(await dev.ask("MD?", m)):
                await asyncio.sleep(.001)
            print(".")
            await asyncio.sleep(.1)
        print(await self.dev.ask("TP?", m))
        print(await self.dev.ask("QM?", m))
