# -*- coding: utf-8 -*-

"""
This file contains the hardware control for a NT-MDT piezo stage.

N.B. NT-MDT Nova Px control software must be running.

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
import platform

from collections import OrderedDict

from core.module import Base, ConfigOption
from interface.motor_interface import MotorInterface


class PiezoStageNTMDT(Base, MotorInterface):

    """unstable: Matt van Breugel
    This is the hardware module for communicating with NT-MDT piezo scanning stages
    over USB (via the NovaSDK dll). It uses the VB script from the documentation.
    """
    _modclass = 'PiezoStageNTMDT'
    _modtype = 'hardware'

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        @return: error code
        """
        if platform.architecture()[0] == '64bit':
            path_dll = os.path.join(self.get_main_dir(),
                                    'thirdparty',
                                    'nt_mdt',
                                    'NovaSDK_x64.dll'
                                    )
        elif platform.architecture()[0] == '32bit':
            path_dll = os.path.join(self.get_main_dir(),
                                    'thirdparty',
                                    'nt_mdt',
                                    'NovaSDK.dll'
                                    )
        else:
            self.log.error('Unknown platform, cannot load the Nova SDK dll.')

        self._novadll = ctypes.windll.LoadLibrary(path_dll)

        if self._check_connection():
            self._set_servo_state(True)
            return 0
        else:
            self.log.error('I cannot connect to Nova Px.')
            return 1

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        @return: error code
        """
        self._set_servo_state(False)
        return 0

    def get_constraints(self):
        """ Retrieve the hardware constrains from the motor device.

        @return dict: dict with constraints for the sequence generation and GUI

        Provides all the constraints for the xyz stage  and rot stage (like total
        movement, velocity, ...)
        Each constraint is a tuple of the form
            (min_value, max_value, stepsize)
        """
        constraints = OrderedDict()

        axis_max_constraints = self._get_scanner_range()

        axis0 = {}
        axis0['label'] = 'x'
        axis0['pos_min'] = 0.0
        axis0['pos_max'] = axis_max_constraints['x']

        axis1 = {}
        axis1['label'] = 'y'
        axis1['pos_min'] = 0.0
        axis1['pos_max'] = axis_max_constraints['y']

        axis2 = {}
        axis2['label'] = 'z'
        axis2['pos_min'] = 0.0
        axis2['pos_max'] = axis_max_constraints['z']

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

        return param_dict

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


        command =   ('SetParam tBase, cValue, pi_ScrPosX, 0, {xpos}\n\n'
                     'SetParam tBase, cValue, pi_ScrPosX, 0, {ypos}\n\n'
                     'SetParam tBase, cValue, pi_ScrPosX, 0, {zpos}\n\n'
                     'Do\n\n'
                     '\tIdle\n\n'
                     'Loop Until GetParam(tScanner, cStatus, 0) = 0'
                     .format(xpos=param_dict['x'], ypos=param_dict['x'], zpos=param_dict['x']))

        self._run_script_text(command)
        param_dict = self.get_pos()
        self._update_gui()
        return param_dict

    def abort(self):
        """Stops movement of the stage

        @return int: error code (0:OK, -1:error)
        """
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
        param_dict = {}

        for axis in ['x', 'y', 'z']:
            command =   ('{axis}Pos = GetParam(tBase, cValue, pi_ScrPos{axis}, 0, False)\n\n'
                        'SetSharedDataVal "shared{axis}Pos", {axis}Pos, "F64", 8'
                        .format(axis=axis.upper()))

            self._run_script_text(command)
            param_dict[axis] = self._get_shared_float('shared{axis}Pos'.format(axis=axis.upper()))  # TODO check returned units

        if param_list:
            param_list = [x.lower() for x in param_list]  # make all param_list elements lower case
            for axis in list(set(param_dict.keys()) - set(param_list)):  # axes not in param_list
                del param_dict[axis]
            return param_dict
        else:
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

    def calibrate(self, param_list=None):
        """ Calibrates the stage.

        @param dict param_list: param_list: optional, if a specific calibration
                                of an axis is desired, then the labels of the
                                needed axis should be passed in the param_list.
                                If nothing is passed, then all connected axis
                                will be calibrated.

        After calibration the stage moves to home position which will be the
        zero point for the passed axis.

        @return dict pos: dictionary with the current position of the ac#xis
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

########################## internal methods ####################################

    def _do_move_abs(self, axis, move):
        """internal method for the absolute move in meter

        @param axis string: name of the axis that should be moved

        @param float move: desired position in meter

        @return str axis: axis which is moved
                move float: absolute position to move to
        """
        constraints = self.get_constraints()
        #self.log.info(axis + 'MA{0}'.format(int(move*1e8)))
        if not(constraints[axis]['pos_min'] <= move <= constraints[axis]['pos_max']):
            self.log.warning('Cannot make the movement of the axis "{0}"'
                             'since the border [{1},{2}] would be crossed! Ignore command!'
                             .format(axis, constraints[axis]['pos_min'], constraints[axis]['pos_max']))
        else:
            self._write_xyz(axis, 'MA{0}'.format(int(move * 1e7)))  # 1e7 to convert meter to SI units
            #self._write_xyz(axis, 'MP')
        return axis, move

    def _set_servo_state(self, to_state):
        """internal method enabling / disabling the stage feedback

        @param bool to_state: desired state of the feedback servos
        """
        # TODO this is unlikely to be the correct command
        for axis in ['x', 'y', 'z']:
            command =   ('SetParam tBase, cValue, pi_Scr{axis}}FBState, 0, {to_state}'
                        .format(axis=axis.upper(), to_state=int(to_state)))  # bool to int
        self._update_gui()

    def _get_scanner_range(self):
        """ get the range of movement of the scanner

        @returns dict axis_max_constraints: contains maximum positional values
        """
        axis_max_constraints = {}

        for axis in ['x', 'y', 'z']:
            command =   ('Set ParInfo = GetParam(tBase, cInfo, pi_ScrPos{axis}, 0)\n\n'
                        'Val{axis} = ParInfo.MaxValue\n\n'
                        'SetSharedDataVal "shared{}PosMax", Val{axis}, "F64", 8'
                        .format(axis=axis.upper()))

            self._run_script_text(command)
            axis_max_constraints[axis] = self._get_shared_float('shared{axis}PosMax'.format(axis=axis.upper()))  # TODO check returned units

        return axis_max_constraints


########################## Nova PX Communication ###############################

    def _run_script_text(self, command):
        """ execute a command in Nova Px

        @param string command: VBScript code to be executed
        """
        self._novadll.RunScriptText(command.encode())

    def _get_shared_float(self, variable):
        """ retreive a shared data variable of type float from Nova Px

        @param string variable: The variable must have already been created

        @returns float value: The value of variable
        """
        outbuf = ctypes.c_double()
        buflen = ctypes.c_int()

        self._novadll.GetSharedData(variable.encode(), None, ctypes.byref(buflen))  # get the required buffer size
        self._novadll.GetSharedData(variable.encode(), ctypes.byref(outbuf), ctypes.byref(buflen))  # fill the buffer

        return outbuf.value

    def _update_gui(self):
        """ update the Nova Px graphical user unterface

        this operation is noted to be "not threadsafe" in the original documentation
        """
        command = 'Perform tGlobal, gGuiUpdate'
        self._run_script_text(command)

    def _check_connection(self):
        """ set and get a shared variable to check the connection with Nova Px

        @returns bool success: True if values match
        """
        command = 'SetSharedDataVal "test_connection", 1.61803398875, "F64", 8'
        self._run_script_text(command)
        return self._get_shared_float('test_connection') == 1.61803398875

################################################################################
