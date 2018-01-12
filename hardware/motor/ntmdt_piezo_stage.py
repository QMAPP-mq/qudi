# -*- coding: utf-8 -*-

"""
This file contains the hardware control for a NT-MDT piezo stage.

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

from collections import OrderedDict

from core.module import Base, ConfigOption
from interface.motor_interface import MotorInterface


class PiezoStagePI(Base, MotorInterface):

    """unstable: Matt van Breugel
    This is the hardware module for communicating with NT-MDT piezo scanning stages
    over USB (via the NovaSDK dll). It uses the VB script from the documentation.
    """
    _modclass = 'PiezoStageNTMDT'
    _modtype = 'hardware'

    _devID = ctypes.c_int()

    _double3d = ctypes.c_double * 3  # This is creating a 3D double array object
    _double1d = ctypes.c_double * 1  # This is creating a 1D double object
    _bool1d = ctypes.c_bool * 1  # This is creating a 1D bool object

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        @return: error code
        """
        path_dll = os.path.join(self.get_main_dir(),
                                'thirdparty',
                                'nt_mdt',
                                'NovaSDK_x64.dll'
                                )
        self._novadll = ctypes.windll.LoadLibrary(path_dll)

        # Find out what devices are connected
        emptybufferpy = ' ' * 1000
        charBuffer = ctypes.c_char_p(emptybufferpy.encode())
        bufSize = ctypes.c_int(1000)

        numofdevs = self._pidll.PI_EnumerateUSB(charBuffer, bufSize, ctypes.c_char_p(b''))

        # read the device list out of ctype charBuffer into a regular python list of strings
        device_list = charBuffer.value.decode().split('\n')

        # split list into elements, check for PI devices
        pi_devices = [device for device in device_list if 'PI' in device]

        if len(pi_devices) == 1:
            device_name = ctypes.c_char_p(pi_devices[0].encode())
            self._pidll.PI_ConnectUSB(device_name)
            self._devID = ctypes.c_int(0)

        elif len(pi_devices) > 1:
            self.log.warning('There is more than 1 PI device connected, I do not know which one to choose!')

        else:
            self.log.warning('I cannot find any connected devices with "PI" in their name.')

        if self._pidll.PI_IsConnected(self._devID) is False:
            return 1
        else:
            self._set_servo_state(True)
            return 0

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        @return: error code
        """
        self._set_servo_state(False)
        self._pidll.PI_RTO(self._devID, ctypes.c_char_p(''.encode()))
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

        # Move in x:
        newpos = self._double1d(param_dict['x'] * 1e6)
        ax = ctypes.c_char_p('1'.encode())
        self._pidll.PI_MOV(self._devID, ax, newpos)
        onT = self._bool1d(0)
        while not onT[0]:
            self._pidll.PI_qONT(self._devID, ax, onT)

        # Move in y:
        newpos = self._double1d(param_dict['y'] * 1e6)
        ax = ctypes.c_char_p('2'.encode())
        self._pidll.PI_MOV(self._devID, ax, newpos)
        onT = self._bool1d(0)
        while not onT[0]:
            self._pidll.PI_qONT(self._devID, ax, onT)

        # Move in z:
        newpos = self._double1d(param_dict['z'] * 1e6)
        ax = ctypes.c_char_p('3'.encode())
        self._pidll.PI_MOV(self._devID, ax, newpos)
        onT = self._bool1d(0)
        while not onT[0]:
            self._pidll.PI_qONT(self._devID, ax, onT)

        param_dict = self.get_pos()
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

        # Now we create an instance of that object
        posBuffer = self._double3d()
        axesBuffer = ctypes.c_char_p(''.encode())

        err = self._pidll.PI_qPOS(ctypes.c_int(0), axesBuffer, posBuffer)

        param_dict = {}
        param_dict['x'] = posBuffer[0] / 1e6
        param_dict['y'] = posBuffer[1] / 1e6
        param_dict['z'] = posBuffer[2] / 1e6

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

########################## internal methods ##################################

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
                             ''.format(axis, constraints[axis]['pos_min'], constraints[axis]['pos_max']))
        else:
            self._write_xyz(axis, 'MA{0}'.format(int(move * 1e7)))  # 1e7 to convert meter to SI units
            #self._write_xyz(axis, 'MP')
        return axis, move

    def _set_servo_state(self, to_state):
        """internal method enabling / disabling the servos

        @param bool to_state: desired state of the servos
        """

        servo_state = self._bool1d()

        axis_list = ['1', '2', '3']

        for axis in axis_list:
            axesBuffer = ctypes.c_char_p(str(axis).encode())

            self._pidll.PI_qSVO(self._devID, axesBuffer, servo_state)

            if (servo_state[0] is False) and (to_state is True):
                self._pidll.PI_SVO(self._devID, axis, self._bool1d(1))
            elif (servo_state[0] is True) and (to_state is False):
                self._pidll.PI_SVO(self._devID, axis, self._bool1d(0))

########################## Nova PX Communication ##################################

def _run_script_text(command):
    """ execute a command in Nova Px

    @param string command: VBScript code to be executed
    """
    self._novadll.RunScriptText(command.encode())

def _get_shared_float(variable):
    """ retreive a shared data variable of type float from Nova Px

    @param string variable: The variable must have already been created

    @returns float value: The value of variable
    """
    outbuf = ctypes.c_double()
    buflen = ctypes.c_int()

    _novadll.GetSharedData(variable.encode(), None, ctypes.byref(buflen))  # get the required buffer size
    _novadll.GetSharedData(variable.encode(), ctypes.byref(outbuf), ctypes.byref(buflen))  # fill the buffer

    return outbuf.value

#########################################################################################
