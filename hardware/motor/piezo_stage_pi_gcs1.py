# -*- coding: utf-8 -*-

"""
This file contains the hardware control for PI piezo stages running GCS.

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


class PiezoStagePI_GCS1(Base, MotorInterface):

    """unstable: Cyril Laplane, Matt van Breugel
    This is the hardware module for communicating with PI Piezo scanning stages
    over PCI (via the PI dll). It uses the General Command Set (GCS) from
    the PI documentation.

    This module has been developed for the E-761 Digital Piezo Controller,
    but probably works with any PI controller that talks GCS over PCI.

    Example configuration:
    ```
    # pi_piezo:
    #     module.Class: 'motor.piezo_stage_pi_pci_gcs.PiezoStagePI'
    #     axis_labels:
    #         - x
    #         - y
    #         - z
    #     x:
    #         constraints:
    #             pos_min: 0e-6
    #             pos_max: 300e-6
    #     y:
    #         constraints:
    #             pos_min: 0e-6
    #             pos_max: 300e-6
    #     z:
    #         constraints:
    #             pos_min: 0e-6
    #             pos_max: 300e-6
    ```
    """

    _modclass = 'PiezoStagePI_GCS1'
    _modtype = 'hardware'

    _devID = ctypes.c_int()
    _board_number = ConfigOption('board_number', 1, missing='warn')

    _double3d = ctypes.c_double * 3  # This is creating a 3D double array object
    _double1d = ctypes.c_double * 1  # This is creating a 1D double object
    _bool1d = ctypes.c_bool * 1  # This is creating a 1D bool object

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    # Open connection with the PCI board and get ID 
    def on_activate(self):
        """ Initialise and activate the hardware module.

            @return: error code (0:OK, -1:error)
        """

        path_dll = os.path.join(self.get_main_dir(),
                                'thirdparty',
                                'physik_instrumente',
                                'E7XX_GCS_DLL_x64.dll'
                                )
        self._pidll = ctypes.windll.LoadLibrary(path_dll)
        
        # Open a PCI connection to the E761 board (there is only one board so its number is 1)
        device_name = self._pidll.E7XX_ConnectPciBoard(self._board_number)
        # self._devID = ctypes.c_int(0)
        self._devID = device_name

        if device_name < 0:
            self.log.warning('Cant connect with the PCI board !!')
        else:
            if self._pidll.E7XX_IsConnected(self._devID) is False:
                return 1
            else:
                self._set_servo_state(True)
                self._configured_constraints = self.get_constraints()
                return 0

    def on_deactivate(self):
        """ Deinitialise and deactivate the hardware module.

            @return: error code (0:OK, -1:error)
        """

        self._set_servo_state(False)
        self._pidll.E7XX_CloseConnection(self._devID)
        return 0

    def get_constraints(self):
        """ Retrieve the hardware constrains from the motor device.

        Provides all the constraints for the xyz stage  and rot stage (like total
        movement, velocity, ...)
        Each constraint is a tuple of the form
            (min_value, max_value, stepsize)

            @return dict constraints : dict with constraints for the device
        """

        constraints = OrderedDict()

        config = self.getConfiguration()

        axis0 = {}
        axis0['label'] = 'x'
        axis0['pos_min'] = config['x']['constraints']['pos_min']
        axis0['pos_max'] = config['x']['constraints']['pos_max']

        axis1 = {}
        axis1['label'] = 'y'
        axis1['pos_min'] = config['y']['constraints']['pos_min']
        axis1['pos_max'] = config['y']['constraints']['pos_max']

        axis2 = {}
        axis2['label'] = 'z'
        axis2['pos_min'] = config['z']['constraints']['pos_min']
        axis2['pos_max'] = config['z']['constraints']['pos_max']

        # assign the parameter container for x to a name which will identify it
        constraints[axis0['label']] = axis0
        constraints[axis1['label']] = axis1
        constraints[axis2['label']] = axis2

        return constraints

    def move_rel(self, param_dict):
        """ Move stage relatively in given direction

            @param dict param_dict : dictionary, which passes all the relevant
                                     parameters, which should be changed. Usage:
                                     {'axis_label': <the-abs-pos-value>}.
                                     'axis_label' must correspond to a label given
                                     to one of the axis.


            @return dict param_dict : dictionary with the current magnet position
        """
        self.log.info('Function not yet implemented')

        return param_dict

    def move_abs(self, param_dict):
        """ Move the stage to an absolute position

        @param dict param_dict : dictionary, which passes all the relevant
                                 parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.
                                 The values for the axes are in meter,
                                 the value for the rotation is in degrees.

        @return dict param_dict : dictionary with the current axis position
        """

        invalid_axis = set(param_dict)-set(['x', 'y', 'z'])

        if invalid_axis:
            for axis in invalid_axis:      
                self.log.warning('Desired axis {axis} is undefined'
                                .format(axis=axis))

        for axis in ['x', 'y', 'z']:

            if axis in param_dict.keys():
                if axis == 'x':
                    to_position = param_dict['x']
                    self._do_move_abs(axis, to_position)
                elif axis == 'y':
                    to_position = param_dict['y']
                    self._do_move_abs(axis, to_position)
                elif axis == 'z':
                    to_position = param_dict['z']
                    self._do_move_abs(axis, to_position)

        param_dict = self.get_pos()
        return param_dict

    def abort(self):
        """ Stop movement of the stage

        @return int: error code (0:OK, -1:error)
        """
        self.log.info('Function not yet implemented')

        return 0

    def get_pos(self, param_list=None):
        """ Get the current position of the stage axis

        @param list param_list : optional, if a specific position of an axis
                                 is desired, then the labels of the needed
                                 axis should be passed in the param_list.
                                 If nothing is passed, then the positions of
                                 all axes are returned.

        @return dict param_dict : with keys being the axis labels and item the current
                                  position.
        """

        # Now we create an instance of that object
        posBuffer = self._double3d()
        axesBuffer = ctypes.c_char_p(''.encode())

        err = self._pidll.E7XX_qPOS(self._devID, axesBuffer, posBuffer)

        param_dict = {}
        param_dict['x'] = posBuffer[0] / 1e6  # unit conversion from communication
        param_dict['y'] = posBuffer[1] / 1e6  # unit conversion from communication
        param_dict['z'] = posBuffer[2] / 1e6  # unit conversion from communication

        return param_dict

    def get_status(self, param_list=None):
        """ Get the status of the position

        @param list param_list : optional, if a specific status of an axis
                                 is desired, then the labels of the needed
                                 axis should be passed in the param_list.
                                 If nothing is passed, then from each axis the
                                 status is asked.

        @return dict : with the axis label as key and the status number as item.
            The meaning of the return value is:
            Bit 0: Ready Bit 1: On target Bit 2: Reference drive active Bit 3: Joystick ON
            Bit 4: Macro running Bit 5: Motor OFF Bit 6: Brake ON Bit 7: Drive current active
        """
        self.log.info('Not yet implemented for this hardware')

    def calibrate(self, param_list=None):
        """ Calibrate the stage.

        @param dict param_list : param_list: optional, if a specific calibration
                                 of an axis is desired, then the labels of the
                                 needed axis should be passed in the param_list.
                                 If nothing is passed, then all connected axis
                                 will be calibrated.

        After calibration the stage moves to home position which will be the
        zero point for the passed axis.

        @return dict pos : dictionary with the current position of the ac#xis
        """
        self.log.info('Not yet implemented for this hardware')

        pos = {}

        return pos

    def get_velocity(self, param_list=None):
        """ Get the current velocity for all connected axes in m/s.

            @param list param_list : optional, if a specific velocity of an axis
                                     is desired, then the labels of the needed
                                     axis should be passed as the param_list.
                                     If nothing is passed, then from each axis the
                                     velocity is asked.

            @return dict : with the axis label as key and the velocity as item.
        """
        self.log.info('Function not yet implemented for this stage')

    def set_velocity(self, param_dict):
        """ Write new value for velocity in m/s.

        @param dict param_dict : dictionary, which passes all the relevant
                                 parameters, which should be changed. Usage:
                                 {'axis_label': <the-velocity-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        @return dict param_dict2 : dictionary with the updated axis velocity
        """
        self.log.info('Not yet implemented for this hardware')

########################## internal methods ##################################

    def _do_move_abs(self, axis, to_pos):
        """ Make absolute axis move in meters (internal method)

        @param axis string  : name of the axis that should be moved
               float to_pos : desired position in meters
        """

        if not(self._configured_constraints[axis]['pos_min'] <= to_pos <= self._configured_constraints[axis]['pos_max']):
            self.log.warning('Cannot make the movement of the axis "{axis}"'
                             'since the border [{min},{max}] would be crossed! Ignore command!'
                             ''.format(axis=axis, min=self._configured_constraints[axis]['pos_min'], max=self._configured_constraints[axis]['pos_max']))
        else:
            self._write_axis_move(axis, to_pos)

    def _write_axis_move(self, axis, to_pos):
        """ Move a specified axis (internal method)

            @param set axis     : name of the axis that should be moved
                   float to_pos : desired position in meters
        """

        newpos = self._double1d(to_pos * 1e6)  # unit conversion for communication
        ax = ctypes.c_char_p(axis.encode())
        self._pidll.E7XX_MOV(self._devID, ax, newpos)
        onT = self._bool1d(0)
        while not onT[0]:
            self._pidll.E7XX_qONT(self._devID, ax, onT)

    def _set_servo_state(self, to_state):
        """ Set the servo state (internal method)

        @param bool to_state : desired state of the servos
        """

        servo_state = self._bool1d()

        axis_list = ['x', 'y', 'z']

        for axis in axis_list:
            axesBuffer = ctypes.c_char_p(str(axis).encode())

            self._pidll.E7XX_qSVO(self._devID, axesBuffer, servo_state)

            if (servo_state[0] is False) and (to_state is True):
                self._pidll.E7XX_SVO(self._devID, axis, self._bool1d(1))
            elif (servo_state[0] is True) and (to_state is False):
                self._pidll.E7XX_SVO(self._devID, axis, self._bool1d(0))
