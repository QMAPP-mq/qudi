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

This module is inspired by the work of Robert TODO

You should have received a copy of the GNU General Public License
along with Qudi. If not, see <http://www.gnu.org/licenses/>.

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import time
import usb.core
import usb.util

from collections import OrderedDict

from core.module import Base, ConfigOption
from interface.motor_interface import MotorInterface


class PiezoScrewsNF(Base, MotorInterface):

    """unstable: Matt van Breugel, Lachlan Rogers
    This is the hardware module for communicating with New Focus piezo screws.

    This module has been developed for the New Focus picomotor controller model 8742
    but probably works with any New Focus controller with a comparible command set.
    """
    _modclass = 'PiezoScrewsNF'
    _modtype = 'hardware'

    eol_write = b"\r"
    eol_read = b"\r\n"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        @return: error code
        """

        # TODO: get these from config
        self.vendor_id = 0x104d
        self.product_id = 0x4000

        # find our device
        self.dev = usb.core.find(idVendor=self.vendor_id, idProduct=self.product_id)

        # was it found?
        if self.dev is None:
            raise ValueError('Device not found')
            return 1

        # set the active configuration. With no arguments, the first
        # configuration will be the active one
        self.dev.set_configuration()

        # get an endpoint instance
        cfg = self.dev.get_active_configuration()
        intf = cfg[(0,0)]

        self.ep_out = usb.util.find_descriptor(
            intf,
            # match the first OUT endpoint
            custom_match = 
            lambda e: 
                usb.util.endpoint_direction(e.bEndpointAddress) == 
                usb.util.ENDPOINT_OUT)

        assert self.ep_out is not None
        assert self.ep_out.wMaxPacketSize == 64

        self.ep_in = usb.util.find_descriptor(
            intf,
            # match the first OUT endpoint
            custom_match = 
            lambda e: 
                usb.util.endpoint_direction(e.bEndpointAddress) == 
                usb.util.ENDPOINT_IN)

        assert self.ep_in is not None
        assert self.ep_in.wMaxPacketSize == 64

        return 0


    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        @return: error code
        """

        for i in range(1, 5):  # 1, 2, 3, 4
            self._stop(i)

        self._abort()

        # self.get_pos({'x','y','z'})

        usb.util.dispose_resources(self.dev)

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

        for axis_label in param_dict:
            if 'x' in axis_label:
                axis_numbers.append(1)
            if 'y' in axis_label:
                axis_numbers.append(2)
            if 'z' in axis_label:
                axis_numbers.append(3)

        for axis in axis_numbers:
            if axis == 1:
                self._move_rel_axis(axis, param_dict['x'])
            elif axis == 2:
                self._move_rel_axis(axis, param_dict['y'])
            elif axis == 3:
                self._move_rel_axis(axis, param_dict['z'])

        return self.get_pos()

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

        for axis_label in param_dict:
            if 'x' in axis_label:
                axis_numbers.append(1)
            if 'y' in axis_label:
                axis_numbers.append(2)
            if 'z' in axis_label:
                axis_numbers.append(3)

        for axis in axis_numbers:
            if axis == 1:
                self._move_abs_axis(axis, param_dict['x'])
            elif axis == 2:
                self._move_abs_axis(axis, param_dict['y'])
            elif axis == 3:
                self._move_abs_axis(axis, param_dict['z'])

        return self.get_pos()

    def abort(self):
        """Stops movement of the stage with no deceleration

        @return int: error code (0:OK, -1:error)
        """
        self._abort()
        return 0

    def get_pos(self, param_dict = None):
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
        pos_dict = {}

        if param_dict is not None:
            for axis_label in param_dict:
                if 'x' in axis_label:
                    axis_numbers.append(1)
                if 'y' in axis_label:
                    axis_numbers.append(2)
                if 'z' in axis_label:
                    axis_numbers.append(3)
        else:
            axis_numbers = [1, 2, 3]

        for axis in axis_numbers:
            if axis == 1:
                value = int(self._ask('MD?', xx= axis))
                # value = self._on_target(axis)
                while value == False:
                    time.sleep(0.0005)
                    value = int(self._ask('MD?', xx= axis))
                    # value = self._on_target(axis)
                pos_dict['x'] = self._get_pos_axis(axis)
            elif axis == 2:
                value = int(self._ask('MD?', xx= axis))
                # value = self._on_target(axis)
                while value == False:
                    time.sleep(0.0005)
                    value = int(self._ask('MD?', xx= axis))
                    # value = self._on_target(axis)
                pos_dict['y'] = self._get_pos_axis(axis)
            elif axis == 3:
                value = int(self._ask('MD?', xx= axis))
                # value = self._on_target(axis)
                while value == False:
                    time.sleep(0.0005)
                    value = int(self._ask('MD?', xx= axis))
                    # value = self._on_target(axis)
                pos_dict['z'] = self._get_pos_axis(axis)

        return pos_dict

    def get_status(self, param_dict):
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

        axis_numbers = []
        status_dict = {}

        if param_dict is not None:
            for axis_label in param_dict:
                if 'x' in axis_label:
                    axis_numbers.append(1)
                if 'y' in axis_label:
                    axis_numbers.append(2)
                if 'z' in axis_label:
                    axis_numbers.append(3)
        else:
            axis_numbers = [1, 2, 3]

        for axis in axis_numbers:
            if axis == 1:
                status_dict['x'] = self._done(axis)
            elif axis == 2:
                status_dict['y'] = self._done(axis)
            elif axis == 3:
                status_dict['z'] = self._done(axis)

        return status_dict

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

    def get_velocity(self, param_dict):
        """ Gets the current velocity for all connected axes in m/s.

        @param list param_list: optional, if a specific velocity of an axis
                                    is desired, then the labels of the needed
                                    axis should be passed as the param_list.
                                    If nothing is passed, then from each axis the
                                    velocity is asked.

        @return dict : with the axis label as key and the velocity as item.
        """
        axis_numbers = []
        velocity_dict = {}

        if param_dict is not None:
            for axis_label in param_dict:
                if 'x' in axis_label:
                    axis_numbers.append(1)
                if 'y' in axis_label:
                    axis_numbers.append(2)
                if 'z' in axis_label:
                    axis_numbers.append(3)
        else:
            axis_numbers = [1, 2, 3]

        for axis in axis_numbers:
            if axis == 1:
                velocity_dict['x'] = self._ask_velocity(axis)
            elif axis == 2:
                velocity_dict['y'] = self._ask_velocity(axis)
            elif axis == 3:
                velocity_dict['z'] = self._ask_velocity(axis)

        return velocity_dict

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

        for axis_label in param_dict:
            if 'x' in axis_label:
                axis_numbers.append(1)
            if 'y' in axis_label:
                axis_numbers.append(2)
            if 'z' in axis_label:
                axis_numbers.append(3)

        for axis in axis_numbers:
            if axis == 0:
                param_dict['x'] = self._set_velocity_axis(axis, param_dict['x'])
            elif axis == 1:
                param_dict['y'] = self._set_velocity_axis(axis, param_dict['y'])
            elif axis == 2:
                param_dict['z'] = self._set_velocity_axis(axis, param_dict['z'])

        return self.get_velocity()

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

    def _writeline(self, cmd):
        """
        """
        self.ep_out.write(cmd.encode() + self.eol_write)
        
    def _readline(self):
        """
        """
        r = self.ep_in.read(64).tobytes()
        assert r.endswith(self.eol_read)
        r = r[:-2].decode()
        return r

    def _fmt_cmd(self, cmd, xx=None, *nn):
        """ Format a command.

        Args:
            cmd (str): few-letter command
            xx (int, optional for some commands): Motor channel
            nn (multiple int, optional): additional parameters
        """
        if xx is not None:
            cmd = "{:d}".format(xx) + cmd
        if nn:
            cmd += ", ".join("{:d}".format(n) for n in nn)
        return cmd

    def _ask(self, cmd, xx=None, *nn):
        """
        """
        cmd = self._fmt_cmd(cmd, xx, *nn)
        self._writeline(cmd)
        time.sleep(0.1)
        return self._readline()

    def _do(self, cmd, xx=None, *nn):
        """ Format and send a command to the device

        See Also:
            :meth:`fmt_cmd`: for the formatting and additional
                parameters.
        """
        cmd = self._fmt_cmd(cmd, xx, *nn)
        assert len(cmd) < 64
        self._writeline(cmd)

    def _on_target(self, axis):
        """
        """
        # return bool(self._ask('{}MD?'.format(axis)))
        return bool(self._ask('MD?', axis))

    def _move_rel_axis(self, axis, distance):
        """
        """
        # TODO can we convert distance to steps
        steps = distance
        self._do('PR', axis, steps)

        # while not self._on_target(axis):
        #     time.sleep(0.1)
        # time.sleep(1)

        # value = int(self._ask('MD?', xx= axis))
        value = self._on_target(axis)
        while value == False:
            time.sleep(0.0005)
            # value = int(self._ask('MD?', xx= axis))
            value = self._on_target(axis)
    
    def _move_abs_axis(self, axis, distance):
        """
        """
        # TODO can we convert distance to steps
        steps = distance
        self._do('PA', axis, steps)

        # while not self._on_target():
        #     time.sleep(0.1)
        # time.sleep(0.5)

        value = int(self._ask('MD?', xx= axis))
        # value = self._on_target(axis)
        while value == 0:
            time.sleep(0.0005)
            value = int(self._ask('MD?', xx= axis))
            # value = self._on_target(axis)
        
    def _get_pos_axis(self, axis):
        """
        """
        # target_position = self._ask('PA?',axis)
        # relative_position = self._ask('PR?',axis)
        actual_position = self._ask('TP?',axis)
        # print ('Get destination position (abs)', axis, target_position)  # debugging
        # print ('Get destination position (rel)', axis, relative_position)  # debugging
        # print ('Get position', axis, actual_position)  # debugging
        return actual_position
        log_file = open("hardware/motor/newfocusdatalog.txt", "r+")
        log_file.write(pos_dict)
        log_file.close()

    def _set_velocity_axis(self, axis, velocity):
        """
        """
        self._do('VA', axis, velocity)
   
    def _abort(self): #created by Jarrod to abort movement instantly no deceleration
        """
        """
        self._do('AB')

    def _stop(self, axis):#created by Jarrod to stop movement with deceleration
        """
        """
        self._do('ST', xx=axis)

    def _done(self, axis): #created by Jarrod to get the status of the motor
        """
        """
        # value = int(self._ask('MD?', xx=axis))
        value = self._on_target(axis)
        #cycle = 0
        #while cycle < 1:
        if value == False:
            # m = 'Moving'
            # print (m)
            return 2
        #    cycle = cycle + value
        else:
            # d = 'Done'
            # print (d)
            return 0

    def _ask_velocity(self, axis): #created by Jarrod to get the velocity the motor will move at
        """
        """
        v = self._ask('VA?', axis)
        # print (v)
        return v

    def _set_home(self, axis, position): #created by Jarrod to set the home position to values entered.
        """
        """
        self._do('DH', axis, position)
    
    def _check_home(self):
        """
        """
        # read_log =  open("hardware/motor/newfocusdatalog.txt", "r+")
        # home_dict = eval(read_log.read())
        # read_log.close

        home_dict = self.get_pos({'x','y','z'})
        home_dict['x'] = 0 - int(home_dict['x'])
        home_dict['y'] = 0 - int(home_dict['y'])
        home_dict['z'] = 0 - int(home_dict['z'])
        return home_dict

    def _print_to_log(self):
        """
        """
        log_file = open("hardware/motor/newfocusdatalog.txt", "r+")
        log_file.write(str(self.get_pos({'x','y','z'})))
        log_file.close()

