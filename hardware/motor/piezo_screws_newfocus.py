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

    Example configuration:
    ```
    # nf_screws:
    #     module.Class: 'motor.piezo_screws_newfocus.PiezoScrewsNF'
    #     vendorID: 0x104d
    #     productID: 0x4000
    #     axis_labels:
    #         - x
    #         - y
    #     x:
    #         channel: 0
    #         constraints:
    #             pos_min: 0
    #             pos_max: 26e-3
    #     y:
    #         channel: 1
    #         constraints:
    #             pos_min: 0
    #             pos_max: 26e-3
    ```
    """

    _modclass = 'PiezoScrewsNF'
    _modtype = 'hardware'

    eol_write = b"\r"
    eol_read = b"\r\n"

    # # x_axis_channel = 1
    # # y_axis_channel = 2
    # z_axis_channel = 3
    # # x_axis_min = 0
    # # x_axis_max = 26e-3
    # # y_axis_min = 0
    # # y_axis_max = 26e-3
    # z_axis_min = 0
    # z_axis_max = 26e-3

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def on_activate(self):
        """ Initialisae the hardware module.

            @return int error code (0:OK, -1:error)
        """

        # self.vendor_id = ConfigOption('vendorID', 0x104d, missing='warn')
        # self.product_id = ConfigOption('productID' ,0x4000, missing='warn')

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

        # # get the config for this device. #need to figure out how to call nested config
        # config = self.getConfiguration()

        # for axis_label in config['axis_labels']:
        #     if axis_label == 'x':
        #         self.x_axis_channel = config[axis_label]['channel']
        #         self.x_axis_min = config[axis_label]['constraints']['pos_min']
        #         self.x_axis_max = config[axis_label]['constraints']['pos_max']
        #     if axis_label == 'y':
        #         self.y_axis_channel = config[axis_label]['channel']
        #         self.y_axis_min = config[axis_label]['constraints']['pos_min']
        #         self.y_axis_max = config[axis_label]['constraints']['pos_max']
        #     if axis_label == 'z':
        #         self.z_axis_channel = config[axis_label]['channel']
        #         self.z_axis_min = config[axis_label]['constraints']['pos_min']
        #         self.z_axis_max = config[axis_label]['constraints']['pos_max']

        self._configured_constraints = self.get_constraints()

        # self._go_to_original_home()  # may reimplement later

        return 0

    def on_deactivate(self):
        """ Deactivate of the hardware module.
        
            @return int error code (0:OK, -1:error)
        """

        for i in range(1, 5):  # 1, 2, 3, 4
            self._stop(i)

        self._abort()

        # self.get_pos({'x','y','z'})

        usb.util.dispose_resources(self.dev)

        return 0

    def get_constraints(self):
        """ Retrieve the hardware constrains from the motor device.

            Provides all the constraints for the xyz stage  and rot stage (like total
            movement, velocity, ...)
            
            Each constraint is a tuple of the form
                (min_value, max_value, stepsize)

            @return dict constraints : dict with constraints for the screws
        """

        constraints = OrderedDict()

        config = self.getConfiguration()

        constraints['axis_labels'] = config['axis_labels']

        axis0 = {}
        axis0['label'] = 'x'
        axis0['channel'] = config['x']['channel']
        axis0['pos_min'] = config['x']['constraints']['pos_min']
        axis0['pos_max'] = config['x']['constraints']['pos_max']

        if 'y' in config['axis_labels']:
            axis1 = {}
            axis1['label'] = 'y'
            axis1['channel'] = config['y']['channel']
            axis1['pos_min'] = config['y']['constraints']['pos_min']
            axis1['pos_max'] = config['y']['constraints']['pos_max']

        if 'z' in config['axis_labels']:
            axis2 = {}
            axis2['label'] = 'z'
            axis1['channel'] = config['z']['channel']
            axis1['pos_min'] = config['z']['constraints']['pos_min']
            axis1['pos_max'] = config['z']['constraints']['pos_max']

        # assign the parameter container for x to a name which will identify it
        constraints[axis0['label']] = axis0

        if 'y' in config['axis_labels']:
            constraints[axis1['label']] = axis1
        
        if 'z' in config['axis_labels']:
            constraints[axis2['label']] = axis2

        return constraints

    def move_rel(self, param_dict):
        """ Move the stage in given direction (relative movement)

        TODO: currently in steps, but should be in distance

        @param dict param_dict : dictionary, which passes all the relevant
                                 parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.


        @return dict : dictionary with the current axis positions
        """

        invalid_axis = set(param_dict)-set(self._configured_constraints['axis_labels'])

        if invalid_axis:
            for axis in invalid_axis:      
                self.log.warning('Desired axis {axis} is undefined'
                                .format(axis=axis))
                param_dict.remove(axis)

        # TODO: there must be a better way to do this

        axis_numbers = []

        for axis_label in param_dict:
            if 'x' in axis_label:
                axis_numbers.append(self.x_axis_channel)
            if 'y' in axis_label:
                axis_numbers.append(self.y_axis_channel)
            if 'z' in axis_label:
                axis_numbers.append(self.z_axis_channel)

        for axis in axis_numbers:
            if axis == self.x_axis_channel:
                self._move_rel_axis(axis, param_dict['x'])
            elif axis == self.y_axis_channel:
                self._move_rel_axis(axis, param_dict['y'])
            elif axis == self.z_axis_channel:
                self._move_rel_axis(axis, param_dict['z'])

        # return self.get_pos()

    def move_abs(self, param_dict):
        """ Move stage to absolute position

        @param dict param_dict : dictionary, which passes all the relevant
                                 parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.
                                 The values for the axes are in meters,

        @return dict : dictionary with the current axis positions
        """

        invalid_axis = set(param_dict)-set(self._configured_constraints['axis_labels'])

        if invalid_axis:
            for axis in invalid_axis:      
                self.log.warning('Desired axis {axis} is undefined'
                                .format(axis=axis))
                param_dict.remove(axis)

        # TODO: there must be a better way to do this

        # axis_numbers = []

        for axis_label in param_dict:
            axis_channel = self._configured_constraints[axis_label]['channel']
            self._move_abs_axis(axis_channel, param_dict[axis_label])

        # for axis_label in param_dict:
        #     if 'x' in axis_label:
        #         axis_numbers.append(self.x_axis_channel)
        #     if 'y' in axis_label:
        #         axis_numbers.append(self.y_axis_channel)
        #     if 'z' in axis_label:
        #         axis_numbers.append(self.z_axis_channel)

        # for axis in axis_numbers:
        #     if axis == self.x_axis_channel:
        #         self._move_abs_axis(axis, param_dict['x'])
        #     elif axis == self.y_axis_channel:
        #         self._move_abs_axis(axis, param_dict['y'])
        #     elif axis == self.z_axis_channel:
        #         self._move_abs_axis(axis, param_dict['z'])

        # log_file = open("hardware/motor/newfocusdatalog.txt", "w")
        # log_file.write(str(param_dict))
        # log_file.close()

        return self.get_pos()

    def abort(self):
        """Stop movement of the stage with no deceleration

        @return int error code (0:OK, -1:error)
        """

        self._abort()
        return 0

    def get_pos(self, param_dict = None):
        """ Get the current position of the screws

        @param list param_list : optional, if a specific position of an axis
                                 is desired, then the labels of the needed
                                 axis should be passed in the param_list.
                                 If nothing is passed, then the positions of
                                 all axes are returned.

        @return dict pos_dict : with keys being the axis labels and item the current
                                position.
        """

        if param_dict is not None:
            invalid_axis = set(param_dict)-set(self._configured_constraints['axis_labels'])

            if invalid_axis:
                for axis in invalid_axis:      
                    self.log.warning('Desired axis {axis} is undefined'
                                    .format(axis=axis))
                    param_dict.remove(axis)

        # TODO: there still must be a better way to do this
        
        pos_dict = {}

        if param_dict is None:
            param_dict = self._configured_constraints['axis_labels']

        for axis_label in param_dict:
            axis_channel = self._configured_constraints[axis_label]['channel']
            value = int(self._ask('MD?', xx = axis_channel))
            while value == 0:
                time.sleep(0.0005)
                value = int(self._ask('MD?', xx = axis))
            pos_dict[axis_label] = self._get_pos_axis(axis_channel)

        # if param_dict is not None:
        #     for axis_label in param_dict:
        #         if 'x' in axis_label:
        #             if 'x' in self._configured_constraints['axis_labels']:
        #                 axis = self._configured_constraints['x']['channel']
        #                 value = int(self._ask('MD?', xx= axis))
        #                 while value == 0:
        #                     time.sleep(0.0005)
        #                     value = int(self._ask('MD?', xx= axis))
        #                 pos_dict['x'] = self._get_pos_axis(axis)
        #         if 'y' in axis_label:
        #             if 'y' in self._configured_constraints['axis_labels']:
        #                 axis = self._configured_constraints['y']['channel']
        #                 value = int(self._ask('MD?', xx= axis))
        #                 while value == 0:
        #                     time.sleep(0.0005)
        #                     value = int(self._ask('MD?', xx= axis))
        #                 pos_dict['y'] = self._get_pos_axis(axis)
        #         if 'z' in axis_label:
        #             if 'z' in self._configured_constraints['axis_labels']:
        #                 axis = self._configured_constraints['z']['channel']
        #                 value = int(self._ask('MD?', xx= axis))
        #                 while value == 0:
        #                     time.sleep(0.0005)
        #                     value = int(self._ask('MD?', xx= axis))
        #                 pos_dict['z'] = self._get_pos_axis(axis)
        # else:
        #     if 'x' in self._configured_constraints['axis_labels']:
        #                 axis = self._configured_constraints['x']['channel']
        #                 value = int(self._ask('MD?', xx= axis))
        #                 while value == 0:
        #                     time.sleep(0.0005)
        #                     value = int(self._ask('MD?', xx= axis))
        #                 pos_dict['x'] = self._get_pos_axis(axis)

        #     if 'y' in self._configured_constraints['axis_labels']:
        #                 axis = self._configured_constraints['y']['channel']
        #                 value = int(self._ask('MD?', xx= axis))
        #                 while value == 0:
        #                     time.sleep(0.0005)
        #                     value = int(self._ask('MD?', xx= axis))
        #                 pos_dict['y'] = self._get_pos_axis(axis)

        #     if 'z' in self._configured_constraints['axis_labels']:
        #                 axis = self._configured_constraints['z']['channel']
        #                 value = int(self._ask('MD?', xx= axis))
        #                 while value == 0:
        #                     time.sleep(0.0005)
        #                     value = int(self._ask('MD?', xx= axis))
        #                 pos_dict['z'] = self._get_pos_axis(axis)

        return pos_dict

    def get_status(self, param_dict):
        """ Get the status of the position

        @param list param_list : optional, if a specific status of an axis
                                 is desired, then the labels of the needed
                                 axis should be passed in the param_list.
                                 If nothing is passed, then from each axis the
                                 status is asked.

        @return dict status_dict : with the axis label as key and the status number as item.
                                   The meaning of the return value is:
                                   Bit 0: Ready Bit 1: On target Bit 2: Reference drive active Bit 3: Joystick ON
                                   Bit 4: Macro running Bit 5: Motor OFF Bit 6: Brake ON Bit 7: Drive current active
        """

        if param_dict is not None:
            invalid_axis = set(param_dict)-set(self._configured_constraints['axis_labels'])

            if invalid_axis:
                for axis in invalid_axis:      
                    self.log.warning('Desired axis {axis} is undefined'
                                    .format(axis=axis))
                param_dict.remove(axis)

        axis_numbers = []
        status_dict = {}

        if param_dict is not None:
            for axis_label in param_dict:
                if 'x' in axis_label:
                    axis_numbers.append(self.x_axis_channel)
                if 'y' in axis_label:
                    axis_numbers.append(self.y_axis_channel)
                if 'z' in axis_label:
                    axis_numbers.append(self.z_axis_channel)
        else:
            axis_numbers = [self.x_axis_channel, self.y_axis_channel, self.z_axis_channel]

        for axis in axis_numbers:
            if axis == self.x_axis_channel:
                status_dict['x'] = self._done(axis)
            elif axis == self.y_axis_channel:
                status_dict['y'] = self._done(axis)
            elif axis == self.z_axis_channel:
                status_dict['z'] = self._done(axis)

        return status_dict

    def calibrate(self, param_list=None):
        """ Calibrate the stage.

        @param dict param_list: param_list: optional, if a specific calibration
                                of an axis is desired, then the labels of the
                                needed axis should be passed in the param_list.
                                If nothing is passed, then all connected axis
                                will be calibrated.

        After calibration the stage moves to home position which will be the
        zero point for the passed axis.

        @return dict pos: dictionary with the current position of the axis
        """
        self.log.warning('Function not supported')

        pos = {}

        return pos

    def get_velocity(self, param_dict):
        """ Get the current velocity for all connected axes in m/s.

        @param list param_list : optional, if a specific velocity of an axis
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
                    axis_numbers.append(self.x_axis_channel)
                if 'y' in axis_label:
                    axis_numbers.append(self.y_axis_channel)
                if 'z' in axis_label:
                    axis_numbers.append(self.z_axis_channel)
        else:
            axis_numbers = [self.x_axis_channel, self.y_axis_channel, self.z_axis_channel]

        for axis in axis_numbers:
            if axis == self.x_axis_channel:
                velocity_dict['x'] = self._ask_velocity(axis)
            elif axis == self.y_axis_channel:
                velocity_dict['y'] = self._ask_velocity(axis)
            elif axis == self.z_axis_channel:
                velocity_dict['z'] = self._ask_velocity(axis)

        return velocity_dict

    def set_velocity(self, param_dict):
        """ Write new value for velocity in m/s.

        @param dict param_dict : dictionary, which passes all the relevant
                                 parameters, which should be changed. Usage:
                                 {'axis_label': <the-velocity-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        @return dict : dictionary with the updated axis velocity
        """

        # TODO: there must be a better way to do this

        axis_numbers = []

        for axis_label in param_dict:
            if 'x' in axis_label:
                axis_numbers.append(self.x_axis_channel)
            if 'y' in axis_label:
                axis_numbers.append(self.y_axis_channel)
            if 'z' in axis_label:
                axis_numbers.append(self.z_axis_channel)

        for axis in axis_numbers:
            if axis == self.x_axis_channel:
                param_dict['x'] = self._set_velocity_axis(axis, param_dict['x'])
            elif axis == self.y_axis_channel:
                param_dict['y'] = self._set_velocity_axis(axis, param_dict['y'])
            elif axis == self.z_axis_channel:
                param_dict['z'] = self._set_velocity_axis(axis, param_dict['z'])

        return self.get_velocity()

########################## instrument communication ###########################

    def _writeline(self, cmd):
        """ Write a command to the device

            @param str cmd : few-letter command
        """

        self.ep_out.write(cmd.encode() + self.eol_write)
        
    def _readline(self):
        """ Read a response from the device

            @return str r : the device response
        """

        r = self.ep_in.read(64).tobytes()
        assert r.endswith(self.eol_read)
        r = r[:-2].decode()
        return r

    def _fmt_cmd(self, cmd, xx=None, *nn):
        """ Format a command

            @param str cmd : few-letter command
                   int xx  : motor channel (optional for some commands)
                   int nn  : additional parameters (multiple int, optional)
        """

        if xx is not None:
            cmd = "{:d}".format(xx) + cmd
        if nn:
            cmd += ", ".join("{:d}".format(n) for n in nn)
        return cmd

    def _ask(self, cmd, xx=None, *nn):
        """ Query the device

            @param str cmd : few-letter command
                   int xx  : motor channel (optional for some commands)
                   int nn  : additional parameters (multiple int, optional)

            @return str : the device response
        """

        cmd = self._fmt_cmd(cmd, xx, *nn)
        self._writeline(cmd)
        time.sleep(0.1)
        return self._readline()

    def _do(self, cmd, xx=None, *nn):
        """ Format and send a command to the device

            @param str cmd : few-letter command
                   int xx  : motor channel (optional for some commands)
                   int nn  : additional parameters (multiple int, optional)

            See Also:
                :meth:`fmt_cmd`: for the formatting and additional
                    parameters.
        """

        cmd = self._fmt_cmd(cmd, xx, *nn)
        assert len(cmd) < 64
        self._writeline(cmd)

########################## internal methods ###################################

    def _do_move_abs(self, axis, move):
        """ Internal method for the absolute move in meter

        @param str axis   : name of the axis that should be moved
               float move : desired position in meter

        @return str axis   : axis which is moved
                float move : absolute position to move to
        """

        # TODO: implement this

        #self.log.info(axis + 'MA{0}'.format(int(move*1e8)))
        if not(self._configured_constraints[axis]['pos_min'] <= move <= self._configured_constraints[axis]['pos_max']):
            self.log.warning('Cannot make the movement of the axis "{0}"'
                             'since the border [{1},{2}] would be crossed! Ignore command!'
                             ''.format(axis,
                                       self._configured_constraints[axis]['pos_min'],
                                       self._configured_constraints[axis]['pos_max']))
        else:
            self._write_xyz(axis, 'MA{0}'.format(int(move * 1e7)))  # 1e7 to convert meter to SI units
            #self._write_xyz(axis, 'MP')
        return axis, move

    def _set_servo_state(self, to_state):
        """ Internal method enabling / disabling the servos

        @param bool to_state: desired state of the servos
        """

        self.log.warning('This is an open loop device')

        return 1

    def _on_target(self, axis):
        """
        """
        # return bool(self._ask('{}MD?'.format(axis)))
        return bool(self._ask('MD?', axis))

    def _move_rel_axis(self, axis, distance):
        """ Move a device axis relatively from current position

            @param int axis       : axis to be acted upon
                   float distance : distance to be moved
        """
        
        steps = int(float(distance)/float(0.00000003)) #  each step is (apparently) 30nm
        self._do('PR', axis, steps)

        # while not self._on_target(axis):
        #     time.sleep(0.1)
        # time.sleep(1)

        value = int(self._ask('MD?', xx= axis))
        # value = self._on_target(axis)
        while value == 0:
            time.sleep(0.0005)
            value = int(self._ask('MD?', xx= axis))
            # value = self._on_target(axis)
    
    def _move_abs_axis(self, axis, distance):
        """ Move a device axis to an absolute position

            @param int axis       : axis to be acted upon
                   float distance : distance to be moved
        """
        
        steps = int(float(distance)/float(0.00000003)) #  each step is (apparently) 30nm
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
        """ Get the position of a specified axis

            @param int axis : axis to be acted upon

            @return float : axis position
        """
        # target_position = self._ask('PA?',axis)
        # relative_position = self._ask('PR?',axis)
        actual_position_in_steps = float(self._ask('TP?', axis))
        actual_position_in_SI = (actual_position_in_steps*30)*1e-9
        # print ('Get destination position (abs)', axis, target_position)  # debugging
        # print ('Get destination position (rel)', axis, relative_position)  # debugging
        # print ('Get position', axis, actual_position)  # debugging
        return actual_position_in_SI

    def _set_velocity_axis(self, axis, velocity):
        """ Set the velovity of an axis

            @param int axis       : axis to be acted upon
                   float velocity : desired axis velocity
        """

        self._do('VA', axis, velocity)
   
    def _abort(self):
        """ Emergency abort movement (no deceleration)
        """

        self._do('AB')

    def _stop(self, axis):
        """ Stop movement of the device
        """

        self._do('ST', xx=axis)

    def _done(self, axis): #created by Jarrod to get the status of the motor
        """ Get the status of the motor

            @param int axis : axis to be queried

            @return int error code (0:OK, -1:error)
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

    def _ask_velocity(self, axis):
        """ Get the motor velocity

            @param int axis : axis to be queried
        """

        v = self._ask('VA?', axis)
        return v

    def _set_home(self, axis, pos):
        """ Set the home position

            @param int axis  : axis to be queried
                   float pos : desired home position
        """

        self._do('DH', axis, pos)
    
    def _check_home(self):
        """ Get the home position ('ET phone home..')

            @return dict home_dict : contains home position of each axis
        """

        # read_log =  open("hardware/motor/newfocusdatalog.txt", "r+")
        home_dict = eval(open("hardware/motor/newfocusdatalog.txt").read())
        # read_log.close

        # home_dict = self.get_pos({'x','y','z'})
        home_dict['x'] = 0 - float(home_dict['x'])
        home_dict['y'] = 0 - float(home_dict['y'])
        home_dict['z'] = 0 - float(home_dict['z'])
        return home_dict

    def _home_dictionary(self):
        """ TODO What is this?
        """
        self._check_home()

    def _print_to_log(self):
        """ Print positional information to a log file
        """

        log_file = open("hardware/motor/newfocusdatalog.txt", "w")
        log_file.write(str(self.get_pos({'x','y','z'})))
        log_file.close()

        self._check_home()
        self.move_rel(self._check_home())
        self._set_home(self.x_axis_channel, 0)
        self._set_home(self.y_axis_channel, 0)
        self._set_home(self.z_axis_channel, 0)
        self.get_pos({'x','y','z'})
        self._print_to_log()

    def _on_scan_done(self):
        """ Query if the scan is complete
        """
        if scan_done == True: # TODO scan_done is undefined
            self._print_to_log()
