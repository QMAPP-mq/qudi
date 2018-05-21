# -*- coding: utf-8 -*-

"""
This file contains the hardware control for a NT-MDT piezo stage.

N.B. NT-MDT Nova Px control software must be running.

N.B. Nova Px software must be revision 18579 (3.4.0) or newer.

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

    """ Hardware module for communicating with NT-MDT piezo scanning stages
    over USB (via the NovaSDK dll). 
    
    It uses the VB script from the documentation.
    
    unstable: Matt van Breugel

    Example configuration:
    ```
    # ntmdt_stage:
    #     module.Class: 'motor.ntmdt_piezo_stage.PiezoStageNTMDT'
    #     axis_labels:
    #         - x
    #         - y
    #         - z
    #         - tube_x
    #         - tube_y
    #         - tube_z
    #     x:
    #         device_id: 1
    #         channel: 0
    #         constraints:
    #             pos_min: 0e-6
    #             pos_max: 100e-6
    #     y:
    #         device_id: 1
    #         channel: 1
    #         constraints:
    #             pos_min: 0e-6
    #             pos_max: 100e-6
    #     z:
    #         device_id: 1
    #         channel: 2
    #         constraints:
    #             pos_min: 0e-6
    #             pos_max: 6e-6
    #     tube_x:
    #         device_id: 0
    #         channel: 0
    #         constraints:
    #             pos_min: 0e-6
    #             pos_max: 100e-6
    #     tube_y:
    #         device_id: 0
    #         channel: 0
    #         constraints:
    #             pos_min: 0e-6
    #             pos_max: 100e-6
    #     tube_z:
    #         device_id: 0
    #         channel: 0
    #         constraints:
    #             pos_min: 0e-6
    #             pos_max: 6e-6
    ```
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
            path_dll = os.path.join(os.path.abspath(''),
                                    'thirdparty',
                                    'nt_mdt',
                                    'NovaSDK_x64.dll'
                                    )
        elif platform.architecture()[0] == '32bit':
            path_dll = os.path.join(os.path.abspath(''),
                                    'thirdparty',
                                    'nt_mdt',
                                    'NovaSDK.dll'
                                    )
        else:
            self.log.error('Unknown platform, cannot load the Nova SDK dll.')

        self._novadll = ctypes.windll.LoadLibrary(path_dll)
        
        time.sleep(1)
        
        if self._check_connection():
            self.log.info('Nova Px handshake successful')
            self._configuration = self.get_constraints()
            self._set_servo_state(True)
            return 0
        else:
            self.log.error('I cannot connect to Nova Px')
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

        config = self.getConfiguration()

        axis0 = {}
        axis0['label'] = 'x'
        axis0['scanner'] = config['x']['device_id']
        axis0['channel'] = config['x']['channel']
        axis0['pos_min'] = config['x']['constraints']['pos_min']
        axis0['pos_max'] = config['x']['constraints']['pos_max']

        axis1 = {}
        axis1['label'] = 'y'
        axis1['scanner'] = config['y']['device_id']
        axis1['channel'] = config['y']['channel']
        axis1['pos_min'] = config['y']['constraints']['pos_min']
        axis1['pos_max'] = config['y']['constraints']['pos_max']

        axis2 = {}
        axis2['label'] = 'z'
        axis2['scanner'] = config['z']['device_id']
        axis2['channel'] = config['z']['channel']
        axis2['pos_min'] = config['z']['constraints']['pos_min']
        axis2['pos_max'] = config['z']['constraints']['pos_max']

        #  check if the user has specified they have the 'tube' scanner
        if [s for s in config['axis_labels'] if 'tube' in s] is not None:

                axis3 = {}
                axis3['label'] = 'tube_x'
                axis3['scanner'] = config['tube_x']['device_id']
                axis3['channel'] = config['tube_x']['channel']
                axis3['pos_min'] = config['tube_x']['constraints']['pos_min']
                axis3['pos_max'] = config['tube_x']['constraints']['pos_max']

                axis4 = {}
                axis4['label'] = 'tube_y'
                axis4['scanner'] = config['tube_y']['device_id']
                axis4['channel'] = config['tube_y']['channel']
                axis4['pos_min'] = config['tube_y']['constraints']['pos_min']
                axis4['pos_max'] = config['tube_y']['constraints']['pos_max']

                axis5 = {}
                axis5['label'] = 'tube_z'
                axis5['scanner'] = config['tube_z']['device_id']
                axis5['channel'] = config['tube_z']['channel']
                axis5['pos_min'] = config['tube_z']['constraints']['pos_min']
                axis5['pos_max'] = config['tube_z']['constraints']['pos_max']

        # assign the parameter container for x to a name which will identify it
        constraints[axis0['label']] = axis0
        constraints[axis1['label']] = axis1
        constraints[axis2['label']] = axis2

        #  check if the user has specified they have the 'tube' scanner
        if [s for s in config['axis_labels'] if 'tube' in s] is not None:
            constraints[axis3['label']] = axis3
            constraints[axis4['label']] = axis4
            constraints[axis5['label']] = axis5

        if axis0['scanner'] != axis1['scanner']:
            self.log.warning('Your x and y axes are configured as different devices, is this correct?')

        if [s for s in config['axis_labels'] if 'tube' in s] is not None:
            if axis3['scanner'] != axis4['scanner']:
                self.log.warning('Your x and y tube axes are configured as different devices, is this correct?')

        return constraints

    def move_rel(self, param_dict):
        """ Move the stage in given direction (relative movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.


        @return dict pos: dictionary with the current magnet position
        """
        self.log.info('I cannot do this, use the absolute movement function')
        return param_dict

    def move_abs(self, param_dict=None):
        """Move the stage to absolute position

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.
                                The values for the axes are in meter,
                                the value for the rotation is in degrees.

        @return dict pos: dictionary with the current axis position
        """

        invalid_axis = set(param_dict)-set(self._configuration)

        if invalid_axis:
            for axis in invalid_axis:      
                self.log.warning('Desired axis {axis} is undefined'
                                .format(axis=axis))

        for axis in ['x', 'y', 'z']:
            if axis in param_dict.keys():
                scanner = self._configuration[axis]['scanner']
                channel = self._configuration[axis]['channel']
                to_position = param_dict[axis]
                self._do_move_abs(axis, scanner, channel, to_position)
                time.sleep(0.1)

        #  check if the user has specified they have the 'tube' scanner
        if [s for s in self._configuration['axis_labels'] if 'tube' in s] is not None:

            for axis in ['tube_x', 'tube_y', 'tube_z']:
                if axis in param_dict.keys():
                    scanner = self._configuration[axis]['scanner']
                    channel = self._configuration[axis]['channel']
                    to_position = param_dict[axis]
                    self._do_move_abs(axis, scanner, channel, to_position)
                    time.sleep(0.1)

        # # Use this code to populate the returned parmeter dictionary, 
        # # it has been removed to speed-up scanning.
        # self.get_pos()
        # time.sleep(0.1)
        # param_dict = self.get_pos()

        self._update_gui()
        return param_dict

    def abort(self):
        """Stops movement of the stage

        Nova Px software must be revision 18659 or newer for this feature.

        @return int: error code (0:OK, -1:error)
        """
        scanners = []

        for axis in set(self._configuration):
            if self._configuration[axis]['scanner'] not in scanners:
                scanners.append(self._configuration[axis]['scanner'])

        for scanner in scanners:
            self._emergency_interrupt(scanner)

        status = 0

        for scanner in scanners:
            # CInt(Abs(Value))
            command = ('Sc{scanner}Mv = GetParam(tScanner, cStatus, {scanner})\n\n'
                       'Sc{scanner}Mv = CInt(Abs(Sc{scanner}Mv))\n\n'  # bool to int
                       'SetSharedDataVal "shared_Sc{scanner}Mv", Sc{scanner}Mv, "F64", 8'
                       .format(scanner=scanner))
                       
            self._run_script_text(command)
            time.sleep(0.1)
            status += self._get_shared_float('shared_Sc{scanner}Mv'.format(scanner=scanner))
            self._reset_shared_data('shared_Sc{scanner}Mv'.format(axis=axis))
            time.sleep(0.1)
        
        self._update_gui()

        return status

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
                scanner = self._configuration[axis]['scanner']
                channel = self._configuration[axis]['channel']

                command = ('{axis}Pos = GetParam(tScanner, scPosition, {scanner}, {channel})\n\n'
                        'SetSharedDataVal "shared_{axis}Pos", {axis}Pos, "F64", 8'
                        .format(axis=axis, channel=channel, scanner=scanner))

                self._run_script_text(command)
                time.sleep(0.1)
                param_dict[axis] = self._get_shared_float('shared_{axis}Pos'.format(axis=axis))  *1e-6
                # NT-MDT scanner communication in microns
                time.sleep(0.1)

                # reset shared data values
                self._reset_shared_data('shared_{axis}Pos'.format(axis=axis))
                time.sleep(0.1)

        #  check if the user has specified they have the 'tube' scanner
        if [s for s in self._configuration['axis_labels'] if 'tube' in s] is not None:

            for axis in ['tube_x', 'tube_y', 'tube_z']:
                scanner = self._configuration[axis]['scanner']
                channel = self._configuration[axis]['channel']

                command = ('{axis}Pos = GetParam(tScanner, scPosition, {scanner}, {channel})\n\n'
                        'SetSharedDataVal "shared_{axis}Pos", {axis}Pos, "F64", 8'
                        .format(axis=axis, channel=channel, scanner=scanner))

                self._run_script_text(command)
                time.sleep(0.1)
                param_dict[axis] = self._get_shared_float('shared_{axis}Pos'.format(axis=axis))  *1e-6
                # NT-MDT scanner communication in microns
                time.sleep(0.1)
                self._reset_shared_data('shared_{axis}Pos'.format(axis=axis))
                time.sleep(0.1)

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

    def _do_move_abs(self, axis, scanner, channel, to_pos):
        """ Internal method for absolute axis move in meters

        @param string axis: name of the axis to be moved
        @param int channel: channel of the axis to be moved 
        @param float to_pos: desired position in meters
        """

        if not(self._configuration[axis]['pos_min'] <= to_pos <= self._configuration[axis]['pos_max']):
            self.log.warning('Cannot make the movement of the {axis} axis'
                             'since the border [{min},{max}] would be crossed! Ignore command!'
                             .format(axis=axis, min=self._configuration[axis]['pos_min'], max=self._configuration[axis]['pos_max']))
        else:
            self._write_axis_move(axis, scanner, channel, to_pos)

    def _write_axis_move(self, axis, scanner, channel, to_pos):
        """ Internal method to move a specified axis

        @param string axis: name of the axis to be moved
        @param int channel: channel of the axis to be moved 
        @param float to_pos: desired position in meters
        """

        to_pos = to_pos /1e-6  # NT-MDT scanner communication in microns

        command = ('SetParam tScanner, scPosition, {scanner}, {channel}, {position}\n'
                    'Do\n'
                    'idle\n'
                    'Loop Until GetParam(tScanner, cStatus, {scanner}) = False'
                    .format(channel=channel, position=to_pos, scanner=scanner))

        self._run_script_text(command)
        time.sleep(0.1)

########################## Feedback Methods ####################################

    def _set_servo_state(self, to_state):
        """ Internal method enabling / disabling the stage feedback

        @param bool to_state: desired state of the feedback servos
        """
        self._set_servo_state_xy(self._configuration['x']['scanner'], to_state)
        time.sleep(0.5)
        self._update_gui()

        #  not required, but will catch an odd configuration
        self._set_servo_state_xy(self._configuration['y']['scanner'], to_state)
        time.sleep(0.5)
        self._update_gui()

        self._set_servo_state_z(self._configuration['z']['scanner'], to_state)
        time.sleep(0.5)
        self._update_gui()

        #  check if the user has specified they have the 'tube' scanner
        if [s for s in self._configuration['axis_labels'] if 'tube' in s] is not None:
            
            self._set_servo_state_xy(self._configuration['tube_x']['scanner'], to_state)
            time.sleep(0.5)
            self._update_gui()

            #  not required, but will catch an odd configuration
            self._set_servo_state_xy(self._configuration['tube_y']['scanner'], to_state)
            time.sleep(0.5)
            self._update_gui()

            self._set_servo_state_z(self._configuration['tube_z']['scanner'], to_state)
            time.sleep(0.5)
            self._update_gui()

    def _set_servo_state_xy(self, scanner, to_state):
        """ Internal method to enable/disable XY closed loop feedback

        @param bool to_state: the desired state of the feedback loop
        """
        command =   ('SetParam tScanner, cParam, {scanner}, XYCLState, {to_state}'
                    .format(scanner=scanner, to_state=int(to_state)))  # bool to int
        self._run_script_text(command)

    def _set_servo_state_z(self, scanner, to_state):
        """ Internal method to enable/disable Z closed loop feedback

        @param bool to_state: the desired state of the feedback loop
        """
        command =   ('SetParam tScanner, cParam, {scanner}, ZCLState, {to_state}'
                    .format(scanner=scanner, to_state=int(to_state)))  # bool to int
        self._run_script_text(command)

########################## Nova PX Communication ###############################

    def _run_script_text(self, command):
        """ Execute a command in Nova Px

        @param string command: VBScript code to be executed
        """
        self._novadll.RunScriptText(command.encode())

    def _run_script_text_thread(self, command):
        """ Execute a command in a Nova Px in a separate thread

        This function can be used to interrupt a running process. Nova Px 
        software must be revision 18659 or newer for this function.

        @param string command: VBScript code to be executed
        """
        self._novadll.RunScriptTextThread(command.encode())

    def _get_shared_float(self, variable):
        """ Retreive a shared data variable of type float from Nova Px

        @param string variable: The variable must have already been created

        @returns float value: The value of variable
        """
        outbuf = ctypes.c_double()
        buflen = ctypes.c_int()

        self._novadll.GetSharedData(variable.encode(), None, ctypes.byref(buflen))  # get the required buffer size
        self._novadll.GetSharedData(variable.encode(), ctypes.byref(outbuf), ctypes.byref(buflen))  # fill the buffer

        return outbuf.value

    def _reset_shared_data(self, variable):
        """ Reset a shared data variable

        @param string variable: The variable must have already been created
        """
        self._novadll.ResetSharedData(variable.encode())

    def _update_gui(self):
        """ Update the Nova Px graphical user unterface

        this operation is noted to be "not threadsafe" in the original documentation
        """
        command = 'Perform tGlobal, gGUIUpdate'
        self._run_script_text(command)

    def _check_connection(self):
        """ Set and get a shared variable to check the connection with Nova Px

        @returns bool success: True if values match
        """
        command = 'SetSharedDataVal "test_connection", 1.61803398875, "F64", 8'
        self._run_script_text(command)
        if self._get_shared_float('test_connection') == 1.61803398875:
            self._reset_shared_data('test_connection')
            return True
        else:
            return False

########################## Message Box #########################################

    def _make_message(self, message):
        """ Make a message box appear in Noxa Px. Use for debugging.

        @param string message: Message to be displayed.
        """
        command = 'msgbox "{message}"'.format(message=message)
        self._novadll.RunScriptText(command.encode())

########################## Thermal Controls ####################################

    def _toggle_heating(self, to_state):
        """ Enable or disable the stage heater

        @param bool to_state: The desired state of the heater
        """
        command =   ('SetParam tThermoController, thHeatingEnabled, {state}'
                    .format(to_state=int(to_state)))  # 0 - off, 1 - on
        self._run_script_text(command)

    def _set_temperature_setpoint(self, setpoint):
        """ Set the temperature setpoint of the heater

        @param float setpoint: The desired temperature setpoint
        """
        command =   ('SetParam tThermoController, thSetPoint, {temp}'
                    .format(temp=setpoint))
        self._run_script_text(command)

    def _get_heater_power(self):
        """ Get the current heater power in %

        @returns float power: The current heater power in %
        """

        command = ('ThPower = GetParam(tThermoController, thPower)\n\n'
                    'SetSharedDataVal "shared_ThPower", ThPower, "F64", 8'
                    )

        self._run_script_text(command)
        time.sleep(0.1)
        power = self._get_shared_float('shared_ThPower')
        # NT-MDT scanner communication in %
        time.sleep(0.1)

        self._reset_shared_data('shared_ThPower')
        time.sleep(0.1)

        return power

    def _get_temperature(self, channel_list=None):
        """ Get the current temperature value

        @param list channel_list: optional, if a specific temperature of a channel
                                is desired, then the labels of the needed
                                channel should be passed in the channel_list.
                                If nothing is passed, then the temperatures of
                                all channels are returned.

        @return dict: with keys being the channel labels and item the current
                      temperature.
        """

        for channel in channel_list:
            channel = str(channel)  # convert ints to strings

        channel_dict = {}
        
        for channel in ['1', '2', '3']:

            command = ('T{channel} = GetParam(tThermoController, thT{channel}CurValue)\n\n'
                       'SetSharedDataVal "shared_T{channel}", T{channel}, "F64", 8'
                       .format(channel=channel))

            self._run_script_text(command)
            time.sleep(0.1)
            channel_dict[channel] = self._get_shared_float('shared_T{channel}'.format(channel=channel))
            # NT-MDT scanner communication in celcius
            time.sleep(0.1)

        for channel in ['1', '2', '3']:  # reset shared data values
            self._reset_shared_data('shared_T{channel}'.format(channel=channel))
            time.sleep(0.1)

        if channel_list:
            for channel in list(set(channel_dict.keys()) - set(channel_list)):  # channels not in channel_list
                del channel_dict[channel]
            return channel_dict
        else:
            return channel_dict

########################## Interrupt Movement ##################################

    def _emergency_interrupt(self, scanner):
        """ Abort motion of a scanners

        To abort the movement of a scanner in motion, the command must be 
        called in a separate thread. See self._run_script_text_thread.

        @param int scanner: The scanner to be stopped
        """
        self.log.warning('Aborting the motion of NT-MDT scanner {scanner}'.format(scanner=scanner))

        command = ('Perform tScanner, scStop, {scanner}'
                    .format(scanner=scanner))

        self._run_script_text_thread(command)
        time.sleep(0.1)
