# -*- coding: utf-8 -*-

"""
This file contains the hardware control for a Attocube ECC100 piezo stage.

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

from ctypes import (c_int, c_int32, c_int16, c_uint32, c_int64, 
                    c_byte, c_ubyte, c_short, c_double, cdll, pointer, 
                    byref)


import numpy as np
from collections import namedtuple


class PiezoStageATTOCUBE(Base, MotorInterface):

    """ Hardware module for communicating with Attocube ECC100 piezo scanning stages
    over USB (via the Attocube dll). 
    
    unstable: Reece Roberts and Guillermo Munoz

    Example configuration:
    ```
        TODO: write an example configuration
    ```
    """
    _modclass = 'PiezoStageATTOCUBE'
    _modtype = 'hardware'

    _device_id = ConfigOption('device_id', missing='error')

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def on_activate(self):
        """ Initialisation performed during activation of the module.

            @return: error code (0:OK, -1:error)
        """
        if platform.architecture()[0] == '64bit':
            path_dll = os.path.join(os.path.abspath(''),
                                    'thirdparty',
                                    'attocube',
                                    'ECC100_DLL',
                                    'Win_64Bit',
                                    'lib',
                                    'ecc.dll'
                                    )
        elif platform.architecture()[0] == '32bit':
            path_dll = os.path.join(os.path.abspath(''),
                                    'thirdparty',
                                    'attocube',
                                    'ECC100_DLL',
                                    'Win_32Bit',
                                    'lib',
                                    'ecc.dll'
                                    )
        else:
            self.log.error('Unknown platform, cannot load the ECC100 dll.')

        self._eccdll = ctypes.WinDLL(path_dll)
        
        time.sleep(1)

        self._configured_constraints = self.get_constraints()

        

        _eccdev = AttoCubeECC100(device_id = _device_id, debug=True)
        
        if self._check_connection():
            self.log.info('ECC100 handshake successful')
            self._set_servo_state(True)
            return 0
        else:
            self.log.error('I cannot connect to ECC100 and all three ECS5050 stages')
            return 1

    
     
    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
            @return: error code (0:OK, -1:error)
        """
        self._set_servo_state(False)
        return 0

    def get_constraints(self):
        """ Retrieve the hardware constrains from the motor device.

        @return dict: dict with constraints for the sequence generation and GUI

        Provides all the constraints for the xyz stage and rot stage (like total
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
        if [s for s in config['axis_labels'] if 'tube' in s]:

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
        if [s for s in config['axis_labels'] if 'tube' in s]:
            constraints[axis3['label']] = axis3
            constraints[axis4['label']] = axis4
            constraints[axis5['label']] = axis5

        if axis0['scanner'] != axis1['scanner']:
            self.log.warning('Your x and y axes are configured as different devices, is this correct?')

        if [s for s in config['axis_labels'] if 'tube' in s]:
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
        self.log.warning('I cannot do this, use the absolute movement function')
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

        invalid_axis = set(param_dict)-set(self._configured_constraints)

        if invalid_axis:
            for axis in invalid_axis:      
                self.log.warning('Desired axis {axis} is undefined'
                                .format(axis=axis))

        for axis in ['x', 'y', 'z']:
            if axis in param_dict.keys():
                scanner = self._configured_constraints[axis]['scanner']
                channel = self._configured_constraints[axis]['channel']
                to_position = param_dict[axis]
                self._do_move_abs(axis, scanner, channel, to_position)
                time.sleep(0.1)

        #  check if the user has specified they have the 'tube' scanner
        if [s for s in self._configured_constraints if 'tube' in s]:

            for axis in ['tube_x', 'tube_y', 'tube_z']:
                if axis in param_dict.keys():
                    scanner = self._configured_constraints[axis]['scanner']
                    channel = self._configured_constraints[axis]['channel']
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

        for axis in set(self._configured_constraints):
            if self._configured_constraints[axis]['scanner'] not in scanners:
                scanners.append(self._configured_constraints[axis]['scanner'])

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
                scanner = self._configured_constraints[axis]['scanner']
                channel = self._configured_constraints[axis]['channel']

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
        if [s for s in self._configured_constraints if 'tube' in s]:

            for axis in ['tube_x', 'tube_y', 'tube_z']:
                scanner = self._configured_constraints[axis]['scanner']
                channel = self._configured_constraints[axis]['channel']

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
        self.log.warning('This operation is not yet supported')

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
        self.log.warning('This operation is not yet supported')

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
        self.log.warning('This operation is not yet supported')

    def set_velocity(self, param_dict):
        """ Write new value for velocity in m/s.

        @param dict param_dict: dictionary, which passes all the relevant
                                    parameters, which should be changed. Usage:
                                     {'axis_label': <the-velocity-value>}.
                                     'axis_label' must correspond to a label given
                                     to one of the axis.

        @return dict param_dict2: dictionary with the updated axis velocity
        """
        self.log.warning('This operation is not yet supported')

########################## internal methods ####################################

    def _do_move_abs(self, axis, scanner, channel, to_pos):
        """ Internal method for absolute axis move in meters

        @param string axis  : name of the axis to be moved
               int channel  : channel of the axis to be moved 
               float to_pos : desired position in meters
        """

        if not(self._configured_constraints[axis]['pos_min'] <= to_pos <= self._configured_constraints[axis]['pos_max']):
            self.log.warning('Cannot make the movement of the {axis} axis'
                             'since the border [{min},{max}] would be crossed! Ignore command!'
                             .format(axis=axis, min=self._configured_constraints[axis]['pos_min'], max=self._configured_constraints[axis]['pos_max']))
        else:
            self._write_axis_move(axis, scanner, channel, to_pos)

    def _write_axis_move(self, axis, scanner, channel, to_pos):
        """ Internal method to move a specified axis

        @param string axis  : name of the axis to be moved
               int channel  : channel of the axis to be moved 
               float to_pos : desired position in meters
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

        @param bool to_state : desired state of the feedback servos
        """

        self._set_servo_state_xy(self._configured_constraints['x']['scanner'], to_state)
        time.sleep(0.5)
        self._update_gui()

        #  not required, but will catch an odd configuration
        self._set_servo_state_xy(self._configured_constraints['y']['scanner'], to_state)
        time.sleep(0.5)
        self._update_gui()

        self._set_servo_state_z(self._configured_constraints['z']['scanner'], to_state)
        time.sleep(0.5)
        self._update_gui()

        #  check if the user has specified they have the 'tube' scanner
        if [s for s in self._configured_constraints if 'tube' in s]:
            
            self._set_servo_state_xy(self._configured_constraints['tube_x']['scanner'], to_state)
            time.sleep(0.5)
            self._update_gui()

            #  not required, but will catch an odd configuration
            self._set_servo_state_xy(self._configured_constraints['tube_y']['scanner'], to_state)
            time.sleep(0.5)
            self._update_gui()

            self._set_servo_state_z(self._configured_constraints['tube_z']['scanner'], to_state)
            time.sleep(0.5)
            self._update_gui()

    def _set_servo_state_xy(self, scanner, to_state):
        """ Internal method to enable/disable XY closed loop feedback

        @param int scanner   : the scanner number
               bool to_state : the desired state of the feedback loop
        """

        command =   ('SetParam tScanner, cParam, {scanner}, XYCLState, {to_state}'
                    .format(scanner=scanner, to_state=int(to_state)))  # bool to int
        self._run_script_text(command)

    def _set_servo_state_z(self, scanner, to_state):
        """ Internal method to enable/disable Z closed loop feedback

        @param int scanner   : the scanner number
               bool to_state : the desired state of the feedback loop
        """

        command =   ('SetParam tScanner, cParam, {scanner}, ZCLState, {to_state}'
                    .format(scanner=scanner, to_state=int(to_state)))  # bool to int
        self._run_script_text(command)

########################## Nova PX Communication ###############################

    def _run_script_text(self, command):
        """ Execute a command in Nova Px

        @param string command : VBScript code to be executed
        """

        self._eccdll.RunScriptText(command.encode())

    def _run_script_text_thread(self, command):
        """ Execute a command in a Nova Px in a separate thread

        This function can be used to interrupt a running process. Nova Px 
        software must be revision 18659 or newer for this function.

        @param string command : VBScript code to be executed
        """

        self._eccdll.RunScriptTextThread(command.encode())

    def _get_shared_float(self, variable):
        """ Retreive a shared data variable of type float from Nova Px

        @param string variable : The variable must have already been created

        @returns float value : The value of variable
        """

        outbuf = ctypes.c_double()
        buflen = ctypes.c_int()

        self._eccdll.GetSharedData(variable.encode(), None, ctypes.byref(buflen))  # get the required buffer size
        self._eccdll.GetSharedData(variable.encode(), ctypes.byref(outbuf), ctypes.byref(buflen))  # fill the buffer

        return outbuf.value

    def _reset_shared_data(self, variable):
        """ Reset a shared data variable

        @param string variable : The variable must have already been created
        """

        self._eccdll.ResetSharedData(variable.encode())

    def _update_gui(self):
        """ Update the Nova Px graphical user unterface

        this operation is noted to be "not threadsafe" in the original documentation
        """

        command = 'Perform tGlobal, gGUIUpdate'
        self._run_script_text(command)

    def _check_connection(self):
        """ Check that all three stages are connected and match the ECS5050 linear actuator.

        @returns bool success : True if values match
        """
        if _eccdev.read_actor_info(0) == ('ECS5050', 'ECC_actorLinear') and _eccdev.read_actor_info(1) == ('ECS5050', 'ECC_actorLinear') and _eccdev.read_actor_info(2) == ('ECS5050', 'ECC_actorLinear'):
            return True 
        else:
            return False



########################## Message Box #########################################

    def _make_message(self, message):
        """ Make a message box appear in Noxa Px. Use for debugging.

        @param string message : Message to be displayed in a box.
        """

        command = 'msgbox "{message}"'.format(message=message)
        self._eccdll.RunScriptText(command.encode())

########################## Thermal Controls ####################################

    def _toggle_heating(self, to_state):
        """ Enable or disable the stage heater

        @param bool to_state : The desired state of the heater
        """

        command =   ('SetParam tThermoController, thHeatingEnabled, {state}'
                    .format(to_state=int(to_state)))  # 0 - off, 1 - on
        self._run_script_text(command)

    def _set_temperature_setpoint(self, setpoint):
        """ Set the temperature setpoint of the heater

        @param float setpoint : The desired temperature setpoint
        """

        command =   ('SetParam tThermoController, thSetPoint, {temp}'
                    .format(temp=setpoint))
        self._run_script_text(command)

    def _get_heater_power(self):
        """ Get the current heater power in %

        @returns float power : The current heater power in %
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

        @param list channel_list : optional, if a specific temperature of a channel
                                   is desired, then the labels of the needed
                                   channel should be passed in the channel_list.
                                   If nothing is passed, then the temperatures of
                                   all channels are returned.

        @return dict : with keys being the channel labels and item the current
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

        @param int scanner : The scanner to be stopped
        """

        self.log.warning('Aborting the motion of NT-MDT scanner {scanner}'.format(scanner=scanner))

        command = ('Perform tScanner, scStop, {scanner}'
                    .format(scanner=scanner))

        self._run_script_text_thread(command)
        time.sleep(0.1)


############################ Class file of attocube control #############################
class AttoCubeECC100(object):
    
    def __init__(self, device_num=0, device_id = None, debug=False):
        self.debug = debug
        self.device_num = device_num
        
        if self.debug:
            print("Initializing AttoCubeECC100 device ", device_num)
        
        #self.num_devices = ecc.ECC_Check()
        self.dev_list = ecc_enumerate()
        
        # if device_id is defined, find the appropriate device_num
        if device_id is not None:
            dev_id_found = False
            for dev in self.dev_list:
                if dev.dev_id == device_id:
                    self.device_num = dev.dev_num
                    self.device_id = dev.dev_id
                    dev_id_found = True
            if not dev_id_found:
                ## no device based on ID found
                raise IOError("AttoCubeECC100 No Device found based on device_id={}".format(device_id))
        else:
            self.device_id = self.dev_list[self.device_num].dev_id
                    
        assert 0 <= self.device_num < len(self.dev_list), "Attocube device num out of range: {} of {}".format(self.device_num, len(self.dev_list))

        # check if device is locked
        assert not self.dev_list[self.device_num].dev_locked
        
        # Connect to Device
        self.devhandle = c_uint32()
        handle_err(ecc.ECC_Connect(self.device_num,byref(self.devhandle)))

        self.device_id = self.read_device_id()
        
        
    def close(self):
        handle_err(ecc.ECC_Close(self.devhandle))


    def read_actor_info(self, axis):
        return self.read_actor_name(axis), self.read_actor_type(axis)
    
    def read_actor_name(self,axis):
        actor_name = ctypes.create_string_buffer(20)
        handle_err(ecc.ECC_getActorName(
                            self.devhandle,
                            axis, # Int32 axis
                            byref(actor_name), # char * name
                            ))
        return actor_name.value.decode('ascii').strip()
    
    def read_actor_type(self,axis):
        actor_type_id = c_int32()
        handle_err(ecc.ECC_getActorType(
                            self.devhandle,
                            axis, # Int32 axis
                            byref(actor_type_id) #ECC_actorType * type (enum)
                            ))
        return ECC_ACTOR_TYPES[actor_type_id.value]

    def read_device_id(self):
        dev_id = ctypes.c_int32()
        handle_err(ecc.ECC_controlDeviceId(self.devhandle, byref(dev_id), False))
        return dev_id.value

    def read_enable_axis(self, axis):
        cenable = c_int32()
        handle_err(ecc.ECC_controlOutput(self.devhandle,
                                 axis, #axis
                                 byref(cenable), #Bln32 * enable,
                                 0, # read
                                 ))
        return cenable.value
        
    def enable_axis(self, axis, enable=True):
        cenable = c_int32(int(enable))
        handle_err(ecc.ECC_controlOutput(self.devhandle,
                                 axis, #axis
                                 byref(cenable), #Bln32 * enable,
                                 1, # set
                                 ))
        
    def read_enable_closedloop_axis(self, axis):
        cenable = c_int32()
        handle_err(ecc.ECC_controlMove(self.devhandle,
                                 axis, #axis
                                 byref(cenable), #Bln32 * enable,
                                 0, # read
                                 ))
        return cenable.value
        
    def enable_closedloop_axis(self, axis, enable=True):
        cenable = c_int32(int(enable))
        handle_err(ecc.ECC_controlMove(self.devhandle,
                                 axis, #axis
                                 byref(cenable), #Bln32 * enable,
                                 1, # set
                                 ))


    def single_step(self, axis, direction=True):
        """direction True (or >0): forward, False (or <=0): backward"""
        backward= (direction <= 0)
        #backward: Selects the desired direction. False triggers a forward step, true a backward step.  
        handle_err(ecc.ECC_setSingleStep(self.devhandle, # device handle
                                 axis,  # axis
                                 int(backward))) #backward (direction control)

    def single_step_forward(self, axis):
        self.single_step(axis, True)
    def single_step_backward(self, axis):
        self.single_step(axis, False)


    def read_position_axis(self, axis):
        """returns position in mm, device speaks nm
        """
        pos = c_int32()
        handle_err(ecc.ECC_getPosition( 
                                self.devhandle, #Int32 deviceHandle,
                                axis, #Int32 axis,
                                byref(pos))) #Int32* position );
        return pos.value*1e-6


    def is_electrically_connected(self, axis):
        """Connected status.
        Retrieves the connected status. Indicates whether an actor is eletrically connected to the controller.
        """
        connected = c_int32()
        handle_err(ecc.ECC_getStatusConnected(
                                self.devhandle,
                                axis,
                                byref(connected)))
        return bool(connected.value)

    def read_reference_position(self, axis):
        """returns position in mm, device speaks nm
        """
        refpos = c_int32()
        handle_err(ecc.ECC_getReferencePosition(
                                self.devhandle,
                                axis, #Int32 axis
                                byref(refpos), #Int32* reference
                                ))
        return refpos.value*1e-6

    def read_reference_status(self, axis):
        """
        Reference status.
        Retrieves the status of the reference position. It may be valid or invalid.
        """
        valid = c_int32()
        handle_err(ecc.ECC_getStatusReference(
                                  self.devhandle,
                                  axis,
                                  byref(valid)))
        return bool(valid.value)
    
    def read_target_range_axis(self, axis):
        raise NotImplementedError()
    
    def write_target_position_axis(self, axis, target_pos):
        """ position in mm, device speaks nm
        """
        tpos = c_int32(int(target_pos*1e6))
        handle_err(ecc.ECC_controlTargetPosition(
                            self.devhandle,
                            axis, #Int32 axis
                            byref(tpos), # Int32* target
                            1, #Bln32 set
                            ))
        time.sleep(0.000010)
        return tpos.value*1e-6
                   
    def read_target_position_axis(self, axis):
        tpos = c_int32()
        handle_err(ecc.ECC_controlTargetPosition(
                            self.devhandle,
                            axis, #Int32 axis
                            byref(tpos), # Int32* target
                            0, #Bln32 set
                            ))
        if self.debug: print('ecc100 read_target_position_axis', axis, tpos.value)

        return tpos.value*1e-6
    
    def read_target_status(self, axis):
        """
        Target status. 
        Retrieves the target status. Indicates whether the actual 
        position is within the target range.
        """
        target_status = c_uint32()
        handle_err(ecc.ECC_getStatusTargetRange(
                            self.devhandle,
                            axis, #Int32 axis
                            byref(target_status), # Bln32* target                            
                            ))
        return bool(target_status.value)

    def read_eot_back_status(self, axis):
        """
        Target status. 
        Retrieves eot end of travel status (pro).
        """
        eot_status = c_uint32()
        handle_err(ecc.ECC_getStatusEotBkwd(
                            self.devhandle,
                            axis, #Int32 axis
                            byref(eot_status), # Bln32* target                            
                            ))
        return bool(eot_status.value)

    def read_eot_forward_status(self, axis):
        """
        Target status. 
        Retrieves eot end of travel status (pro).
        """
        eot_status = c_uint32()
        handle_err(ecc.ECC_getStatusEotFwd(
                            self.devhandle,
                            axis, #Int32 axis
                            byref(eot_status), # Bln32* target                            
                            ))
        return bool(eot_status.value)

    def read_eot_stop_status(self, axis):
        """
        Target status. 
        Retrieves eot end of travel status (pro).
        """
        eot_status = c_uint32()
        handle_err(ecc.ECC_controlEotOutputDeactive(
                            self.devhandle,
                            axis, #Int32 axis
                            byref(eot_status), # Bln32* target  
                            0, #set                          
                            ))
        return bool(eot_status.value)
    
    def enable_eot_stop(self, axis, enable):
        """
        Target status. 
        Retrieves eot end of travel status (pro).
        """
        eot_status = c_uint32(enable)
        handle_err(ecc.ECC_controlEotOutputDeactive(
                            self.devhandle,
                            axis, #Int32 axis
                            byref(eot_status), # Bln32* target  
                            1, #set                          
                            ))
        return bool(eot_status.value)

    def read_enable_eot_stop(self, axis ):
        """
        Target status. 
        Retrieves eot end of travel status (pro).
        """
        eot_status = c_uint32()
        handle_err(ecc.ECC_controlEotOutputDeactive(
                            self.devhandle,
                            axis, #Int32 axis
                            byref(eot_status), # Bln32* target  
                            0, #set                          
                            ))
        return bool(eot_status.value)

    def read_frequency(self, axis):
        """returns Frequency in Hz, device speaks mHz"""
        freq = c_int32()
        handle_err(ecc.ECC_controlFrequency(
                            self.devhandle,
                            axis, #Int32 axis
                            byref(freq), #Int32* frequency
                            0, # Bln32 set
                            ))
        return freq.value*1e-3
    
    def write_frequency(self,axis, frequency):
        """freq: Frequency in mHz"""
        freq = c_int32(int(frequency*1e3))
        handle_err(ecc.ECC_controlFrequency(
                            self.devhandle,
                            axis, #Int32 axis
                            byref(freq), #Int32* frequency
                            1, # Bln32 set
                            ))
        return freq.value*1e-3
        
    def read_openloop_voltage(self, axis):
        """ Read open loop analog voltage adjustment
        
        returns voltage in V, unit speaks uV
        
        requires Pro version
        """
        ol_volt = c_int32()
        handle_err(ecc.ECC_controlFixOutputVoltage(
                            self.devhandle,
                            axis,
                            byref(ol_volt),# Int32 * voltage
                            0, #set
                            ))
        return ol_volt.value*1e-6
    
    def write_openloop_voltage(self, axis, voltage):
        """ Write open loop analog voltage adjustment
            voltage in V, unit speaks uV
        """
        ol_volt = c_int32(voltage*1e6)
        handle_err(ecc.ECC_controlFixOutputVoltage(
                            self.devhandle,
                            axis,
                            byref(ol_volt),# Int32 * voltage
                            1, #set
                            ))
        return ol_volt.value*1e-6

    def enable_ext_trigger(self, axis):
        raise NotImplementedError()

    def start_continuous_motion(self, axis, direction):
        """
        + 1 continuous motion start in Forward (+) direction
        - 1 continuous motion start in Backward (-) direction
        0   stop continuous motion
        
        Int32 NCB_API ECC_controlContinousFwd( Int32 deviceHandle,
                                       Int32 axis,
                                       Bln32* enable,
                                       Bln32 set );
        """
        c_enable = c_int32(1) # true to start motion
        if direction > 0:
            handle_err(ecc.ECC_controlContinousFwd(self.devhandle, axis, byref(c_enable), 1))
        elif direction < 0:
            handle_err(ecc.ECC_controlContinousBkwd(self.devhandle, axis, byref(c_enable), 1))
        else:
            self.stop_continous_motion(axis)
        
    def stop_continous_motion(self, axis):
        
        """The parameter "false" stops all movement of the axis regardless its direction.
        """
        c_enable = c_int32(0) # stop motion
        handle_err(ecc.ECC_controlContinousFwd(self.devhandle, axis, byref(c_enable), 1))


    def read_continuous_motion(self, axis):
        """ returns +1, 0, or -1
         + 1 continuous motion happening in Forward  (+) direction
         - 1 continuous motion happening in Backward (-) direction
           0 continuous motion stopped
        """

        c_enable = c_int32()
        handle_err(ecc.ECC_controlContinousFwd(self.devhandle,
                                 axis, #axis
                                 byref(c_enable), #Bln32 * enable,
                                 0, # read
                                 ))
        if c_enable.value:
            return +1
        
        c_enable = c_int32()
        handle_err(ecc.ECC_controlContinousBkwd(self.devhandle,
                                 axis, #axis
                                 byref(c_enable), #Bln32 * enable,
                                 0, # read
                                 ))
        if c_enable.value:
            return -1
        
        return 0
        
    
    def read_enable_auto_update_reference(self, axis):
        c_enable = c_int32()
        handle_err(ecc.ECC_controlReferenceAutoUpdate(self.devhandle,
                                 axis, #axis
                                 byref(c_enable), #Bln32 * enable,
                                 0, # read
                                 ))
        return c_enable.value
        
    def enable_auto_update_reference(self, axis, enable=True):
        c_enable = c_int32(enable)
        handle_err(ecc.ECC_controlReferenceAutoUpdate(self.devhandle,
                                 axis, #axis
                                 byref(c_enable), #Bln32 * enable,
                                 1, # set
                                 ))
        return c_enable.value
        
    def read_enable_auto_reset_reference(self, axis):
        c_enable = c_int32()
        handle_err(ecc.ECC_controlAutoReset(self.devhandle,
                                 axis, #axis
                                 byref(c_enable), #Bln32 * enable,
                                 0, # read
                                 ))
        return c_enable.value
        
    def enable_auto_reset_reference(self, axis, enable=True):
        c_enable = c_int32(enable)
        handle_err(ecc.ECC_controlAutoReset(self.devhandle,
                                 axis, #axis
                                 byref(c_enable), #Bln32 * enable,
                                 1, # set
                                 ))
        return c_enable.value
        
    def read_step_voltage(self, axis):
        """
        Control amplitude in V, device uses mV
        Read the amplitude of the actuator signal.
        """
        ampl = c_int32()
        handle_err(ecc.ECC_controlAmplitude(
                            self.devhandle,
                            axis, # Int32 axis
                            byref(ampl), #Int32* amplitude
                            0, #set
                            ))
        return ampl.value*1e-3
        
    def write_step_voltage(self, axis, volts=30):
        """
        Control amplitude in V, device uses mV
        Read the amplitude of the actuator signal.
        """
        ampl = c_int32(int(volts*1e3))
        handle_err(ecc.ECC_controlAmplitude(
                            self.devhandle,
                            axis, # Int32 axis
                            byref(ampl), #Int32* amplitude
                            1, #set
                            ))
        return ampl.value*1e-3
        
    
    def reset_axis(self,axis):
        """
        Reset position.
        Resets the actual position to zero and marks the reference position as invalid.
        
        """
        handle_err(ecc.ECC_setReset(self.devhandle, axis))