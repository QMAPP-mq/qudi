# -*- coding: utf-8 -*-
"""
This module controlls an M Squared laser (SolsTiS head).

NOTE: It is important that the computer connecting to the SolsTiS
has the same IP address as configured on the SolsTiS under
Network Settings -> Remote Interface

NOTE: This hardware module is currently not utilising a wavelength
meter, which may have been installed with your M Squared laser.

NOTE: Tested on M Squared SolsTiS running:
Interface version: Solstis_3_NS_V54
DSP version: Solstis 3 Rel 5.22 06-09-16 10:23
This information can be found under Configure > Interface Update

NOTE: SolsTiS 3 TCP/IP Protocol Version 21 used for this system.

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

import math
import random
import time

import socket
# import numpy as np
import json

from core.module import Base, ConfigOption
from interface.simple_laser_interface import SimpleLaserInterface
from interface.simple_laser_interface import LaserState
from interface.simple_laser_interface import ShutterState
from interface.simple_laser_interface import ControlMode

class MSquaredLaser(Base, SimpleLaserInterface):

    """ Matt van Breugel, Lachlan J. Rogers
    M Squared ultra narrow linewidth, Ti:Sapphire laser

    Example configuration:
    ```
    mylaser:
        module.Class: 'laser.msquared_laser.MSquaredLaser'
        laser_ip: '192.168.1.222'
        laser_port: 39933
        default_x_alignment_param: 77.85
        default_y_alignment_param: 62.80
    ```
    """

    _modclass = 'msquaredlaser'
    _modtype = 'hardware'

    _ipaddy = ConfigOption('laser_ip', missing='error')
    _port = ConfigOption('laser_port', missing='error')

    _beam_align_x = None
    _beam_align_y = None

    _my_beam_align_x_default = ConfigOption('default_x_alignment_param', 50, missing='warn')
    _my_beam_align_y_default = ConfigOption('default_y_alignment_param', 50, missing='warn')

    _beam_align_mode = None

    wavelength = None
    wavelength_lock = False

    def __init__(self, **kwargs):
        """ """
        super().__init__(**kwargs)
        self.lstate = LaserState.UNKNOWN
        self.shutter = ShutterState.UNKNOWN
        self.mode = ControlMode.MIXED  # simple laser interface contains no appripriate control mode
        self.current_setpoint = -1
        self.power_setpoint = -1

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def on_activate(self):
        """ Activate module.
        """
        try:
            err = self._connect(self._ipaddy, self._port)
            if err == 0:
                self.log.info('Connected to M-Squared hardware')
                self.wavelength = self.get_wavelength()
                # self.wavelength_lock = self._set_wavelength_lock('on')  # For use with attached wavelength meter

                self._beam_align_mode = self._set_beam_align_mode('manual')

                if self._get_beam_align() != (50., 50.):  # if Beam X and Beam Y are not set to defaults
                    self._beam_align_x, self._beam_align_y = self._get_beam_align() # update the beam alignment variables
                else:  # else set the beam alignment parameters to my defaults and update the beam alignment variables
                    self._beam_align_x, self._beam_align_y = self._set_beam_align(self._my_beam_align_x_default, self._my_beam_align_y_default)

            else:
                self.log.error('Attempt to connect M-Squared laser returned'
                               'error code {}'.format(err)
                               )
        except:
            self.log.error('The attempt to connect raised an exception.')

    def on_deactivate(self):
        """ Deactivate module.
        """
        self.s.close()  # close the connection

    def get_power_range(self):
        """ Return optical power range

            @return (float, float): power range
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return (-1, -1)

    def get_power(self):
        """ Return laser power

            @return float: Laser power in watts
        """ 
        
        self.log.warning('This operation is not supported by the M Squared laser')
        return -1

    def get_power_setpoint(self):
        """ Return optical power setpoint.

            @return float: power setpoint in watts
        """ 
        
        self.log.warning('This operation is not supported by the M Squared laser')
        return self.power_setpoint

    def set_power(self, power):
        """ Set power setpoint.

            @param float power: power setpoint

            @return float: actual new power setpoint
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return self.power_setpoint

    def get_current_unit(self):
        """ Get unit for laser current.

            @return str: unit
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return ''

    def get_current_range(self):
        """ Get laser current range.

            @return (float, float): laser current range
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return (-1, -1)

    def get_current(self):
        """ Get current laser current

            @return float: laser current in current curent units
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return self.current_setpoint

    def get_current_setpoint(self):
        """ Get laser curent setpoint

            @return float: laser current setpoint
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return self.current_setpoint

    def set_current(self, current):
        """ Set laser current setpoint

            @prarm float current: desired laser current setpoint

            @return float: actual laser current setpoint
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return self.current_setpoint

    def allowed_control_modes(self):
        """ Get supported control modes

            @return list(): list of supported ControlMode
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return []

    def get_control_mode(self):
        """ Get the currently active control mode

            @return ControlMode: active control mode
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return self.mode

    def set_control_mode(self, control_mode):
        """ Set the active control mode

            @param ControlMode control_mode: desired control mode

            @return ControlMode: actual active ControlMode
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return self.mode

    def on(self):
        """ Turn on laser.

            @return LaserState: actual laser state
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        self.lstate = LaserState.UNKNOWN
        return self.lstate

    def off(self):
        """ Turn off laser.

            @return LaserState: actual laser state
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        self.lstate = LaserState.UNKNOWN
        return self.lstate

    def get_laser_state(self):
        """ Get laser state

            @return LaserState: actual laser state
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return self.lstate

    def set_laser_state(self, state):
        """ Set laser state.

            @param LaserState state: desired laser state

            @return LaserState: actual laser state
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return self.lstate

    def get_shutter_state(self):
        """ Get laser shutter state

            @return ShutterState: actual laser shutter state
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return self.shutter

    def set_shutter_state(self, state):
        """ Set laser shutter state.

            @param ShutterState state: desired laser shutter state

            @return ShutterState: actual laser shutter state
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        return self.shutter

    def get_temperatures(self):
        """ Get all available temperatures.

            @return dict: dict of temperature namce and value in degrees Celsius
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        pass

    def set_temperatures(self, temps):
        """ Set temperatures for lasers with tunable temperatures.

            @return {}: empty dict, dummy not a tunable laser
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        pass

    def get_temperature_setpoints(self):
        """ Get temperature setpoints.

            @return dict: temperature setpoints for temperature tunable lasers
        """ 
        self.log.warning('This operation is not supported by the M Squared laser')
        pass

    def get_extra_info(self):
        """ Multiple lines of dignostic information

            @return str: much laser, very useful
        """
        return "much laser, very useful"

    def get_wavelength(self):
        """ Get the current laser wavelength

            @return float: the laser wavelength in metres
        """
        return self._get_status('wavelength')[0] * 1e-9

    def set_wavelength(self, target_wavelength):
        """ Set the wavelength of the laser

            @param float target_wavelength : target wavelength to set the laser

            @return int: -1 if error (else no return)
        """
        # self.wavelength_lock = self._set_wavelength_lock('off')  # For use with attached wavelength meter

        self.log.info('M Squared hardware module not configured for use with wavelength meter')

        message = {'transmission_id': [1],
                   'op': 'move_wave_t', # use set_wave_m if using a wavelength meter
                   'parameters': {'wavelength': [target_wavelength * 1e9]
                                  }
                   }

        self._send_command(message)
        time.sleep(0.1)
        response = self._read_response()

        if response['status'][0] == 2:
            self.log.warning('Wavelength out of range')
            return -1
        elif response['status'][0] != 0:
            self.log.warning('An error occured setting the wavelength')
            return -1

        # self.wavelength_lock = self._set_wavelength_lock('on') # For use with attached wavelength meter
        self.wavelength = self.get_wavelength()

########################## internal methods ###################################

    def _get_status(self, param):
        """ Querey the status of the status of the laser and return the requested parameter

            @return various
        """

        message = {'transmission_id': [5],
                   'op': 'get_status'
                  }
        self._send_command(message)
        response = self._read_response()
        return response[param]

    def _get_tuning_status(self):
        """ Check the tuning status of the laser

            @return bool : True if still tuning, False if complete
        """

        message = {'transmission_id': [2],
                   'op': 'poll_move_wave_t'
                   }
        self._send_command(message)
        time.sleep(0.1)
        response = self._read_response()

        if response['status'][0] != 0:
            return True
        else:
            return False

    def _get_wavelength_lock(self):
        """ Get the wavelength lock status

            @return bool : status of the wavelength lock
        """

        message = {'transmission_id':[7], 'op':'poll_wave_m'}

        self._send_command(message)
        time.sleep(0.1)
        response = self._read_response()

        if response['lock_status'][0] == 3:
            return True
        else:
            return False

    def _set_wavelength_lock(self, target_state):
        """ Set the wavelength lock either `on' or `off'

            @param bool targer_state : desired lock state of the laser

            @return bool state : status of the wavelength lock
        """

        message = {'transmission_id':[6], 'op':'lock_wave_m',
                   'parameters':{'operation':target_state}}

        self._send_command(message)
        time.sleep(0.1)
        response = self._read_response()

        if response['status'][0] == 0:
            if target_state == 'on':
                return True
            else:
                return False
        else:
            if target_state == 'on':
                verb = 'apply'
                state = False
            else:
                verb = 'remove'
                state = True

            self.log.error('Unable to {} wavelength lock!'.format(verb))
            return state

    def _get_beam_align(self):
        """ Get the Beam X and Beam Y alignment variables

            @return: float : the Beam X alignment parameter
                     float : the Beam Y alignment parameter
        """

        message = {'transmission_id':[8], 'op':'get_alignment_status'}

        self._send_command(message)
        time.sleep(0.1)
        response = self._read_response()

        return response['x_alignment'][0], response['y_alignment'][0]

    def _set_beam_align(self, beam_x, beam_y):
        """ Set the beam alignment

            @param  float beam_x : the Beam X alignment parameter
                    float beam_y : the Beam Y alignment parameter

            @return: float : the Beam X alignment parameter
                     float : the Beam Y alignment parameter
        """

        self._set_beam_align_x(beam_x)
        self._set_beam_align_y(beam_y)

        return self._get_beam_align()

    def _set_beam_align_x(self, beam_x):
        """ Set the beam x alignment parameter

            @param beam_x float : new beam x alignment param
        """

        message = {'transmission_id': [9],
                    'op': 'beam_adjust_x',
                    'parameters': {'x_value': [beam_x]
                                    }
            }

        self._send_command(message)
        time.sleep(0.1)
        response = self._read_response()

        if response['status'][0] == 3:
            self.log.error('Beam x alignment operation failed - not in manual mode')
        elif response['status'][0] != 0:
            self.log.error('An error has occured setting the Beam X parameter')
        else:
            self._beam_align_x = beam_x  # only update if operation completed

    def _set_beam_align_y(self, beam_y):
        """ Set the beam y alignment parameter

            @param beam_y float : new beam y alignment param
        """

        message = {'transmission_id': [10],
                    'op': 'beam_adjust_y',
                    'parameters': {'y_value': [beam_y]
                                    }
            }

        self._send_command(message)
        time.sleep(0.1)
        response = self._read_response()

        if response['status'][0] == 3:
            self.log.error('Beam y alignment operation failed - not in manual mode')
        elif response['status'][0] != 0:
            self.log.error('An error has occured setting the Beam Y parameter')
        else:
            self._beam_align_y = beam_y  # only update if operation completed

    def _get_beam_align_mode(self):
        """ Get the beam alignment mode

            @return str mode : the beam alignment mode
        """

        message = {'transmission_id':[12], 'op':'get_alignment_status'}

        self._send_command(message)
        time.sleep(0.1)
        response = self._read_response()

        return response['condition']

    def _set_beam_align_mode(self, mode):
        """ Set the beam alignment mode

            @params mode str : 'manual'
                               'automatic'
                               'hold' (stop and hold current values)
                               'one shot'

            @return mode str : the new alignment mode
                    err  int : -1 if error
        """

        if mode == 'manual':
            new_mode = 1
        elif mode == 'automatic':
            new_mode = 2
        elif mode == 'hold':
            new_mode = 3
        elif mode == 'one shot':
            new_mode = 4
            self.log.info('Alignment mode should switch back to Manual once One Shot optimisation is complete')
        else:
            self.log.error('Invalid mode selected')
            return -1
        
        message = {'transmission_id': [11],
                    'op': 'beam_alignment',
                    'parameters': {'mode': [new_mode],
                                    }
            }

        self._send_command(message)
        time.sleep(0.1)
        response = self._read_response()

        if response['status'][0] != 0:
            self.log.error('Could not set the SolsTiS alignment mode to {}'.format(mode))

        return self._get_beam_align_mode()

########################## instrument communication methods ###################

    def _connect(self, ipaddy, port):
        """ Establish a connection to the laser

            @param ipaddy str : laser ip address
                   port int   : laser port

            @return int: 0 if ok, -1 if error
        """

        try:
            self.s.settimeout(5)
            self.s.connect((ipaddy, port))
            myIP = self.s.getsockname()[0]
            message = {"transmission_id": [3],
                       "op": "start_link",
                       "parameters": {"ip_address": myIP}
                       }
            self._send_command(message)
            response = self._read_response()
            if response['status'] == 'ok':
                return 0
            else:
                self.log.error('I cannot connect to the M Squared laser - check config ip and port')
                # these can be found in the SolsTiS config web interface
                return -1
        except:
            return -2
        # TODO: if the laser is connected, check for a protocol error:
        # print('protocol error: ', response['protocol_error'])

    def _ping(self):
        """ Ping the laser

            @return int: 0 if ok, -1 if error
        """

        try:
            message = {'transmission_id' : [4], 'op':'ping',
                   'parameters':{'text_in':'TESTING'}}

            self._send_command(message)
            time.sleep(0.1)
            response = self._read_response()

            if response['text_out'] == 'testing':
                return 0
            else:
                return -1
        except:
            return -1

    def _send_command(self, message):
        """ Send a command to the laser

        @param message str : command string to be delivered to the laser

        @return int: 0 if ok, -1 if error
        """

        self.s.sendall(json.dumps({"message": message}).encode())
        return 0

    def _read_response(self):
        """ Read the response of a query sent to the laser.

        @return response str : query response
        """

        response = self.s.recv(1024)
        response = json.loads(response)
        return response["message"]["parameters"]
