# -*- coding: utf-8 -*-
"""
This module controlls an M Squares laser (SolsTiS head).

NOTE: It is important that the computer connecting to the SolsTiS
has the same IP address as configured on the SolsTiS under
Network Settings -> Remote Interface

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

from core.module import Base
from interface.simple_laser_interface import SimpleLaserInterface
from interface.simple_laser_interface import LaserState
from interface.simple_laser_interface import ShutterState
from interface.simple_laser_interface import ControlMode

class MSquaredLaser(Base, SimpleLaserInterface):

    """ Matt van Breugel, Lachlan J. Rogers
    M Squared ultra narrow linewidth, Ti:Sapphire laser
    """
    _modclass = 'msquaredlaser'
    _modtype = 'hardware'

    laser_ip = '192.168.1.222'  # TODO: pass this from config file
    laser_port = 39933  # TODO pass this from config file

    _ipaddr = laser_ip
    _port = laser_port

    def __init__(self, **kwargs):
        """ """
        super().__init__(**kwargs)
        self.lstate = LaserState.OFF
        self.shutter = ShutterState.CLOSED
        self.mode = ControlMode.POWER
        self.current_setpoint = 0
        self.power_setpoint = 0

        self.wavelength = None
        self.wavelength_lock = False

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def on_activate(self):
        """ Activate module.
        """
        try:
            err = self._connect(self._ipaddr, self._port)
            if err == 0:
                self.log.info('Connected to M-Squared hardware')
                self.wavelength = self.get_wavelength()
                # self.wavelength_lock = self._set_wavelength_lock('on')
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
        """ # Not support by this laser
        return (0, 0.250)

    def get_power(self):
        """ Return laser power

            @return float: Laser power in watts
        """ # Not support by this laser
        return self.power_setpoint * random.gauss(1, 0.01)

    def get_power_setpoint(self):
        """ Return optical power setpoint.

            @return float: power setpoint in watts
        """ # Not support by this laser
        return self.power_setpoint

    def set_power(self, power):
        """ Set power setpoint.

            @param float power: power setpoint

            @return float: actual new power setpoint
        """ # Not support by this laser
        self.power_setpoint = power
        self.current_setpoint = math.sqrt(4 * self.power_setpoint) * 100
        return self.power_setpoint

    def get_current_unit(self):
        """ Get unit for laser current.

            @return str: unit
        """ # Not support by this laser
        return '%'

    def get_current_range(self):
        """ Get laser current range.

            @return (float, float): laser current range
        """ # Not support by this laser
        return (0, 100)

    def get_current(self):
        """ Get current laser current

            @return float: laser current in current curent units
        """ # Not support by this laser
        return self.current_setpoint * random.gauss(1, 0.05)

    def get_current_setpoint(self):
        """ Get laser curent setpoint

            @return float: laser current setpoint
        """ # Not support by this laser
        return self.current_setpoint

    def set_current(self, current):
        """ Set laser current setpoint

            @prarm float current: desired laser current setpoint

            @return float: actual laser current setpoint
        """ # Not support by this laser
        self.current_setpoint = current
        self.power_setpoint = math.pow(self.current_setpoint / 100, 2) / 4
        return self.current_setpoint

    def allowed_control_modes(self):
        """ Get supported control modes

            @return list(): list of supported ControlMode
        """ # Not support by this laser
        return [ControlMode.POWER, ControlMode.CURRENT]

    def get_control_mode(self):
        """ Get the currently active control mode

            @return ControlMode: active control mode
        """ # Not support by this laser
        return self.mode

    def set_control_mode(self, control_mode):
        """ Set the active control mode

            @param ControlMode control_mode: desired control mode

            @return ControlMode: actual active ControlMode
        """ # Not support by this laser
        self.mode = control_mode
        return self.mode

    def on(self):
        """ Turn on laser.

            @return LaserState: actual laser state
        """ # Not support by this laser
        time.sleep(1)
        self.lstate = LaserState.ON
        return self.lstate

    def off(self):
        """ Turn off laser.

            @return LaserState: actual laser state
        """ # Not support by this laser
        time.sleep(1)
        self.lstate = LaserState.OFF
        return self.lstate

    def get_laser_state(self):
        """ Get laser state

            @return LaserState: actual laser state
        """ # Not support by this laser
        return self.lstate

    def set_laser_state(self, state):
        """ Set laser state.

            @param LaserState state: desired laser state

            @return LaserState: actual laser state
        """ # Not support by this laser
        time.sleep(1)
        self.lstate = state
        return self.lstate

    def get_shutter_state(self):
        """ Get laser shutter state

            @return ShutterState: actual laser shutter state
        """ # Not support by this laser
        return self.shutter

    def set_shutter_state(self, state):
        """ Set laser shutter state.

            @param ShutterState state: desired laser shutter state

            @return ShutterState: actual laser shutter state
        """ # Not support by this laser
        time.sleep(1)
        self.shutter = state
        return self.shutter

    def get_temperatures(self):
        """ Get all available temperatures.

            @return dict: dict of temperature namce and value in degrees Celsius
        """ # Not support by this laser
        pass

    def set_temperatures(self, temps):
        """ Set temperatures for lasers with tunable temperatures.

            @return {}: empty dict, dummy not a tunable laser
        """ # Not support by this laser
        pass

    def get_temperature_setpoints(self):
        """ Get temperature setpoints.

            @return dict: temperature setpoints for temperature tunable lasers
        """ # Not support by this laser
        pass

    def get_extra_info(self):
        """ Multiple lines of dignostic information

            @return str: much laser, very useful
        """
        return "SolsTiS 3 TCP/IP Protocol"

    def get_wavelength(self):
        """ Get the current laser wavelength

            @return float: the laser wavelength in metres
        """
        return self._get_status('wavelength')[0]

    def set_wavelength(self, target_wavelength):
        """ Set the wavelength of the laser

            @return float: the laser wavelength, or -1 if error
        """
        # self.wavelength_lock = self._set_wavelength_lock('off')

        self.log.info('M Squared hardware module not configured for use with wavelength meter')

        message = {'transmission_id': [3],
                   'op': 'move_wave_t',
                   'parameters': {'wavelength': [target_wavelength],
                                  'report': 'finished'
                                  }
                   }
        self._send_command(message)
        time.sleep(0.1)
        response = self._read_response()

        print(message)
        print(response)

        if response['status'][0] != 0:
            return -1
        self.s.settimeout(300)
        while True:
            time.sleep(0.1)
            response = self._read_response()
            if len(response) == 0:
                continue
            else:
                break
        if 'report' in response:  # because the stop tuning response will be captured here as well
            if response['report'][0] != 0:
                return -1
            else:
                # self.wavelength_lock = self._set_wavelength_lock('on')
                self.wavelength = self.get_wavelength()
                return 0
        else:
            return 'stopped'





        if self.wavelength != target_wavelength:
            self.log.error('Something went wrong, the wavelength was unable to be changed.')
            return -1
        else:
            return 0

    def _connect(self, ipaddr, port):
        """ Establish a connection to the laser

            @return int: 0 if ok, -1 if error
        """
        try:
            self.s.settimeout(5)
            self.s.connect((ipaddr, port))
            myIP = self.s.getsockname()[0]
            message = {"transmission_id": [1],
                       "op": "start_link",
                       "parameters": {"ip_address": myIP}
                       }
            self._send_command(message)
            response = self._read_response()
            # TODO catch this and remind the user to double check IP address in SolsTiS config
            #print('protocol error: ', response['protocol_error'])
            if response['status'] == 'ok':
                return 0
            else:
                return -1
        except:
            return -2

    def _send_command(self, message):
        """ Send a command to the laser

        @return json: message + params
        """
        self.s.sendall(json.dumps({"message": message}).encode())
        return 0

    def _read_response(self):
        response = self.s.recv(1024)
        response = json.loads(response)
        return response["message"]["parameters"]

    def _ping(self):
        """ Ping the laser

            @return int: 0 if ok, -1 if error
        """
        try:
            message = {'transmission_id' : [2], 'op':'ping',
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

    def _get_status(self, param):
        """ Querey the status of the status of the laser and return the requested parameter

            @return various
        """
        message = {'transmission_id': [8],
                   'op': 'get_status'
                  }
        self._send_command(message)
        response = self._read_response()
        return response[param]

    def _set_wavelength_lock(self, target_state):
        """ Set the wavelength lock either `on' or `off'

            @return bool status of the wavelength lock
        """

        message = {'transmission_id':[6], 'op':'lock_wave_m',
                   'parameters':{'operation':target_state}}

        self._send_command(message)
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

    def _get_wavelength_lock(self, target_state):
        """ Get the wavelength lock status

            @return bool status of the wavelength lock
        """
        message = {'transmission_id':[7], 'op':'poll_wave_m'}

        response = self._send_command(message)

        if response['lock_status'][0] == 3:
            return True
        else:
            return False
