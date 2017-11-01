# -*- coding: utf-8 -*-
"""
This module controlls an M Squares laser (SolsTiS head).

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

from core.module import Base
from interface.simple_laser_interface import SimpleLaserInterface
from interface.simple_laser_interface import LaserState
from interface.simple_laser_interface import ShutterState
from interface.simple_laser_interface import ControlMode
import math
import random
import time

class MSquaredLaser(Base, SimpleLaserInterface):
    """
    M Squared ultra narrow linewidth, Ti:Sapphire laser
    """
    _modclass = 'msquaredlaser'
    _modtype = 'hardware'

    laser_ip = '192.168.1.222' # TODO: pass this from config file
    laser_port = 39933 # TODO pass this from config file
    
    _ipaddy = ip
    _port = port

    _target_wavelength = 785.0e-9 # TODO: pass this from interface

    def __init__(self, **kwargs):
        """ """
        super().__init__(**kwargs)
        self.lstate = LaserState.OFF
        self.shutter = ShutterState.CLOSED
        self.mode = ControlMode.POWER
        self.current_setpoint = 0
        self.power_setpoint = 0

        self.wavelength
        self.wavelength_lock

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def on_activate(self):
        """ Activate module.
        """
        try:
            _connect(_ip, _port)
        else:
            pass # TODO: throw a 'Could not connect to the laser' error

    def on_deactivate(self):
        """ Deactivate module.
        """
        self.s.close() # close the connection

    def get_power_range(self):
        """ Return optical power range

            @return (float, float): power range
        """
        return (0, 0.250)

    def get_power(self):
        """ Return laser power

            @return float: Laser power in watts
        """
        return self.power_setpoint * random.gauss(1, 0.01)

    def get_power_setpoint(self):
        """ Return optical power setpoint.

            @return float: power setpoint in watts
        """
        return self.power_setpoint

    def set_power(self, power):
        """ Set power setpoint.

            @param float power: power setpoint

            @return float: actual new power setpoint
        """
        self.power_setpoint = power
        self.current_setpoint = math.sqrt(4*self.power_setpoint)*100
        return self.power_setpoint

    def get_current_unit(self):
        """ Get unit for laser current.

            @return str: unit
        """
        return '%'

    def get_current_range(self):
        """ Get laser current range.

            @return (float, float): laser current range
        """
        return (0, 100)

    def get_current(self):
        """ Get current laser current

            @return float: laser current in current curent units
        """
        return self.current_setpoint * random.gauss(1, 0.05)

    def get_current_setpoint(self):
        """ Get laser curent setpoint

            @return float: laser current setpoint
        """
        return self.current_setpoint

    def set_current(self, current):
        """ Set laser current setpoint

            @prarm float current: desired laser current setpoint

            @return float: actual laser current setpoint
        """
        self.current_setpoint = current
        self.power_setpoint = math.pow(self.current_setpoint/100, 2) / 4
        return self.current_setpoint

    def allowed_control_modes(self):
        """ Get supported control modes

            @return list(): list of supported ControlMode
        """
        return [ControlMode.POWER, ControlMode.CURRENT]

    def get_control_mode(self):
        """ Get the currently active control mode

            @return ControlMode: active control mode
        """
        return self.mode

    def set_control_mode(self, control_mode):
        """ Set the active control mode

            @param ControlMode control_mode: desired control mode

            @return ControlMode: actual active ControlMode
        """
        self.mode = control_mode
        return self.mode

    def on(self):
        """ Turn on laser.

            @return LaserState: actual laser state
        """
        time.sleep(1)
        self.lstate = LaserState.ON
        return self.lstate

    def off(self):
        """ Turn off laser.

            @return LaserState: actual laser state
        """
        time.sleep(1)
        self.lstate = LaserState.OFF
        return self.lstate

    def get_laser_state(self):
        """ Get laser state

            @return LaserState: actual laser state
        """
        return self.lstate

    def set_laser_state(self, state):
        """ Set laser state.

            @param LaserState state: desired laser state

            @return LaserState: actual laser state
        """
        time.sleep(1)
        self.lstate = state
        return self.lstate

    def get_shutter_state(self):
        """ Get laser shutter state

            @return ShutterState: actual laser shutter state
        """
        return self.shutter

    def set_shutter_state(self, state):
        """ Set laser shutter state.

            @param ShutterState state: desired laser shutter state

            @return ShutterState: actual laser shutter state
        """
        time.sleep(1)
        self.shutter = state
        return self.shutter

    def get_temperatures(self):
        """ Get all available temperatures.

            @return dict: dict of temperature namce and value in degrees Celsius
        """
        pass

    def set_temperatures(self, temps):
        """ Set temperatures for lasers with tunable temperatures.

            @return {}: empty dict, dummy not a tunable laser
        """
        pass

    def get_temperature_setpoints(self):
        """ Get temperature setpoints.

            @return dict: temperature setpoints for temperature tunable lasers
        """
        pass

    def get_extra_info(self):
        """ Multiple lines of dignostic information

            @return str: much laser, very useful
        """
        return "Dummy laser v0.9.9\nnot used very much\nvery cheap price very good quality"

    def get_wavelength(self):
        """ Get the current laser wavelength

            @return float: the laser wavelength in metres
        """
        self.wavelength = _get_status('wavelength')

    def set_wavelength(self, target_wavelength):
        """ Set the wavelength of the laser

            @return float: the laser wavelength, or -1 if error
        """
        self.wavelength_lock = _lock_wavelength(self, 'off')

        message = {'transmission_id' : [3], 'op':'set_wave_m', 
               'parameters':{'wavelength': [target_wavelength],
               'report':'finished'}}
        response = self.send(message)
        if response['status'][0] != 0:
            return False
        self.s.settimeout(300)
        while True:
            time.sleep(0.1)
            response = self.s.recv(1024)
            response = json.loads(response)
            response = response["message"]["parameters"]
            if  len(response) == 0:
                continue
            else:
                break
        if 'report' in response:   #because the stop tuning response will be captured here as well
            if response['report'][0] != 0:
                return False
            else:
                return True
        else:
            return 'stopped'

        self.wavelength = get_wavelength(self)

        self.wavelength_lock = _lock_wavelength(self, 'on')

        if self.wavelength != target_wavelength:
            return -1
        else:
            return self.wavelength

    def _connect(self, ip_addry, port):
        """ Establish a connection to the laser

            @return int: 0 if ok, -1 if error
        """
        try:
            self.s.settimeout(5)
            self.s.connect((ip_addry,port))
            myIP = self.s.getsockname()[0]
            msg = {"transmission_id" : [1] , "op":"start_link",
                   "parameters":{ "ip_address": myIP}}
            reply = self.send(msg)
            if reply['status'] == 'ok':
                return 0
            else:
                return -1
        except:
            return -1

    def _send_command(self, message):
        """ Send a command to the laser

        @return json: message + params
        """
        self.s.sendall(json.dumps( { "message" : message }))
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
            response = self.send(message)
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
        pass # TODO: implement this function

    def _lock_wavelength(self, target_state):
        """ Set the wavelength lock either `on' or `off'

            @return None
        """
        message = {'transmission_id':[6], 'op':'lock_wave_m',
        'parameters':{'operation':target_state}}
        # return self.send(message)
        # TODO: return lock state