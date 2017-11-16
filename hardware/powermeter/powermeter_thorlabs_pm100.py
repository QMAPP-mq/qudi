# -*- coding: utf-8 -*-
"""
This file contains the Qudi hardware for the Thorlabs PM100A Powermeter.

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

import visa
from ThorlabsPM100 import ThorlabsPM100
# TODO: ensure these imports are on the Qudi install list

""" List of function available in the Thorlabs PM100 python module:
http://pythonhosted.org/ThorlabsPM100/thorlabsPM100.html
"""

class ThorlabsPM(Base, SlowCounterInterface):

    """ unstable: Matt van Breugel
    This is the hardware module for communicating with a Thorlabs power meter 
    (PM100A) over USB. It uses the Thorlabs PM100 python module.
    """
    _modclass = 'ThorlabsPM'
    _modtype = 'hardware'

    # config
    self. _wavelength = 532e-9 # defult wavelength on startup
    self._averaging_window = 100 # TODO: read from config file

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

    def on_activate(self):
        """ Initialisation performed during activation of the module.

        @return error code
        """
        # search and connect
        device_list = visa.ResourceManager()
        pm_devices = [device for device in device_list.list_resources() if 'P100' in device]

        if len(pm_devices) == 1:
            instance = device_list.open_resource(pm_devices[0])
            self.ThorlabsPM = ThorlabsPM100(inst=instance)
            self.constraints = self.get_constraints() # read the contraints directly from the hardware
            self.ThorlabsPM.display.brightness = 0.01 # dim the display for measurements
            return 0
        elif len(pm_devices > 1):
            self.log.warning('There is more than 1 Thorlabs PM100 connected, I do not know which one to choose.')
            return 1
        else:
            self.log.warning('I cannot find any Thorlabs PM100 connected.')
            return 1

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.

        @return error code
        """
        self.ThorlabsPM.display.brightness = 1 # restore display brightness to full
        self.ThorlabsPM.abort()
        return 0

    def get_constraints(self):
        """ Return a constraints class for the powermeter."""
        constraints = {}
        constraints['min_wavelength'] = self.ThorlabsPM.sense.correction.minimum_wavelength * 1e-9
        constraints['max_wavelength'] = self.ThorlabsPM.sense.correction.maximum_wavelength * 1e-9
        # constraints.threshold = self.ThorlabsPM.sense.peakdetector.maximum_threshold # not working, unsure of usage

        return constraints

    def get_power(self, _averaging_window=100):
        """ Returns the current power of the powermeter.

        @param int averaging_window: if defined, number of samples over which to average
        (1 sample takes approx. 3ms)

        @return float: the power in Watts
        """
        self.ThorlabsPM.sense.average.count = _averaging_window
        # self.ThorlabsPM.sense.average.count  # get the current averaging window
        # self.ThorlabsPM.read  # get the current power reading

        ''' not working, unsure of threshold usage
        if self.ThorlabsPM.read >= self.constraints.threshold:
            self.ThorlabsPM.system.beeper.immediate() # Issue an audible signal (Thorlabs PM100A not very loud)
            self.log.warning('Power is above maximum detector threshold.')
        '''

        return self.ThorlabsPM.read

    def get_wavelength(self):
        """ Returns the current wavelength setting of the powermeter.

        @return int: the wavelength in meters
        """
        self._wavelength = self.ThorlabsPM.sense.correction.wavelength * 1e-9

        return self._wavelength

    def set_wavelength(self, _target_wavelength):
        """ Sets the wavelength setting of the powermeter.

        @param int _target_wavelength: to wavelength to set the powermeter to

        @return int: the wavelength in meters
        """

        if (_target_wavelength > self.constraints['max_wavelength']) or (_target_wavelength < self.constraints['min_wavelength']):
            self.ThorlabsPM.system.beeper.immediate() # Issue an audible signal (Thorlabs PM100A not very loud)
            self.log.error('Target wavelength is outside the constraints, I can not go to that wavelength.')
        else:
            self.ThorlabsPM.sense.correction.wavelength = _target_wavelength / 1e-9
            self._wavelength = self.ThorlabsPM.sense.correction.wavelength * 1e-9

        return self._wavelength
