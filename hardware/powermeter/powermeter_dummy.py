# -*- coding: utf-8 -*-
"""
This file contains the Qudi hardware for a dummy powermeter.

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
import time

from core.module import Base, ConfigOption
from interface.powermeter_interface import PowermeterInterface
# from ThorlabsPM100 import ThorlabsPM100
# TODO: ensure these imports are on the Qudi install list

import numpy as np

""" List of function available in the Thorlabs PM100 python module:
http://pythonhosted.org/ThorlabsPM100/thorlabsPM100.html
"""


class ThorlabsPM(Base, PowermeterInterface):

    """ unstable: Matt van Breugel
    This is the hardware module for communicating with a Thorlabs power meter
    (PM100x) over USB. It uses the Thorlabs PM100 python module.

    Example configuration
    ```
        pm100a:
            module.Class: 'powermeter.powermeter_thorlabs_pm100.ThorlabsPM'
            serial_number: 'P1000832'
            # averaging_window: 1 # default value for PM100x
            # sampling_time: 3e-3 # default value for PM100x

        pm100d:
            module.Class: 'powermeter.powermeter_thorlabs_pm100.ThorlabsPM'
            serial_number: 'P0006671'
            # averaging_window: 1 # default value for PM100x
            # sampling_time: 3e-3 # default value for PM100x
    ```
    """
    _modclass = 'ThorlabsPM'
    _modtype = 'hardware'

    _wavelength = 532e-9

    _serial_number = ConfigOption('serial_number', None)
    _averaging_window = ConfigOption('averaging_window', 300e-3, missing='warn')  # the default value of a PM100x
    # TODO note that this is not used at the moment
    _sampling_time = ConfigOption('sampling_time', 3e-3, missing='warn')  # 3ms, the expected sampling time of a PM100x

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

    def on_activate(self):
        """ Initialise the hardware module.

            @return int error code (0:OK, -1:error)
        """

        self._connect()
        self.log.info('Connected to dummy powermeter device')
        return 0

    def _connect(self):
        """ Connection to a powermeter device.

        @param object device_list: visa.ResourceManager() object

        @param str device_name : name of the device to connect
        """

        self.constraints = self.get_constraints()  # read the contraints directly from the hardware
        self._wavelength = self.get_wavelength()  # update the class wavelength value

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.

        @return error code (0:OK, -1:error)
        """
        return 0

    def get_constraints(self):
        """ Return a constraints class for the powermeter.

            @return dict constraints : contains PM constraints
        """

        constraints = {}
        constraints['min_wavelength'] = 185 * 1e-9
        constraints['max_wavelength'] = 25 * 1e-6
        constraints['min_sampling_frequency'] = 0
        constraints['max_sampling_frequency'] = 1 / self._sampling_time
        # constraints.threshold = self.ThorlabsPM.sense.peakdetector.maximum_threshold # not working, unsure of usage

        return constraints

    def set_averaging_window(self, target_averaging_window=300e-3):
        """ Set the averaging window of the powermeter.

        @param int target_averaging_window : if defined, time over which to average (in seconds)
                                             (for the PM100x: 1 sample takes approx. 3ms)

        @return int : the averaging window in seconds
        """
        self._averaging_window = target_averaging_window

        return self._averaging_window

    def get_averaging_window(self):
        """ Get the averaging window of the powermeter.

        @return int : the averaging window in seconds
        """
        return self._averaging_window

    def get_power(self, _averaging_window=100):
        """ Return the current power of the powermeter.

        @param int averaging_window : if defined, number of samples over which to average
                                      (1 sample takes approx. 3ms)

        @return float : the power in Watts
        """

        # TODO: Check if this following line is related to issue #8
        # self.ThorlabsPM.sense.average.count = self._averaging_window  # removing as per qmapp issue #8
        # self.ThorlabsPM.sense.average.count  # get the current averaging window
        # self.ThorlabsPM.read  # get the current power reading

        ''' not working, unsure of threshold usage
        if self.ThorlabsPM.read >= self.constraints.threshold:
            self.ThorlabsPM.system.beeper.immediate() # Issue an audible signal (Thorlabs PM100A not very loud)
            self.log.warning('Power is above maximum detector threshold.')
        '''
        mean_signal = 10
        noise_amplitude = 2
        return np.random.normal(mean_signal, noise_amplitude / 2)

    def get_wavelength(self):
        """ Return the current wavelength setting of the powermeter.

        @return float : the wavelength in meters
        """

        self._wavelength = self._wavelength

        return self._wavelength

    def set_wavelength(self, _target_wavelength):
        """ Set the wavelength setting of the powermeter.

        @param int _target_wavelength : wavelength to set the powermeter to (in meters)

        @return float : the wavelength in meters
        """

        if (_target_wavelength > self.constraints['max_wavelength']) or (_target_wavelength < self.constraints['min_wavelength']):
            self.log.error('Target wavelength is outside the constraints, I can not go to that wavelength.')
        else:
            self._wavelength = _target_wavelength

        return self._wavelength
