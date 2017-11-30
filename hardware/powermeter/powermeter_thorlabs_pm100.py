# -*- coding: utf-8 -*-
"""
This file contains the Qudi hardware for the Thorlabs PM100x powermeter series.

NOTE: To find your Thorlabs PM100x serial number: On the device go to:
System Menue > Consol Info > S/N

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
from ThorlabsPM100 import ThorlabsPM100
# TODO: ensure these imports are on the Qudi install list

""" List of function available in the Thorlabs PM100 python module:
http://pythonhosted.org/ThorlabsPM100/thorlabsPM100.html
"""


class ThorlabsPM(Base, PowermeterInterface):

    """ unstable: Matt van Breugel
    This is the hardware module for communicating with a Thorlabs power meter
    (PM100x) over USB. It uses the Thorlabs PM100 python module.
    """
    _modclass = 'ThorlabsPM'
    _modtype = 'hardware'

    _wavelength = None

    _serial_number = ConfigOption('serial_number', None)
    _averaging_window = ConfigOption('averaging_window', 300e-3, missing='warn')  # the default value of a PM100x
    # TODO note that this is not used at the moment
    _sampling_time = ConfigOption('sampling_time', 3e-3, missing='warn')  # 3ms, the expected sampling time of a PM100x

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

    def on_activate(self):
        """ Initialisation performed during activation of the module.

        @return error code
        """
        # search and connect
        device_list = visa.ResourceManager()

        # simple use case: no S/N in config so there should be just one powermeter connected.
        if self._serial_number is None:
            pm_device = [device for device in device_list.list_resources() if 'PM' in device]

            if len(pm_device) == 1:
                self._connect(device_list, pm_device[0])
                return 0
            elif len(pm_device) > 1:
                self.log.warning('There is more than 1 Thorlabs PM100x connected.'
                                 'You need to specify the serial number in your config file.'
                                 )
                return 1
            else:
                self.log.warning('No Thorlabs PM100x devices found.')

        # More advanced: with S/N in config there can be many PM devices connected
        else:
            pm_devices = [device for device in device_list.list_resources() if self._serial_number in device]

            if len(pm_devices) == 1:
                self._connect(device_list, pm_device[0])
                return 0
            elif len(pm_devices > 1):  # this should never be the case
                self.log.warning('There is more than 1 Thorlabs PM100x connected containig'
                                 'S/N: {}'.format(self._serial_number)
                                 )
                return 1
            else:
                self.log.warning('I cannot find any Thorlabs PM100x connected with the '
                                 'S/N: {}'.format(self._serial_number)
                                 )
                return 1

    def _connect(self, device_list, device_name):
        """ Do the actual connection to a powermeter device

        @param object device_list: visa.ResourceManager() object

        @param string device_name: name of the device to connect
        """
        instance = device_list.open_resource(device_name)
        self.ThorlabsPM = ThorlabsPM100(inst=instance)
        self.constraints = self.get_constraints()  # read the contraints directly from the hardware
        self.ThorlabsPM.display.brightness = 0.01  # dim the display for measurements
        self._wavelength = self.get_wavelength()  # update the class wavelength value

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.

        @return error code
        """
        self.ThorlabsPM.display.brightness = 1  # restore display brightness to full
        self.ThorlabsPM.abort()
        return 0

    def get_constraints(self):
        """ Return a constraints class for the powermeter."""
        constraints = {}
        constraints['min_wavelength'] = self.ThorlabsPM.sense.correction.minimum_wavelength * 1e-9
        constraints['max_wavelength'] = self.ThorlabsPM.sense.correction.maximum_wavelength * 1e-9
        # constraints.threshold = self.ThorlabsPM.sense.peakdetector.maximum_threshold # not working, unsure of usage

        return constraints

    def set_averaging_window(self, target_averaging_window=300e-3):
        """ Set the averaging window of the powermeter.

        @param int target_averaging_window: if defined, time over which to average (in seconds)
        (For the PM100x: 1 sample takes approx. 3ms)

        @return int: the averaging window in seconds
        """

        self.ThorlabsPM.sense.average.count = target_averaging_window / self._sampling_time
        time.sleep(0.1)
        self._averaging_window = self.ThorlabsPM.sense.average.count * self._sampling_time

        return self._averaging_window

    def get_averaging_window(self):
        """ Get the averaging window of the powermeter.

        @return int: the averaging window in seconds
        """

        self._averaging_window = self.ThorlabsPM.sense.average.count * self._sampling_time

        return self._averaging_window

    def get_power(self, _averaging_window=100):
        """ Returns the current power of the powermeter.

        @param int averaging_window: if defined, number of samples over which to average
        (1 sample takes approx. 3ms)

        @return float: the power in Watts
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

        return self.ThorlabsPM.read

    def get_wavelength(self):
        """ Returns the current wavelength setting of the powermeter.

        @return int: the wavelength in meters
        """
        self._wavelength = self.ThorlabsPM.sense.correction.wavelength * 1e-9

        return self._wavelength

    def set_wavelength(self, _target_wavelength):
        """ Sets the wavelength setting of the powermeter.

        @param int _target_wavelength: wavelength to set the powermeter to (in meters)

        @return int: the wavelength in meters
        """

        if (_target_wavelength > self.constraints['max_wavelength']) or (_target_wavelength < self.constraints['min_wavelength']):
            self.ThorlabsPM.system.beeper.immediate()  # Issue an audible signal (Thorlabs PM100A not very loud)
            self.log.error('Target wavelength is outside the constraints, I can not go to that wavelength.')
        else:
            self.ThorlabsPM.sense.correction.wavelength = _target_wavelength / 1e-9
            self._wavelength = self.ThorlabsPM.sense.correction.wavelength * 1e-9

        return self._wavelength
