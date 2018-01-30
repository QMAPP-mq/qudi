# -*- coding: utf-8 -*-

"""
This file contains the Qudi Interface for power meters.

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

import abc
from enum import Enum
from core.util.interfaces import InterfaceMetaclass


class PowermeterInterface(metaclass=InterfaceMetaclass):
    """ Define the controls for a powermeter."""

    _modtype = 'PowermeterInterface'
    _modclass = 'interface'

    @abc.abstractmethod
    def get_constraints(self):
        """ Retrieve the hardware constrains from the powermeter device.

        @return dict constraints: object with constraints for the powermeter
        """
        pass

    @abc.abstractmethod
    def get_power(self, _averaging_window=None):
        """ Get a power reading from the device.

        @param _averaging_window: number of samples to average across
                    # TODO this should probably be converted to time for the general interface.

        @return: float: measured power, averaged over averaging window.
        """
        pass

    @abc.abstractmethod
    def get_wavelength(self):
        """ Returns the current wavelength setting of the powermeter.

        @return int: the wavelength in meters
        """
        pass

    @abc.abstractmethod
    def set_wavelength(self, _target_wavelength=None):
        """ Sets the wavelength setting of the powermeter.

        @param int _target_wavelength: wavelength to set the powermeter to (in meters)

        @return int: the wavelength in meters
        """
        pass

    @abc.abstractmethod
    def set_averaging_window(self, target_averaging_window=None):
        """ Set the averaging window of the powermeter.

        @param int target_averaging_window: if defined, time over which to average (in seconds)
        (For the PM100A: 1 sample takes approx. 3ms)

        @return int: the averaging window in seconds
        """
        pass

    @abc.abstractmethod
    def get_averaging_window(self):
        """ Get the averaging window of the powermeter.

        @return int: the averaging window in seconds
        """
        pass
