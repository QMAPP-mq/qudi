# -*- coding: utf-8 -*-

"""
This file contains the Qudi Interface file for general scanner hardware.

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
from core.util.interfaces import InterfaceMetaclass
from core.util.units import in_range
from enum import Enum


class GenScanInterface(metaclass=InterfaceMetaclass):
    """This is the Interface class to define the controls for general scanning hardware.
    """

    _modclass = 'GenScanInterface'
    _modtype = 'interface'

    @abc.abstractmethod
    def reset_hardware(self):
        """
        Switches off any scanner output.
        Must return AFTER the device is actually stopped.

        @return int: error code (0:OK, -1:error)
        """
        pass
    
    @abc.abstractmethod
    def scanner_on(self):
        """
        Switches on the scanner output.
        Must return AFTER the device is actually running.

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abc.abstractmethod
    def scanner_off(self):
        """ Closes the scanner and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """
        pass


    @abc.abstractmethod
    def get_position_range(self):
        """ Returns the physical range of the scanner.

        @return float [4][2]: array of 4 ranges with an array containing lower
                              and upper limit
        """
        pass

    @abc.abstractmethod
    def get_position(self):
        """
        Gets the position of the scanner.
        Returns single float value if the device is in set position mode.
        Returns list like [start, stop, step] if the device is in sweep mode.
        Returns list of positions if the device is in list mode.

        @return [float, list]: positions(s) currently set for this device in metres
        """
        pass

    @abc.abstractmethod
    def set_position(self, position=None):
        """
        Configures the device for set position-mode.

        @param float position: posittion set in metres

        @return tuple(float, float, str): with the relation
            current position in metres
        """
        pass
        @abc.abstractmethod

    def set_position_range(self, myrange=None):
        """ Sets the physical range of the scanner.

        @param float [4][2] myrange: array of 4 ranges with an array containing
                                     lower and upper limit

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abc.abstractmethod
    def scan_line(self):
        """ Switches on the sweep mode.

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abc.abstractmethod
    def set_up_line(self, start=None, stop=None, step=None):
        """
        Configures the device for sweep-mode and optionally sets position start/stop/step

        @return float, float, float, float, str: current start position in metres,
                                                 current stop position in metres,
                                                 current position step in metres,
                                                 current mode
        """
        pass

    def trigger(self):
        """ Trigger the next element in the list or sweep mode programmatically.

        @return int: error code (0:OK, -1:error)

        Ensure that the Frequency was set AFTER the function returns, or give
        the function at least a save waiting time corresponding to the
        frequency switching speed.
        """
        pass

    @abc.abstractmethod
    def get_limits(self):
        """ Return the device-specific limits in a nested dictionary.

          @return GenScannerLimits: General scanner limits object
        """
        pass