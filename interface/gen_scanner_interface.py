# -*- coding: utf-8 -*-

"""
This file contains the Qudi Interface file for a general scanner hardware.

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
    def off(self):
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
    def get_position(self):
        """
        Gets the position of the scanner.
        Returns single float value if the device is in set position mode.
        Returns list like [start, stop, step] if the device is in sweep mode.
        Returns list of positions if the device is in list mode.

        @return [float, list]: frequency(s) currently set for this device in Hz
        """
        pass

    @abc.abstractmethod
    def set_position(self, position=None):
        """
        Configures the device for set position-mode and optionally sets frequency.

        @param float position: posittion set in [length unit]

        @return tuple(float, float, str): with the relation
            current position in [length unit]
        """
        pass

    @abc.abstractmethod
    def list_on(self):
        """
        Switches on the list mode scanning.
        Must return AFTER the device is actually running.

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abc.abstractmethod
    def set_list(self, position=None):
        """
        Configures the device for list-mode

        @param list position: list of positions in [length unit]

        @return list: current positions in [length unit]
        """
        pass

    @abc.abstractmethod
    def reset_listpos(self):
        """
        Reset of scanning list mode position to start (first position step)

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abc.abstractmethod
    def sweep_on(self):
        """ Switches on the sweep mode.

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abc.abstractmethod
    def set_sweep(self, start=None, stop=None, step=None):
        """
        Configures the device for sweep-mode and optionally sets position start/stop/step

        @return float, float, float, float, str: current start position in [length unit],
                                                 current stop position in [length unit],
                                                 current position step in [length unit],
                                                 current mode
        """
        pass

    @abc.abstractmethod
    def reset_sweeppos(self):
        """
        Reset of scanner sweep mode position to start (start position)

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abc.abstractmethod
    def set_ext_trigger(self, pol=TriggerEdge.RISING):
        """ Set the external trigger for this device with proper polarization.

        @param TriggerEdge pol: polarisation of the trigger (basically rising edge or falling edge)

        @return object: current trigger polarity [TriggerEdge.RISING, TriggerEdge.FALLING]
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