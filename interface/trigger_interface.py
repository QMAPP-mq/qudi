# -*- coding: utf-8 -*-
"""
This file contains the Qudi Interface file for a trigger.

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


class TriggerInterface(metaclass=InterfaceMetaclass):
    """ This is the Interface class supplies the controls for a simple ODMR."""

    _modtype = 'ODMRCounterInterface'
    _modclass = 'interface'

    @abc.abstractmethod
    def set_pulse_amplitude(self, amplitude, duration):
        pass
    
    def set_pulse_duration(self, amplitude, duration):
        pass

    @abc.abstractmethod
    def fire_trigger(self, channel):
        """ Return a list of channel names.

        @return list(str): channels recorded during ODMR measurement
        """
        pass