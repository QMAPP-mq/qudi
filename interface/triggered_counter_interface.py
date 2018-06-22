# -*- coding: utf-8 -*-
"""
This file contains the Qudi Interface file for ODMRCounter.

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


class TriggeredCounterInterface(metaclass=InterfaceMetaclass):
    """ This is the Interface class supplies the controls for a simple ODMR."""

    _modtype = 'ODMRCounterInterface'
    _modclass = 'interface'

    @abc.abstractmethod
    def set_up_histogram(self, counting_channel=None, trigger_channel=None, binwidth=1, n_bins=1):
        """ Configure the triggered counter

        @param int counting_channel: this is the physical channel of the counter
        @param int trigger_channel: this is the physical channel of the trigger
        @param int n_bins: number of bins in each histogram
        @param int binwidth: bin width in seconds

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abc.abstractmethod
    def get_counts(self):
        """ Return the count histogram across the bins.

        The histogram will continue to be filled.

        @return histogram: the count histrogram
        """
        pass

    @abc.abstractmethod
    def reset_histrogram(self):
        """ Reset the count histogram.

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abc.abstractmethod
    def get_channels(self):
        """ Return a list of channel names.

        @return list(str): counting and trigger channels
        """
        pass