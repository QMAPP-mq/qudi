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

from core.module import Base, Connector, ConfigOption
from interface.odmr_counter_interface import ODMRCounterInterface


class TriggerCounterInterfuse(Base, ODMRCounterInterface):
    """This is the Interfuse class to define join a trigger and a triggered counter
    """
    _modclass = 'triggercounterinterfuse'
    _modtype = 'hardware'

    # connectors
    trigger1 = Connector(interface='TriggerInterface')
    tcounter1 = Connector(interface='TriggeredCounterInterface')

    # config options
    _clock_frequency = ConfigOption('clock_frequency', 100, missing='warn')

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """

        self._trig_hw = self.trigger1()
        self._tcounter_hw = self.tcounter1()


    def on_deactivate(self):
        pass

    
    def set_up_odmr_clock(self, clock_frequency=None, clock_channel=None):
        """ Configures the hardware clock of the NiDAQ card to give the timing.

        @param float clock_frequency: if defined, this sets the frequency of the
                                      clock
        @param str clock_channel: if defined, this is the physical channel of
                                  the clock

        @return int: error code (0:OK, -1:error)
        """
        
        if clock_frequency is not None:
            binwidth = 1/ clock_frequency
        else:
            binwidth = 1/ self._clock_frequency

        self._tcounter_hw.set_binwidth(binwidth)
        pass

    
    def set_up_odmr(self, counter_channel=None, photon_source=None,
                    clock_channel=None, odmr_trigger_channel=None):
        """ Configures the actual counter with a given clock.

        @param str counter_channel: if defined, this is the physical channel of
                                    the counter
        @param str photon_source: if defined, this is the physical channel where
                                  the photons are to count from
        @param str clock_channel: if defined, this specifies the clock for the
                                  counter
        @param str odmr_trigger_channel: if defined, this specifies the trigger
                                         output for the microwave

        @return int: error code (0:OK, -1:error)
        """
        pass

    def set_odmr_length(self, length=100):
        """Set up the trigger sequence for the ODMR and the triggered microwave.

        @param int length: length of microwave sweep in pixel

        @return int: error code (0:OK, -1:error)
        """
        pass

    
    def count_odmr(self, length = 100):
        """ Sweeps the microwave and returns the counts on that sweep.

        @param int length: length of microwave sweep in pixel

        @return float[]: the photon counts per second
        """
        # This is where the action happens.

        self._tcounter_hw.set_up_histogram()

        self._trig_hw.fire()

        counts = self._tcounter_hw.get_data()

        return counts

    
    def close_odmr(self):
        """ Close the odmr and clean up afterwards.

        @return int: error code (0:OK, -1:error)
        """
        pass

    
    def close_odmr_clock(self):
        """ Close the odmr and clean up afterwards.

        @return int: error code (0:OK, -1:error)
        """
        pass

    
    def get_odmr_channels(self):
        """ Return a list of channel names.

        @return list(str): channels recorded during ODMR measurement
        """
        pass