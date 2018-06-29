# -*- coding: utf-8 -*-
"""
Interfuse to do confocal scans with spectrometer data rather than APD count rates.

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

import time
import numpy as np

from core.module import Base, Connector, ConfigOption
from interface.confocal_scanner_interface import ConfocalScannerInterface


class TriggerScannerCounterInterfuse(Base, ConfocalScannerInterface):

    """This is the Interface class to define the controls for the simple
    microwave hardware.
    """
    _modclass = 'confocalscannerinterface'
    _modtype = 'hardware'

    # connectors
    trigger_hw = Connector(interface='TriggerInterface')
    general_scannner_hw = Connector(interface='GeneralScannerInterface')
    tcounter_hw =Connector(interface='TriggeredCounterInterface' ) # tcounter for "triggered counter", normal "slow counter" will not work.

    # config options
    #_clock_frequency = ConfigOption('clock_frequency', 100, missing='warn')

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

        # Internal parameters
        self._line_length = None
        self.line_paths = []
        self._histo_dict = {}
    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """

        self._trig_hw = self.trigger_hw()
        self._gen_scan_hw = self.general_scannner_hw()
        self._trig_count_hw = self.tcounter_hw

    def on_deactivate(self):
        self.reset_hardware()

    def reset_hardware(self):
        """ Resets the hardware, so the connection is lost and other programs can access it.

        @return int: error code (0:OK, -1:error)
        """
        self._gen_scan_hw.reset_hardware()
        self.log.warning('Scanning Device will be reset.')
        return 0

    def get_position_range(self):
        """ Returns the physical range of the scanner.
        This is a direct pass-through to the scanner HW.

        @return float [4][2]: array of 4 ranges with an array containing lower and upper limit
        """
        pos_range =  [[0, 0]]*4
        hw_ranges = self._gen_scan_hw.get_position_range()

        for axis, ax_range in enumerate(hw_ranges):
            pos_range[axis] = ax_range 

        return pos_range

    def set_position_range(self, myrange=None):
        """ Sets the physical range of the scanner.
        This is a direct pass-through to the scanner HW

        @param float [4][2] myrange: array of 4 ranges with an array containing lower and upper limit

        @return int: error code (0:OK, -1:error)
        """
        self.log.warning('Cannot set the position range on this hardware.')
        return 0

    def set_voltage_range(self, myrange=None, channel=[0,1]):
        """ Sets the voltage range of the Red Pitaya.
        This is a direct pass-through to the scanner HW

        @param float [2] myrange: array containing lower and upper limit in volts
        @param float [2-4] channel: array containing channels to set the range of. Channels 0-3

        @return int: error code (0:OK, -1:error)
        """
        self._gen_scan_hw.set_voltage_range(myrange, channel)
        return 0

    def set_up_scanner_clock(self, clock_frequency = None, clock_channel = None):
        """ Configures the hardware clock of the NiDAQ card to give the timing.
        This is a direct pass-through to the scanner HW

        @param float clock_frequency: if defined, this sets the frequency of the clock
        @param string clock_channel: if defined, this is the physical channel of the clock

        @return int: error code (0:OK, -1:error)
        """
        # TODO: convert clock_frequency into a class variable that might be useful later on.
        self._histo_dict['clock_frequency'] = clock_frequency
        self._histo_dict['clock_channel'] = clock_channel

        return 0

    def set_up_scanner(self, counter_channel = None, photon_source = None, clock_channel = None, scanner_ao_channels = None):
        """ Configures the actual scanner with a given clock.

        TODO this is not technically required, because the spectrometer scanner does not need clock synchronisation.

        @param string counter_channel: if defined, this is the physical channel of the counter
        @param string photon_source: if defined, this is the physical channel where the photons are to count from
        @param string clock_channel: if defined, this specifies the clock for the counter
        @param string scanner_ao_channels: if defined, this specifies the analoque output channels

        @return int: error code (0:OK, -1:error)
        """
        self.log.warning('ConfocalScannerInterfaceDummy>set_up_scanner')
        return 0

    def get_scanner_axes(self):
        """ Pass through scanner axes. """
        
        return self._gen_scan_hw.get_scanner_axes()

    def get_scanner_count_channels(self):
        """ Returns the list of channels that are recorded while scanning an image.

        @return list(str): channel names

        Most methods calling this might just care about the number of channels.
        """
        return ['apd1']

    def scanner_set_position(self, x = None, y = None, z = None, a = None):
        """Move stage to x, y, z, a (where a is the fourth voltage channel).
        This is a direct pass-through to the scanner HW

        @param float x: postion in x-direction (volts)
        @param float y: postion in y-direction (volts)
        @param float z: postion in z-direction (volts)
        @param float a: postion in a-direction (volts)

        @return int: error code (0:OK, -1:error)
        """

        self._gen_scan_hw.set_position(x=x, y=y, z=z, a=a)
        return 0

    def get_scanner_position(self):
        """ Get the current position of the scanner hardware.

        @return float[]: current position in (x, y, z, a).
        """

        pos =  [0]*4
        hw_pos = self._gen_scan_hw.get_position()

        for axis, ax_pos in enumerate(hw_pos):
            pos[axis] = ax_pos

        return pos

    def set_up_line(self, line_path):
        """ Set the line length
        Nothing else to do here, because the line will be scanned using multiple scanner_set_position calls.

        @param int length: length of the line in pixel

        @return int: error code (0:OK, -1:error)
        """
        self._gen_scan_hw.set_up_line(line_path)
        return 0

    def scan_line(self, line_path=None, pixel_clock=False):
        """ Scans a line and returns the counts on that line.

        @param float[][4] line_path: array of 4-part tuples defining the voltage points
        @param bool pixel_clock: whether we need to output a pixel clock for this line

        @return float[]: the photon counts per second
        """

        #if self.module_state() == 'locked':
        #    self.log.error('A scan_line is already running, close this one first.')
        #    return -1
        #
        #self.module_state.lock()

        if not isinstance( line_path, (frozenset, list, set, tuple, np.ndarray, ) ):
            self.log.error('Given voltage list is no array type.')
            return np.array([-1.])
        self._line_length = len(line_path[0])
        count_data = np.zeros((self._line_length, 1))

        self.line_paths.append(line_path)

        self._gen_scan_hw.scan_line(line_path)
        self._trig_hw.fire_trigger()
        return count_data

    def close_scanner(self):
        """ Closes the scanner and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """

        self._gen_scan_hw.scanner_off()

        return 0

    def close_scanner_clock(self):
        """ Closes the clock and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """
        #self._trig_hw.close_scanner_clock()
        return 0
