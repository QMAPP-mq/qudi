# -*- coding: utf-8 -*-

"""
This file contains the Qudi Hardware module Red Pitaya class.

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

import numpy as np
import re

import redpitaya_scpi as scpi

from core.module import Base, ConfigOption
from interface.confocal_scanner_interface import ConfocalScannerInterface


class RedPitaya(Base, GenScannerInterface):
    """ unstable: Matt Joliffe

    A Red Pitaya device that can do lots of things.

    """

    _modtype = 'RPcard'
    _modclass = 'hardware'

    def on_activate(self):
        """ Starts up the NI Card at activation.
        """

        # handle all the parameters given by the config
        self._current_position = np.zeros(len(self._scanner_ao_channels))

        if len(self._scanner_ao_channels) < len(self._scanner_voltage_ranges):
            self.log.error(
                'Specify at least as many scanner_voltage_ranges as scanner_ao_channels!')

        if len(self._scanner_ao_channels) < len(self._scanner_position_ranges):
            self.log.error(
                'Specify at least as many scanner_position_ranges as scanner_ao_channels!')

        if len(self._scanner_counter_channels) + len(self._scanner_ai_channels) < 1:
            self.log.error(
                'Specify at least one counter or analog input channel for the scanner!')

        # Analog output is always needed and it does not interfere with the
        # rest, so start it always and leave it running
        if self._start_analog_output() < 0:
            self.log.error('Failed to start analog output.')
            raise Exception('Failed to start RP Card module due to analog output failure.')

    def on_deactivate(self):
        """ Shut down the NI card.
        """
        self.reset_hardware()
   
    ############################################################################
    # ================ GenerallScannerInterface Commands =======================
    def reset_hardware(self):
        """ Resets the NI hardware, so the connection is lost and other
            programs can access it.

        @return int: error code (0:OK, -1:error)
        """
        retval = 0

        try:
            rp_s.tx_txt('GEN:RST')
        except:
            self.log.exception('Could not reset NI device {0}'.format(device))
            retval = -1
        return retval

    def get_position_range(self):
        """ Returns the physical range of the scanner.

        @return float [4][2]: array of 4 ranges with an array containing lower
                              and upper limit. The unit of the scan range is
                              meters.
        """
        return self._scanner_position_ranges

    def set_position_range(self, myrange=None):
        """ Sets the physical range of the scanner.

        @param float [2][2] myrange: array of 2 ranges with an array containing
                                     lower and upper limit. The unit of the
                                     scan range is meters.

        @return int: error code (0:OK, -1:error)
        """
        if myrange is None:
            myrange = [[0, 1e-6], [0, 1e-6]]

        if not isinstance(myrange, (frozenset, list, set, tuple, np.ndarray, )):
            self.log.error('Given range is no array type.')
            return -1

        if len(myrange) != 4:
            self.log.error(
                'Given range should have dimension 4, but has {0:d} instead.'
                ''.format(len(myrange)))
            return -1

        for pos in myrange:
            if len(pos) != 2:
                self.log.error(
                    'Given range limit {1:d} should have dimension 2, but has {0:d} instead.'
                    ''.format(len(pos), pos))
                return -1
            if pos[0]>pos[1]:
                self.log.error(
                    'Given range limit {0:d} has the wrong order.'.format(pos))
                return -1

        self._scanner_position_ranges = myrange
        return 0

    def set_position(self, x=None, y=None):
        """Move stage to x, y, z, a (where a is the fourth voltage channel).

        #FIXME: No volts
        @param float x: postion in x-direction (volts)
        @param float y: postion in y-direction (volts)

        @return int: error code (0:OK, -1:error)
        """

        if self.module_state() == 'locked':
            self.log.error('Another scan_line is already running, close this one first.')
            return -1

        if x is not None:
            if not(self._scanner_position_ranges[0][0] <= x <= self._scanner_position_ranges[0][1]):
                self.log.error('You want to set x out of range: {0:f}.'.format(x))
                return -1
            self._current_position[0] = str(x)

        if y is not None:
            if not(self._scanner_position_ranges[1][0] <= y <= self._scanner_position_ranges[1][1]):
                self.log.error('You want to set y out of range: {0:f}.'.format(y))
                return -1
            self._current_position[1] = str(y)

        # then directly write the position to the hardware
        try:
            #write the x,y positions to the Red Pitaya
            rp_s.tx_txt('SOUR1:TRAC:DATA:DATA ' + str(x))
            rp_s.tx_txt('SOUR2:TRAC:DATA:DATA ' + str(y))
            #set the x,y outputs to trigger internally and simultaneously 
            rp_s.tx_txt('TRIG:IMM')
            #trigger the output to set position
            rp_s.tx_txt('OUTPUT1:STATE ON')

        except:
            return -1
        return 0

    def get_scanner_position(self):
        """ Get the current position of the scanner hardware.

        @return float[]: current position in (x, y).
        """
        return self._current_position.tolist()

    def scan_line(self, line_path=None):
        """ Scans a line and return the counts on that line.

        @param float[c][m] line_path: array of c-tuples defining the voltage points
            (m = samples per line)
        @param bool pixel_clock: whether we need to output a pixel clock for this line

        @return float[m][n]: m (samples per line) n-channel photon counts per second

        The input array looks for a xy scan of 5x5 points at the position z=-2
        like the following:
            [ [1, 2, 3, 4, 5], [1, 1, 1, 1, 1], [-2, -2, -2, -2] ]
        n is the number of scanner axes, which can vary. Typical values are 2 for galvo scanners,
        3 for xyz scanners and 4 for xyz scanners with a special function on the a axis.
        """
        if len(self._scanner_counter_channels) > 0 and len(self._scanner_counter_daq_tasks) < 1:
            self.log.error('Configured counter is not running, cannot scan a line.')
            return np.array([[-1.]])

        if len(self._scanner_ai_channels) > 0 :
            self.log.error('Configured analog input is not running, cannot scan a line.')
            return -1

        if not isinstance(line_path, (frozenset, list, set, tuple, np.ndarray, ) ):
            self.log.error('Given line_path list is not array type.')
            return np.array([[-1.]])
        try:
            if len(self._scanner_ai_channels) > 0:
                self._analog_data = np.full(
                    (len(self._scanner_ai_channels), self._line_length + 1),
                    222,
                    dtype=np.float64)

                #turn digital output on
                rp_s.tx_txt('DIG:PIN DIO5_P, 1')
                time.sleep(0.01)
                #turn digital output off
                rp_s.tx_txt('DIG:PIN DIO5_P, 0')    
                )

            # stop the analog output task
            self._stop_analog_output()

            # create a new array for the final data (this time of the length
            # number of samples):
            self._real_data = np.empty(
                (len(self._scanner_counter_channels), self._line_length),
                dtype=np.uint32)

            # add up adjoint pixels to also get the counts from the low time of
            # the clock:
            self._real_data = self._scan_data[:, ::2]
            self._real_data += self._scan_data[:, 1::2]

            all_data = np.full(
                (len(self.get_scanner_count_channels()), self._line_length), 2, dtype=np.float64)
            all_data[0:len(self._real_data)] = np.array(
                self._real_data * self._scanner_clock_frequency, np.float64)

            if len(self._scanner_ai_channels) > 0:
                all_data[len(self._scanner_counter_channels):] = self._analog_data[:, :-1]

            # update the scanner position instance variable
            self._current_position = np.array(line_path[:, -1])
        except:
            self.log.exception('Error while scanning line.')
            return np.array([[-1.]])
        # return values is a rate of counts/s
        return all_data.transpose()

    def close_scanner(self):
        """ Closes the scanner and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """
        a = self._stop_analog_output()

        b = 0
        if len(self._scanner_ai_channels) > 0:
            try:
                rp_s.tx_txt('GEN:RST')
            except:
                self.log.exception('Could not close analog.')
                b = -1

        return -1 if a < 0 or b < 0 or else 0
        
    ############################################################################
    # ======== Private methods for GeneralScannerInterface Commands ===========
    
    def _set_up_line(self, length=100):
        """ Sets up the analog output for scanning a line.

        @param int length: length of the line in pixel

        @return int: error code (0:OK, -1:error)
        """
        if len(self._scanner_counter_channels) > 0 and len(self._scanner_counter_daq_tasks) < 1:
            self.log.error('Configured counter is not running, cannot scan a line.')
            return np.array([[-1.]])

        if len(self._scanner_ai_channels) > 0:
            self.log.error('Configured analog input is not running, cannot scan a line.')
            return -1

        self._line_length = length

        if  length < np.inf:
            self.log.exception('Error while setting up scanner to scan a line.')
            return -1

        try:
            x_value = ''
            y_value = ''

            for x_val in line_path[0]:
                x_value += str(x_val) + ', '
                
            x_value = x_value[:len(x_value)-2]   

            for y_val in line_path[1]:
                y_value += str(y_val) + ', '

            y_value = y_value[:len(y_value)-2]

            freq_x = 20
            freq_y = 1000

            #resets generator to default settings
            rp_s.tx_txt('GEN:RST')

            #set source 1,2 to have an arbitrary input 
            rp_s.tx_txt('SOUR1:FUNC ARBITRARY')
            rp_s.tx_txt('SOUR2:FUNC ARBITRARY')

            rp_s.tx_txt('SOUR1:FREQ:FIX ' + str(freq_x))
            rp_s.tx_txt('SOUR2:FREQ:FIX ' + str(freq_y))

            #set source 1,2 waveform to our scan values
            rp_s.tx_txt('SOUR1:TRAC:DATA:DATA ' + x_value)
            rp_s.tx_txt('SOUR2:TRAC:DATA:DATA ' + y_value)                        

            #set source burst repititions to 1
            rp_s.tx_txt('SOUR1:BURS:NCYC 1')
            rp_s.tx_txt('SOUR2:BURS:NCYC 1')

            #enable source 1,2 to be triggered (may cause a trigger)
            rp_s.tx_txt('OUTPUT1:STATE ON')
            rp_s.tx_txt('OUTPUT2:STATE ON')

            #set trigger to be external
            rp_s.tx_txt('SOUR1:TRIG:SOUR EXT_PE')
            rp_s.tx_txt('SOUR2:TRIG:SOUR EXT_PE')

            #set digital input/output pin 5_P to output
            rp_s.tx_txt('DIG:PIN:DIR OUT,DIO5_P')
            #set digital input/output pin 0_PE to external trigger input
            rp_s.tx_txt('DIG:PIN:DIR IN,DIO0_PE')

        return 0
    
    def _scanner_position_to_volt(self, positions=None):
        """ Converts a set of position pixels to acutal voltages.

        @param float[][n] positions: array of n-part tuples defining the pixels

        @return float[][n]: array of n-part tuples of corresponing voltages

        The positions is typically a matrix like
            [[x_values], [y_values], [z_values], [a_values]]
            but x, xy, xyz and xyza are allowed formats.
        """

        if not isinstance(positions, (frozenset, list, set, tuple, np.ndarray, )):
            self.log.error('Given position list is no array type.')
            return np.array([np.NaN])

        vlist = []
        for i, position in enumerate(positions):
            vlist.append(
                (self._scanner_voltage_ranges[i][1] - self._scanner_voltage_ranges[i][0])
                / (self._scanner_position_ranges[i][1] - self._scanner_position_ranges[i][0])
                * (position - self._scanner_position_ranges[i][0])
                + self._scanner_voltage_ranges[i][0]
            )
        volts = np.vstack(vlist)

        for i, v in enumerate(volts):
            if v.min() < self._scanner_voltage_ranges[i][0] or v.max() > self._scanner_voltage_ranges[i][1]:
                self.log.error(
                    'Voltages ({0}, {1}) exceed the limit, the positions have to '
                    'be adjusted to stay in the given range.'.format(v.min(), v.max()))
                return np.array([np.NaN])
        return volts

    def _write_scanner_ao(self, voltages, length=1, start=False):
        """Writes a set of voltages to the analog outputs.

        @param float[][n] voltages: array of n-part tuples defining the voltage
                                    points
        @param int length: number of tuples to write
        @param bool start: write imediately (True)
                           or wait for start of task (False)

        n depends on how many channels are configured for analog output
        """

        # create csv text string of voltages from array array
        _AONwritten= ''
        for value in voltages:
            _AONwritten += str(value) + ', '
        _AONwritten = _AONwritten[:len(wave_form)-2] #remove the ", " at the end of the string
        return self._AONwritten

    

    # ================ End ConfocalScannerInterface Commands ===================