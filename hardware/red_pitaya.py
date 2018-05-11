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

from thirdparty.redpitaya import redpitaya_scpi as scpi

from core.module import Base, ConfigOption
from interface.confocal_scanner_interface import ConfocalScannerInterface


class RedPitaya(Base, GenScannerInterface):
    """ unstable: Matt Joliffe

    A Red Pitaya device that can do lots of things.

    An example config file entry 0would look like:

    ```
    # red_pitaya:
    #     module.Class: 'red_pitaya.RedPitaya'
    #     ip_address: '1.1.1.1'
    #     scanner_ao_channels:
    #         - 'OUT1'
    #         - 'OUT2'
    #     scanner_voltage_ranges:
    #         - [-1, 1]
    #         - [-1, 1]
    #     trigger_out_channel: 'DIO1_P'
    #     scanner_frequency:
    #         - 100
    ```

    """

    _modtype = 'RPcard'
    _modclass = 'hardware'

    _ip = ConfigOption('ip_address', missing='error')
    _scanner_ao_channels = ConfigOption('scanner_ao_channels', missing='error')
    _scanner_voltage_ranges = ConfigOption('scanner_voltage_ranges', missing='error')
    _scanner_frequency = ConfigOption('scanner_frequency', missing='error')
    _trigger_out_channel = ConfigOption(trigger_out_channel, missing='error')

    def on_activate(self):
        """ Starts up the RP Card at activation.
        """

        self.rp_s = scpi.scpi(self._ip)

        rp_s.tx_txt('ACQ:BUF:SIZE?')
        self._buffer_size = int(rp_s.rx_txt())

        # handle all the parameters given by the config
        self._current_position = np.zeros(len(self._scanner_ao_channels))

        self._scanner_position_ranges = self.set_position_range()

        if len(self._scanner_ao_channels) != len(self._scanner_voltage_ranges):
            self.log.error(
                'Specify as many scanner_voltage_ranges as scanner_ao_channels!')

    def on_deactivate(self):
        """ Shut down the Red Pitaya.
        """
        self.reset_hardware()
   
    ############################################################################
    # ================ GenerallScannerInterface Commands =======================
    def reset_hardware(self):
        """ Resets the Red Pitaya hardware, so the connection is lost and other
            programs can access it.

        @return int: error code (0:OK, -1:error)
        """
        retval = 0

        try:
            self.rp_s.tx_txt('GEN:RST')
        except:
            self.log.exception('Could not reset RedPitaya device at ' + self._ip
            retval = -1
        return retval

    def get_position_range(self):
        """ Returns the physical range of the scanner.

        @return float [2][2]: array of 4 ranges with an array containing lower
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

        self.log.info('Setting the position range to ', myrange, ' but this property cannot be configured with this device.')

        # TODO: do something here

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
            self._current_position[0] = np.float(x)

        if y is not None:
            if not(self._scanner_position_ranges[1][0] <= y <= self._scanner_position_ranges[1][1]):
                self.log.error('You want to set y out of range: {0:f}.'.format(y))
                return -1
            self._current_position[1] = np.float(y)

        x_volt = str(self._scanner_position_to_volt(self, positions=x))
        y_volt = str(self._scanner_position_to_volt(self, positions=y))

        # then directly write the position to the hardware
        try:
            #write the x,y positions to the Red Pitaya
            self.rp_s.tx_txt('SOUR1:TRAC:DATA:DATA ' + x_volt)
            self.rp_s.tx_txt('SOUR2:TRAC:DATA:DATA ' + y_volt)
            #set the x,y outputs to trigger internally and simultaneously 
            self.rp_s.tx_txt('TRIG:IMM')
            #trigger the output to set position
            self.rp_s.tx_txt('OUTPUT1:STATE ON')

        except:
            self.log.warning('Cound not set position of RP device on ', self._ip)
            return -1
        return 0

    def get_position(self):
        """ Get the current position of the scanner hardware.

        @return float[]: current position in (x, y).
        """
        return self._current_position.tolist()

    def scan_line(self, line_path=None):
        """ Scans a line and return the counts on that line.

        @param float[c][m] line_path: array of c-tuples defining the voltage points
            (m = samples per line)

        @return float[m][n]: m (samples per line) n-channel photon counts per second

        The input array looks for a xy scan of 5x5 points at the position z=-2
        like the following:
            [ [1, 2, 3, 4, 5], [1, 1, 1, 1, 1], [-2, -2, -2, -2] ]
        n is the number of scanner axes, which can vary. Typical values are 2 for galvo scanners,
        3 for xyz scanners and 4 for xyz scanners with a special function on the a axis.
        """

        if not isinstance(line_path, (frozenset, list, set, tuple, np.ndarray, ) ):
            self.log.error('Given line_path list is not array type.')
            return np.array([[-1.]])

        if len(self.x_line) == 0 and len(self.y_line) == 0:
            self._set_up_line(line_path=line_path)

        try:
            #turn digital output on
            self.rp_s.tx_txt('DIG:PIN'+ self._trigger_out_channel+', 1')  
            time.sleep(0.01)
            #turn digital output off
            self.rp_s.tx_txt('DIG:PIN'+ self._trigger_out_channel+', 0')    

            # update the scanner position instance variable
            self._current_position = np.array(line_path[:, -1])
        except:
            self.log.exception('Error while scanning line.')
            return np.array([[-1.]])
        # return values is a rate of counts/s
        return all_data.transpose()

    def scanner_off(self):
        """ Closes the scanner and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """
        a = self._stop_analog_output()

        b = 0
        if len(self._scaner_ai_channels) > 0:  # TODO: what does this mean?
            try:
                self.rp_s.tx_txt('GEN:RST')
            except:
                self.log.exception('Could not close analog on RP device on ', self._ip)
                b = -1

        return -1 if a < 0 or b < 0 or else 0
        
    ############################################################################
    # ======== Private methods for GeneralScannerInterface Commands ===========
    
    def _set_up_line(self, line_path):
        """ Sets up the analog output for scanning a line.

        @param float[c][m] line_path: array of c-tuples defining the voltage points
        (m = samples per line)

        @return int: error code (0:OK, -1:error)
        """

        #Red Pitaya does not like having a line path less than its buffer size
        x_path = np.linspace(line_path[0][0], line_path[0][len(line_path[0])-1], self._buffer_size)
        y_path = np.linspace(line_path[1][0], line_path[1][len(line_path[1])-1], self._buffer_size)

        x_path = _scanner_position_to_volt(positions = x_path)
        y_path = _scanner_position_to_volt(positions = y_path)
        
        try:
            self.x_line = ''
            self.y_line = ''

            for x_val in x_path:
                self.x_line += str(x_val) + ', '
                
            self.x_line = self.x_line[:len(self.x_line)-2]   

            for y_val in y_path:
                self.y_line += str(self.y_line) + ', '

            self.y_line = y_line[:len(self.y_line)-2]

            #resets generator to default settings
            self.rp_s.tx_txt('GEN:RST')

            #set source 1,2 to have an arbitrary input 
            self.rp_s.tx_txt('SOUR1:FUNC ARBITRARY')
            self.rp_s.tx_txt('SOUR2:FUNC ARBITRARY')

            #set the scanner frequencies from the config file
            self.rp_s.tx_txt('SOUR1:FREQ:FIX ' + str(self._scanner_frequency))
            self.rp_s.tx_txt('SOUR2:FREQ:FIX ' + str(self._scanner_frequency))

            #set source 1,2 waveform to our scan values
            self.rp_s.tx_txt('SOUR1:TRAC:DATA:DATA ' + self.x_line)
            self.rp_s.tx_txt('SOUR2:TRAC:DATA:DATA ' + self.y_line)                        

            #set source burst repititions to 1
            self.rp_s.tx_txt('SOUR1:BURS:NCYC 1')
            self.rp_s.tx_txt('SOUR2:BURS:NCYC 1')

            #enable source 1,2 to be triggered (may cause a trigger)
            self.rp_s.tx_txt('OUTPUT1:STATE ON')
            self.rp_s.tx_txt('OUTPUT2:STATE ON')

            #set trigger to be external
            self.rp_s.tx_txt('SOUR1:TRIG:SOUR EXT_PE')
            self.rp_s.tx_txt('SOUR2:TRIG:SOUR EXT_PE')

            #set digital input/output of trigger channel to output
            self.rp_s.tx_txt('DIG:PIN:DIR OUT,'+ self._trigger_out_channel)
            #set digital input/output pin 0_PE to external trigger input
            self.rp_s.tx_txt('DIG:PIN:DIR IN,DIO0_PE')

        return 0
    
    def _scanner_position_to_volt(self, positions=None):
        """ Converts a set of position pixels to acutal voltages.

        @param float[][n] positions: array of n-part tuples defining the pixels

        @return float[][n]: array of n-part tuples of corresponing voltages

        The positions is typically a matrix like
            [[x_values], [y_values]]
            but x, xy is allowed formats.
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

        # create csv text string of voltages from array
        _AONwritten= ''
        for value in voltages:
            _AONwritten += str(value) + ', '
        _AONwritten = _AONwritten[:len(wave_form)-2] #r emove the ", " at the end of the string
        return self._AONwritten

    def _stop_analog_output(self):
        """ Stops the analog output.

        @return int: error code (0:OK, -1:error)
        """
        retval = 0
        try:
            # stop the analog output
            rp_s.tx_txt('OUTPUT1:STATE OFF')
            rp_s.tx_txt('OUTPUT2:STATE OFF')
        except:
            self.log.exception('Error stopping analog output on RP device at ', self._ip)
            retval = -1
        return retval

    # ================ End ConfocalScannerInterface Commands ===================