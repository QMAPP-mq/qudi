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
import time

from thirdparty.redpitaya import redpitaya_scpi as scpi

from core.module import Base, ConfigOption
from interface.gen_scanner_interface import GenScannerInterface
from interface.trigger_interface import TriggerInterface


class RedPitaya(Base, GenScannerInterface, TriggerInterface):
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
    #     scanner_position_ranges:
    #         - [0, 1e-6]
    #         - [0, 1e-6]
    ```

    """

    _modtype = 'RPcard'
    _modclass = 'hardware'

    _ip = ConfigOption('ip_address', missing='error')
    _scanner_ao_channels = ConfigOption('scanner_ao_channels', missing='error')
    _scanner_voltage_ranges = ConfigOption('scanner_voltage_ranges', missing='error')
    _scanner_frequency = ConfigOption('scanner_frequency', missing='error')
    _trigger_out_channel = ConfigOption('trigger_out_channel', missing='warn')
    _scanner_position_ranges = ConfigOption('scanner_position_ranges', missing='error')

    def on_activate(self):
        """ Starts up the RP Card at activation.
        """
        try:
            self.rp_s = scpi.scpi(self._ip)
        except:
            self.log.error('Could not connect to Red Pitaya '+self._ip)

        self.rp_s.tx_txt('ACQ:BUF:SIZE?')
        self._buffer_size = int(self.rp_s.rx_txt())
        #set digital input/output of trigger channel to output
        self.rp_s.tx_txt('DIG:PIN:DIR OUT,'+ self._trigger_out_channel)

        self.x_path_volt = [0,0]
        self._scan_state = None
        self._scanner_frequency = self._scanner_frequency[0]
        self._pulse_duration = 1/self._scanner_frequency

        # handle all the parameters given by the config

        self._current_position = np.zeros(len(self._scanner_ao_channels))
        self.set_position(x=self._scanner_position_ranges[0][0], y=self._scanner_position_ranges[1][0])

        if len(self._scanner_ao_channels) != len(self._scanner_voltage_ranges):
            self.log.error(
                'Specify as many scanner_voltage_ranges as scanner_ao_channels!')
        if len(self._scanner_ao_channels) != len(self._scanner_position_ranges):
            self.log.error(
                'Specify as many scanner_position_ranges as scanner_ao_channels!')

    def on_deactivate(self):
        """ Shut down the Red Pitaya.
        """
        self.reset_hardware()

    def scanner_on(self):
        pass
   
    ############################################################################
    # ================ GenerallScannerInterface Commands =======================
    def reset_hardware(self):
        """ Resets the Red Pitaya hardware, so the connection is lost and other
            programs can access it.

        @return int: error code (0:OK, -1:error)
        """
        try:
            self.rp_s.tx_txt('GEN:RST')
        except:
            self.log.exception('Could not reset RedPitaya device at ' + self._ip)
            return -1
        return 0

    def get_position_range(self):
        """ Returns the physical range of the scanner.

        @return float [2][2]: array of 4 ranges with an array containing lower
                              and upper limit. The unit of the scan range is
                              meters.
        """
        return self._scanner_position_ranges

    def set_position_range(self, myrange=None):
        """ Sets the physical position ranges. This can't actually be set by the software for RP.

        @param float [2][2] myrange: array of 2 ranges with an array containing
                                     lower and upper limit. The unit of the
                                     scan range is meters.

        @return int: error code (0:OK, -1:error)
        """
        self.log.info('This property cannot be configured with this device.')

        return 0

    def set_position(self, x=None, y=None, z=None, a=None):
        """Move stage to x, y

        @param float x: postion in x-direction (metres)
        @param float y: postion in y-direction (metres)

        @return int: error code (0:OK, -1:error)
        """
        if self.module_state() == 'locked': #TODO: check if this is necessary
            self.log.error('Another scan_line is already running, close this one first.')
            return -1

        if z is not None or a is not None:
            self.log.error('Can only set position in x and y axes')
            return -1
            
        if x is not None:
            if not(self._scanner_position_ranges[0][0] <= x <= self._scanner_position_ranges[0][1]):
                self.log.error('You want to set x out of range: {0:f}.'.format(x))
                return -1
            _is_x_check = 1
            x_volt = self._scanner_position_to_volt(positions=[x], is_x_check=_is_x_check)
            x_volt = str(x_volt[0][0])
            self._current_position[0] = np.float(x)

        if y is not None:
            print('y is not none')
            if not(self._scanner_position_ranges[1][0] <= y <= self._scanner_position_ranges[1][1]):
                self.log.error('You want to set y out of range: {0:f}.'.format(y))
                return -1
            _is_x_check = 0
            y_volt = self._scanner_position_to_volt(positions=[y], is_x_check=_is_x_check)
            y_volt = str(y_volt[0][0])
            self._current_position[1] = np.float(y)

        try:
            if x is not None and y is not None:
                self._red_pitaya_setpos(x=x_volt, y=y_volt)
            elif x is not None:
                self._red_pitaya_setpos(x=x_volt)
            else:
                self._red_pitaya_setpos(y=y_volt)                
            self._scan_state = '_set_pos'
 
            self.rp_s.tx_txt('TRIG:IMM')

            self.rp_s.tx_txt('OUTPUT1:STATE ON')
        except:
            self.log.warning('Cound not set position of RP device on '+ self._ip)
            return -1
        return 0

    def get_position(self):
        """ Get the current position of the scanner hardware.

        @return float[]: current position in (x, y).
        """
        return self._current_position.tolist()

    def scan_line(self, line_path=None):
        """ Scans a line.

        @param float[c][m] line_path: array of c-tuples defining the voltage points
            (m = samples per line)

        @return int: error code (0:OK, -1:error)

        The input array looks for a xy scan of 5x5 points at the position y=1
        like the following:
            [ [1, 2, 3, 4, 5], [1, 1, 1, 1, 1]]
        n is the number of scanner axes, which can vary. Typical values are 2 for galvo scanners,
        """
        if not isinstance(line_path, (frozenset, list, set, tuple, np.ndarray, ) ):
            self.log.error('Given line_path list is not array type.')
            return np.array([[-1.]])

        y_final = line_path[1][len(line_path[1])-1]
        if line_path[1][0] != y_final:
                print('y cheat')
                y_volt = self._scanner_position_to_volt(positions=[y_final], is_x_check=0)
                y_volt = str(y_volt[0][0])

                self.rp_s.tx_txt('SOUR2:FUNC ARBITRARY')
                self.rp_s.tx_txt('SOUR2:TRAC:DATA:DATA ' + y_volt)
                self.rp_s.tx_txt('OUTPUT2:STATE ON')
                self._current_position[1] = np.float(y_final)
                return 0

        if self.x_path_volt[0] != line_path[0][0] or self.x_path_volt[len(self.x_path_volt)-1] != line_path[0][len(line_path[0])-1] or self._scan_state !='_scanner':
            self.set_up_line(line_path=line_path)

        try:
            self.fire_trigger()
            self._scan_state = '_scanner'    

            # update the scanner position instance variable
            self._current_position[0] = np.array(line_path[0][0])
        except:
            self.log.exception('Error while scanning line.')
            return -1
        return 0

    def scanner_off(self):
        """ Closes the scanner and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """

        try:
            self.rp_s.tx_txt('GEN:RST')
        except:
            self.log.exception('Could not close analog on RP device on '+ self._ip)
            return -1

        return 0

    def set_voltage_range(self, myrange=[-1,1],channel=[0,1]):
        for axis in channel:
            if axis > 1:
                self.log.error('Can only set axis 0 or 1 on this device')
                return -1
        if myrange[0] < -1 or myrange[1] > 1:
            self.log.error('Voltage must be between -1 and 1')
            return -1
        if myrange[0] >= myrange[1]:
            self.log.error('Voltage range must go from low to high')
            return -1
        for axis in channel:
            self._scanner_voltage_ranges[axis][0] = myrange[0]
            self._scanner_voltage_ranges[axis][1] = myrange[1]
        self._scan_state = None
        return 0
    
    def get_scanner_axes(self):
        return ['x','y']

    ############################################################################
    # ======== Private methods for GeneralScannerInterface Commands ===========
    
    def set_up_line(self, line_path):
        """ Sets up the analog output for scanning a line.

        @param float[c][m] line_path: array of c-tuples defining the voltage points
        (m = samples per line)

        @return int: error code (0:OK, -1:error)
        """

        #if the scan path varies in y, set the y position to the final value, don't touch x
        #dirty hack to prevent delays from writing positions to RP

        self._is_x_line = 1

        #Red Pitaya does not like having a line path less than its buffer size
        self.x_path_volt = np.linspace(line_path[0][0], line_path[0][len(line_path[0])-1], self._buffer_size)

        x_path = self._scanner_position_to_volt(positions = self.x_path_volt, is_x_check=self._is_x_line)
        try:
            self.x_line = ''

            for x_val in x_path:
                x_val = x_val[0]
                self.x_line += str(x_val) + ', '
                
            self.x_line = self.x_line[:len(self.x_line)-2]   

            if self._scan_state != '_scanner':
                
                #set source 1,2 waveform to our scan values
                self._red_pitaya_scanline_setup()
                self.rp_s.tx_txt('SOUR1:TRAC:DATA:DATA ' + self.x_line) 
                self._red_pitaya_scanline_burstmode()
                self.fire_trigger()
                self.rp_s.tx_txt('OUTPUT1:STATE ON')
            else:
                self.rp_s.tx_txt('SOUR1:TRAC:DATA:DATA ' + self.x_line) 
            time.sleep(5) #debug
        except:        
            self.log.exception('Could not set up scanline on RP device on '+ self._ip)
            return -1

        return 0
    
    def _scanner_position_to_volt(self, is_x_check, positions=None):
        """ Converts a set of position pixels to acutal voltages.

        @param float[][n] positions: array of n-part tuples defining the pixels

        @return float[][n]: array of n-part tuples of corresponing voltages

        The positions is typically a matrix like
            [[x_values], [y_values]]
            but x, xy is allowed formats.
        """

        #if not isinstance(positions, (frozenset, list, set, tuple, np.ndarray, )):
        #    self.log.error('Given position list is no array type.')
        #    return np.array([np.NaN])

        x_check = 0
        if is_x_check ==1:
            x_check = 1
        vlist = []
        for i in (positions):
            vlist.append(
                (self._scanner_voltage_ranges[x_check][1] - self._scanner_voltage_ranges[x_check][0])
                / (self._scanner_position_ranges[x_check][1] - self._scanner_position_ranges[x_check][0])
                * (i - self._scanner_position_ranges[x_check][0])
                + self._scanner_voltage_ranges[x_check][0]
            )
        if x_check ==1:
            vlist = [-x for x in vlist]
        volts = np.vstack(vlist)

        if volts.min() < self._scanner_voltage_ranges[x_check][0] or volts.max() > self._scanner_voltage_ranges[x_check][1]:
            self.log.error(
                'Voltages ({0}, {1}) exceed the limit, the positions have to '
                'be adjusted to stay in the given range.'.format(volts.min(), volts.max()))
            return np.array([np.NaN])
        return volts

    def _red_pitaya_scanline_setup(self):
        #set source 1,2 to have an arbitrary input 
        self.rp_s.tx_txt('SOUR1:FUNC ARBITRARY')

        #set the scanner frequencies from the config file
        self.rp_s.tx_txt('SOUR1:FREQ:FIX ' + str(self._scanner_frequency))

    def _red_pitaya_scanline_burstmode(self):
        #set source burst repititions to 1
        self.rp_s.tx_txt('SOUR1:BURS:NCYC 1')

        #set trigger to be external
        self.rp_s.tx_txt('SOUR1:TRIG:SOUR EXT_PE')

        #set digital input/output of trigger channel to output
        self.rp_s.tx_txt('DIG:PIN:DIR OUT,'+ self._trigger_out_channel)
        #set digital input/output pin 0_PE to external trigger input
        self.rp_s.tx_txt('DIG:PIN:DIR IN,DIO0_PE')

    def _red_pitaya_setpos(self, x=None, y=None):

        #resets generator to default settings
        #self.rp_s.tx_txt('GEN:RST')

        #set source 1,2 to have an arbitrary input 
        self.rp_s.tx_txt('SOUR1:FUNC ARBITRARY')
        self.rp_s.tx_txt('SOUR2:FUNC ARBITRARY')

        #set the scanner frequencies from the config file
        self.rp_s.tx_txt('SOUR1:FREQ:FIX ' + str(self._scanner_frequency))
        self.rp_s.tx_txt('SOUR2:FREQ:FIX ' + str(self._scanner_frequency))
        
        #set source 1,2 waveform to our scan values
        if x is not None:
            self.rp_s.tx_txt('SOUR1:TRAC:DATA:DATA ' + str(x))
        if y is not None:
            self.rp_s.tx_txt('SOUR2:TRAC:DATA:DATA ' + str(y))  
        self.rp_s.tx_txt('OUTPUT1:STATE ON')
        self.rp_s.tx_txt('OUTPUT2:STATE ON')
        #set the x,y outputs to trigger internally and simultaneously 
        self.rp_s.tx_txt('TRIG:IMM')
        

    # ================ End ConfocalScannerInterface Commands ===================

    # ================ TriggerInterface Commands ===============================

    def set_pulse_amplitude(self, amplitude):
        self.log.info('Can not set pulse amplitude on Red Pitaya. Amplitude is 3.3 V')

    def set_pulse_duration(self, duration):
        self._pulse_duration = duration
        return 0 

    def fire_trigger(self):

        #turn digital output on
        self.rp_s.tx_txt('DIG:PIN '+ self._trigger_out_channel+', 1')

        #enable source 1 to be triggered(only used in scanning)

        time.sleep(self._pulse_duration)

        #turn digital output off
        self.rp_s.tx_txt('DIG:PIN '+ self._trigger_out_channel+', 0')
        return 0
    # ================ End TriggerInterface Commands ===================