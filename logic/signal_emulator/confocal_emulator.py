# -*- coding: utf-8 -*-
"""
This module operates a confocal microsope.

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

from qtpy import QtCore
from collections import OrderedDict
from copy import copy
import time
import datetime
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from io import BytesIO

from logic.generic_logic import GenericLogic
from core.util.mutex import Mutex
from core.module import Connector, ConfigOption, StatusVar

class ConfocalEmulator(GenericLogic):

    """
    This logic module polls a remote confocal logic and emulates signals locally.
    """

    # declare connectors
    confocalscanner1 = Connector(interface='ConfocalScannerInterface')
    savelogic = Connector(interface='SaveLogic')
    confocallogic = Connector(interface='ConfocalLogic')

    # status vars
    _clock_frequency = StatusVar('clock_frequency', 500)
    return_slowness = StatusVar(default=50)
    max_history_length = StatusVar(default=10)

    # signals
    signal_start_scanning = QtCore.Signal(str)
    signal_continue_scanning = QtCore.Signal(str)
    signal_stop_scanning = QtCore.Signal()
    signal_xy_image_updated = QtCore.Signal()
    signal_depth_image_updated = QtCore.Signal()
    signal_change_position = QtCore.Signal(str)
    signal_xy_data_saved = QtCore.Signal()
    signal_depth_data_saved = QtCore.Signal()
    # signal_tilt_correction_active = QtCore.Signal(bool)
    # signal_tilt_correction_update = QtCore.Signal()
    # signal_draw_figure_completed = QtCore.Signal()

    sigImageXYInitialized = QtCore.Signal()
    sigImageDepthInitialized = QtCore.Signal()

    # signal_history_event = QtCore.Signal()

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

        #locking for thread safety
        self.threadlock = Mutex()

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        self._scanning_device = self.get_connector('confocalscanner1')
        self._save_logic = self.get_connector('savelogic')
        self._confocallogic = self.get_connector('confocallogic')

    def on_deactivate(self):
        """ Reverse steps of activation

        @return int: error code (0:OK, -1:error)
        """
        return 0

########################
# Forward pass-through
# gui -> logic

    def switch_hardware(self, to_on=False):
        """ Switches the Hardware off or on.

        @param to_on: True switches on, False switched off

        @return int: error code (0:OK, -1:error)
        """
        return self._confocallogic.switch_hardware(to_on)

    def set_clock_frequency(self, clock_frequency):
        """Sets the frequency of the clock

        @param int clock_frequency: desired frequency of the clock

        @return int: error code (0:OK, -1:error)
        """
        return self._confocallogic.set_clock_frequency(clock_frequency)

    def start_scanning(self, zscan = False, tag='logic'):
        """Starts scanning

        @param bool zscan: zscan if true, xyscan if false

        @return int: error code (0:OK, -1:error)
        """
        return self._confocallogic.start_scanning(zscan, tag)
        

    def continue_scanning(self,zscan,tag='logic'):
        """Continue scanning

        @return int: error code (0:OK, -1:error)
        """
        return self._confocallogic.continue_scanning(zscan, tag)

    def stop_scanning(self):
        """Stops the scan

        @return int: error code (0:OK, -1:error)
        """
        return self._confocallogic.stop_scanning()

    def initialize_image(self):
        """Initalization of the image.

        @return int: error code (0:OK, -1:error)
        """
        return self._confocallogic.initialize_image()

    def kill_scanner(self):
        """Closing the scanner device.

        @return int: error code (0:OK, -1:error)
        """
        self._confocallogic.kill_scanner()

    def set_position(self, tag, x=None, y=None, z=None, a=None):
        """Forwarding the desired new position from the GUI to the scanning device.

        @param string tag: TODO

        @param float x: if defined, changes to postion in x-direction (microns)
        @param float y: if defined, changes to postion in y-direction (microns)
        @param float z: if defined, changes to postion in z-direction (microns)
        @param float a: if defined, changes to postion in a-direction (microns)

        @return int: error code (0:OK, -1:error)
        """
        return self._confocallogic.set_position(tag, x, y, z, a)

    def get_position(self):
        """ Get position from scanning device.

        @return list: with three entries x, y and z denoting the current
                      position in meters
        """
        return self._confocallogic._scanning_device.get_scanner_position()

    def get_scanner_axes(self):
        """ Get axes from scanning device.
          @return list(str): names of scanner axes
        """
        return self._confocallogic._scanning_device.get_scanner_axes()

    def get_scanner_count_channels(self):
        """ Get lis of counting channels from scanning device.
          @return list(str): names of counter channels
        """
        return self._confocallogic._scanning_device.get_scanner_count_channels()

    def save_xy_data(self, colorscale_range=None, percentile_range=None):
        """ Save the current confocal xy data to file.

        Two files are created.  The first is the imagedata, which has a text-matrix of count values
        corresponding to the pixel matrix of the image.  Only count-values are saved here.

        The second file saves the full raw data with x, y, z, and counts at every pixel.

        A figure is also saved.

        @param: list colorscale_range (optional) The range [min, max] of the display colour scale (for the figure)

        @param: list percentile_range (optional) The percentile range [min, max] of the color scale
        """
        return self._confocallogic.save_xy_data(colorscale_range, percentile_range)

    def save_depth_data(self, colorscale_range=None, percentile_range=None):
        """ Save the current confocal depth data to file.

        Two files are created.  The first is the imagedata, which has a text-matrix of count values
        corresponding to the pixel matrix of the image.  Only count-values are saved here.

        The second file saves the full raw data with x, y, z, and counts at every pixel.
        """
        return self.save_depth_data(colorscale_range, percentile_range)

    ##################################### Tilt correction ########################################

    # @QtCore.Slot()
    # def set_tilt_point1(self):
    #     """ Gets the first reference point for tilt correction."""
    #     self._confocallogic.point1 = np.array(self._confocallogic._scanning_device.get_scanner_position()[:3])
    #     self._confocallogic.signal_tilt_correction_update.emit()

    # @QtCore.Slot()
    # def set_tilt_point2(self):
    #     """ Gets the second reference point for tilt correction."""
    #     self._confocallogic.point2 = np.array(self._confocallogic._scanning_device.get_scanner_position()[:3])
    #     self._confocallogic.signal_tilt_correction_update.emit()

    # @QtCore.Slot()
    # def set_tilt_point3(self):
    #     """Gets the third reference point for tilt correction."""
    #     self._confocallogic.point3 = np.array(self._confocallogic._scanning_device.get_scanner_position()[:3])
    #     self._confocallogic.signal_tilt_correction_update.emit()

    # @QtCore.Slot()
    # def calc_tilt_correction(self):
    #     """ Calculates the values for the tilt correction. """
    #     a = self._confocallogic.point2 - self._confocallogic.point1
    #     b = self._confocallogic.point3 - self._confocallogic.point1
    #     n = np.cross(a, b)
    #     self._confocallogic._scanning_device.tilt_variable_ax = n[0] / n[2]
    #     self._confocallogic._scanning_device.tilt_variable_ay = n[1] / n[2]

    # @QtCore.Slot(bool)
    # def set_tilt_correction(self, enabled):
    #     """ Set tilt correction in tilt interfuse.

    #         @param bool enabled: whether we want to use tilt correction
    #     """
    #     self._confocallogic._scanning_device.tiltcorrection = enabled
    #     self._confocallogic._scanning_device.tilt_reference_x = self._confocallogic._scanning_device.get_scanner_position()[0]
    #     self._confocallogic._scanning_device.tilt_reference_y = self._confocallogic._scanning_device.get_scanner_position()[1]
    #     self._confocallogic.signal_tilt_correction_active.emit(enabled)

    # def history_forward(self):
    #     """ Move forward in confocal image history.
    #     """
    #     if self._confocallogic.history_index < len(self._confocallogic.history) - 1:
    #         self._confocallogic.history_index += 1
    #         self._confocallogic.history[self._confocallogic.history_index].restore(self)
    #         self._confocallogic.signal_xy_image_updated.emit()
    #         self._confocallogic.signal_depth_image_updated.emit()
    #         self._confocallogic.signal_tilt_correction_update.emit()
    #         self._confocallogic.signal_tilt_correction_active.emit(self._confocallogic._scanning_device.tiltcorrection)
    #         self._confocallogic._change_position('history')
    #         self._confocallogic.signal_change_position.emit('history')
    #         self._confocallogic.signal_history_event.emit()

    # def history_back(self):
    #     """ Move backwards in confocal image history.
    #     """
    #     if self._confocallogic.history_index > 0:
    #         self._confocallogic.history_index -= 1
    #         self._confocallogic.history[self._confocallogic.history_index].restore(self)
    #         self._confocallogic.signal_xy_image_updated.emit()
    #         self._confocallogic.signal_depth_image_updated.emit()
    #         self._confocallogic.signal_tilt_correction_update.emit()
    #         self._confocallogic.signal_tilt_correction_active.emit(self._confocallogic._scanning_device.tiltcorrection)
    #         self._confocallogic._change_position('history')
    #         self._confocallogic.signal_change_position.emit('history')
    #         self._confocallogic.signal_history_event.emit()

#######################
# Reverse signal emulation
# logic -> gui
    def poll_loop(self):
        """ Periodically poll the logic, and emit appropriate signals locally if
        things have changed
        """
        self._prev_poll_time = time.time()

        if self._confocallogic.module_state.isstate('locked'):
            # pass through of data arrays
            if not self.module_state.isstate('locked'):
                self.module_state.lock()
                self.signal_start_scanning.emit('logic')
            self.signal_xy_image_updated.emit()
            self.signal_depth_image_updated.emit()
            self.signal_change_position.emit()
        else:
            if not self.module_state.isstate('idle'):
                self.module_state.unlock()
                self.signal_stop_scanning.emit()

        self.signal_change_position.emit()


        while time.time() < self._prev_poll_time + self._polling_interval:
            time.sleep(0.001)
        print('polling complete at {}'.format(time.time()))
        self.start_polling_Signal.emit()

##############
# Emulator specific tools