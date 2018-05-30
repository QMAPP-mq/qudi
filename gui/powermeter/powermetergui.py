# -*- coding: utf-8 -*-

"""
This file contains the Qudi powermeter gui.

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
import os
import pyqtgraph as pg

from core.module import Connector
from gui.colordefs import QudiPalettePale as palette
from gui.guibase import GUIBase
from qtpy import QtCore
from qtpy import QtWidgets
from qtpy import uic
from qtpy import QtGui



class CounterMainWindow(QtWidgets.QMainWindow):

    """ Create the Main Window based on the *.ui file. """

    def __init__(self, **kwargs):
        # Get the path to the *.ui file
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'ui_powermeter.ui')

        # Load it
        super().__init__(**kwargs)
        uic.loadUi(ui_file, self)
        self.show()


class PowermeterGui(GUIBase):

    """ FIXME: Please document
    """
    _modclass = 'powermetergui'
    _modtype = 'gui'

    # declare connectors
    powerlogic1 = Connector(interface='PowermeterLogic')

    sigStartTrace = QtCore.Signal()
    sigStopTrace = QtCore.Signal()

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

    def on_activate(self):
        """ Definition and initialisation of the GUI.
        """

        self._powermeter_logic = self.get_connector('powerlogic1')

        #####################
        # Configuring the dock widgets
        # Use the inherited class 'CounterMainWindow' to create the GUI window
        self._mw = CounterMainWindow()

        # Setup dock widgets
        self._mw.centralwidget.hide()
        self._mw.setDockNestingEnabled(True)

        # Plot labels.
        self._pw = self._mw.power_trace_PlotWidget

        self._pw.setLabel('left', 'Power', units='W')
        self._pw.setLabel('bottom', 'Time', units='s')

        self.curves = []

        # Raw data curve
        self.curves.append(pg.PlotDataItem(pen=pg.mkPen(palette.c1), symbol=None))
        self._pw.addItem(self.curves[0])

        # Smoothed curve
        self.curves.append(
            pg.PlotDataItem(
                pen=pg.mkPen(palette.c2, style=QtCore.Qt.DotLine),
                symbol='s',
                symbolPen=palette.c2,
                symbolBrush=palette.c2,
                symbolSize=5))
        self._pw.addItem(self.curves[-1])

        # setting the x axis length correctly
        self._pw.setXRange(
            0,
            self._powermeter_logic.trace_length / self._powermeter_logic.sampling_frequency
        )

        #####################
        # Setting default parameters
        self._mw.trace_length_SpinBox.setValue(self._powermeter_logic.trace_length)
        self._mw.sampling_freq_SpinBox.setValue(self._powermeter_logic.sampling_frequency)
        self._mw.wavelength_SpinBox.setValue(self._powermeter_logic.wavelength)

        #####################
        # Connecting user interactions
        self._mw.start_power_Action.triggered.connect(self.start_clicked)
        self._mw.record_counts_Action.triggered.connect(self.save_clicked)

        self._mw.trace_length_SpinBox.valueChanged.connect(self.trace_length_changed)
        self._mw.sampling_freq_SpinBox.valueChanged.connect(self.sampling_frequency_changed)
        self._mw.wavelength_SpinBox.valueChanged.connect(self.wavelength_changed)

        # Connect the default view action
        self._mw.restore_default_view_Action.triggered.connect(self.restore_default_view)

        #####################
        # starting the physical measurement
        self.sigStartTrace.connect(self._powermeter_logic.start_trace)
        self.sigStopTrace.connect(self._powermeter_logic.stopTrace)

        ##################
        # Handling signals from the logic

        self._powermeter_logic.sigTracerUpdated.connect(self.updateData)

        # TODO:
        # self._powermeter_logic.sigCountContinuousNext.connect()
        # self._powermeter_logic.sigCountGatedNext.connect()
        # self._powermeter_logic.sigCountFiniteGatedNext.connect()
        # self._powermeter_logic.sigGatedTracerFinished.connect()
        # self._powermeter_logic.sigGatedTracerContinue.connect()

        self._powermeter_logic.sigTraceLengthChanged.connect(self.update_trace_length_SpinBox)
        self._powermeter_logic.sigSamplingFrequencyChanged.connect(self.update_sampling_freq_SpinBox)
        self._powermeter_logic.sigSavingStatusChanged.connect(self.update_saving_Action)
        self._powermeter_logic.sigTraceStatusChanged.connect(self.update_count_status_Action)

        return 0

    def show(self):
        """Make window visible and put it above all other windows.
        """
        QtWidgets.QMainWindow.show(self._mw)
        self._mw.activateWindow()
        self._mw.raise_()
        return

    def on_deactivate(self):
        # FIXME: !
        """ Deactivate the module
        """
        # disconnect signals
        self._mw.start_power_Action.triggered.disconnect()
        self._mw.record_counts_Action.triggered.disconnect()
        self._mw.trace_length_SpinBox.valueChanged.disconnect()
        self._mw.sampling_freq_SpinBox.valueChanged.disconnect()
        self._mw.restore_default_view_Action.triggered.disconnect()
        self.sigStartTrace.disconnect()
        self.sigStopTrace.disconnect()
        self._powermeter_logic.sigTracerUpdated.disconnect()
        self._powermeter_logic.sigPowerSamplesChanged.disconnect()
        self._powermeter_logic.sigTraceLengthChanged.disconnect()
        self._powermeter_logic.sigSamplingFrequencyChanged.disconnect()
        self._powermeter_logic.sigSavingStatusChanged.disconnect()
        self._powermeter_logic.sigCountingModeChanged.disconnect()
        self._powermeter_logic.sigTraceStatusChanged.disconnect()

        self._mw.close()
        return

    def updateData(self):
        """ The function that grabs the data and sends it to the plot.
        """

        if self._powermeter_logic.module_state() == 'locked':
            self._mw.power_value_Label.setText(
                '{0:,.0f}'.format(self._powermeter_logic.powerdata_smoothed[-1]))

            x_vals = (
                np.arange(0, self._powermeter_logic.trace_length)
                / self._powermeter_logic.sampling_frequency)

            
            self.curves[0].setData(y=self._powermeter_logic.powerdata, x=x_vals)
            self.curves[1].setData(y=self._powermeter_logic.powerdata_smoothed,
                                   x=x_vals
                                  )

        if self._powermeter_logic.get_saving_state():
            self._mw.record_counts_Action.setText('Save')
            self._mw.sampling_freq_SpinBox.setEnabled(False)
        else:
            self._mw.record_counts_Action.setText('Start Saving Data')
            self._mw.sampling_freq_SpinBox.setEnabled(True)

        if self._powermeter_logic.module_state() == 'locked':
            self._mw.start_power_Action.setText('Stop powermeter')
            self._mw.start_power_Action.setChecked(True)
        else:
            self._mw.start_power_Action.setText('Start powermeter')
            self._mw.start_power_Action.setChecked(False)
        return 0

    def start_clicked(self):
        """ Handling the Start button to stop and restart the powermeter.
        """
        if self._powermeter_logic.module_state() == 'locked':
            self._mw.start_power_Action.setText('Start powermeter')
            self.sigStopTrace.emit()
        else:
            self._mw.start_power_Action.setText('Stop powermeter')
            self.sigStartTrace.emit()
        return self._powermeter_logic.module_state()

    def save_clicked(self):
        """ Handling the save button to save the data into a file.
        """
        if self._powermeter_logic.get_saving_state():
            self._mw.record_counts_Action.setText('Start Saving Data')
            self._mw.sampling_freq_SpinBox.setEnabled(True)
            self._powermeter_logic.save_data()
        else:
            self._mw.record_counts_Action.setText('Save')
            self._mw.sampling_freq_SpinBox.setEnabled(False)
            self._powermeter_logic.start_saving()
        return self._powermeter_logic.get_saving_state()

    ########
    # Input parameters changed via GUI

    def trace_length_changed(self):
        """ Handling the change of the count_length and sending it to the measurement.
        """
        self._powermeter_logic.trace_length = int(self._mw.trace_length_SpinBox.value())
        self._pw.setXRange(
            0,
            self._powermeter_logic.trace_length / self._powermeter_logic.sampling_frequency
        )
        return self._mw.trace_length_SpinBox.value()

    def sampling_frequency_changed(self):
        """ Handling the change of the count_frequency and sending it to the measurement.
        """
        self._powermeter_logic.sampling_frequency = self._mw.sampling_freq_SpinBox.value()
        self._pw.setXRange(
            0,
            self._powermeter_logic.trace_length / self._powermeter_logic.sampling_frequency
        )
        return self._mw.sampling_freq_SpinBox.value()

    def wavelength_changed(self):
        """ Handling the change of the wavelength and sending it to the measurement.
        """
        self._powermeter_logic.wavelength = self._mw.wavelength_SpinBox.value()
        # self._pw.setXRange(
        #     0,
        #     self._powermeter_logic.trace_length / self._powermeter_logic.sampling_frequency
        # )
        return self._mw.wavelength_SpinBox.value()

    ########
    # Restore default values

    def restore_default_view(self):
        """ Restore the arrangement of DockWidgets to the default
        """
        # Show any hidden dock widgets
        self._mw.power_trace_DockWidget.show()
        # self._mw.slow_counter_control_DockWidget.show()
        self._mw.powermeter_parameters_DockWidget.show()

        # re-dock any floating dock widgets
        self._mw.power_trace_DockWidget.setFloating(False)
        self._mw.powermeter_parameters_DockWidget.setFloating(False)

        # Arrange docks widgets
        self._mw.addDockWidget(QtCore.Qt.DockWidgetArea(1),
                               self._mw.power_trace_DockWidget
                               )
        self._mw.addDockWidget(QtCore.Qt.DockWidgetArea(8),
                               self._mw.powermeter_parameters_DockWidget
                               )

        # Set the toolbar to its initial top area
        self._mw.addToolBar(QtCore.Qt.TopToolBarArea,
                            self._mw.counting_control_ToolBar)
        return 0

    ##########
    # Handle signals from logic

    def update_sampling_freq_SpinBox(self, count_freq):
        """Function to ensure that the GUI displays the current value of the logic

        @param float count_freq: adjusted count frequency in Hz
        @return float count_freq: see above
        """
        self._mw.sampling_freq_SpinBox.blockSignals(True)
        self._mw.sampling_freq_SpinBox.setValue(count_freq)
        self._pw.setXRange(0, self._powermeter_logic.trace_length / count_freq)
        self._mw.sampling_freq_SpinBox.blockSignals(False)
        return count_freq

    def update_trace_length_SpinBox(self, count_length):
        """Function to ensure that the GUI displays the current value of the logic

        @param int count_length: adjusted count length in bins
        @return int count_length: see above
        """
        self._mw.trace_length_SpinBox.blockSignals(True)
        self._mw.trace_length_SpinBox.setValue(count_length)
        self._pw.setXRange(0, count_length / self._powermeter_logic.sampling_frequency)
        self._mw.trace_length_SpinBox.blockSignals(False)
        return count_length

    def update_wavelength_SpinBox(self, wavelength):
        """Function to ensure that the GUI displays the current value of the logic

        @param float wavelength: adjusted wavelength in nm
        @return int wavelength: see above
        """
        self._mw.wavelength_SpinBox.blockSignals(True)
        self._mw.wavelength.setValue(wavelength)
        # self._pw.setXRange(0, count_length / self._powermeter_logic.sampling_frequency)
        self._mw.wavelength.blockSignals(False)
        return wavelength

    def update_saving_Action(self, start):
        """Function to ensure that the GUI-save_action displays the current status

        @param bool start: True if the measurment saving is started
        @return bool start: see above
        """
        if start:
            self._mw.record_counts_Action.setText('Save')
            self._mw.sampling_freq_SpinBox.setEnabled(False)
        else:
            self._mw.record_counts_Action.setText('Start Saving Data')
            self._mw.sampling_freq_SpinBox.setEnabled(True)
        return start

    def update_count_status_Action(self, running):
        """Function to ensure that the GUI-save_action displays the current status

        @param bool running: True if the counting is started
        @return bool running: see above
        """
        if running:
            self._mw.start_power_Action.setText('Stop powermeter')
        else:
            self._mw.start_power_Action.setText('Start powermeter')
        return running

    # TODO:
    def update_counting_mode_ComboBox(self):
        self.log.warning('Not implemented yet')
        return 0

    # TODO:
    def update_smoothing_ComboBox(self):
        self.log.warning('Not implemented yet')
        return 0

    def _set_wavelength_representor(self, wavelength):
        """ Set the colour of the wavelength representor

            @param float wavelength : in meters
        """

        self._mw.wavelength_representor.setAutoFillBackground(True)

        palette = self._mw.wavelength_representor.palette()

        r, g, b = self._wavelength_to_rgb(wavelength)

        palette.setColor(self._mw.wavelength_representor.backgroundRole(), QtGui.QColor(r, g, b))

        self._mw.wavelength_representor.setPalette(palette)

    def _wavelength_to_rgb(self, wavelength, gamma=0.8):

        '''This converts a given wavelength of light to an 
        approximate RGB color value. The wavelength must be given
        in nanometers in the range from 380 nm through 750 nm
        (789 THz through 400 THz).

        Based on code by Dan Bruton
        http://www.physics.sfasu.edu/astro/color/spectra.html
        
            @param wavelength : input wavelenght in meters
            
            @returns tuple : (red value, green value, blue value)
        '''

        wavelength = wavelength / 1e-9
        
        wavelength = float(wavelength)
        if wavelength >= 380 and wavelength <= 440:
            attenuation = 0.3 + 0.7 * (wavelength - 380) / (440 - 380)
            R = ((-(wavelength - 440) / (440 - 380)) * attenuation) ** gamma
            G = 0.0
            B = (1.0 * attenuation) ** gamma
        elif wavelength >= 440 and wavelength <= 490:
            R = 0.0
            G = ((wavelength - 440) / (490 - 440)) ** gamma
            B = 1.0
        elif wavelength >= 490 and wavelength <= 510:
            R = 0.0
            G = 1.0
            B = (-(wavelength - 510) / (510 - 490)) ** gamma
        elif wavelength >= 510 and wavelength <= 580:
            R = ((wavelength - 510) / (580 - 510)) ** gamma
            G = 1.0
            B = 0.0
        elif wavelength >= 580 and wavelength <= 645:
            R = 1.0
            G = (-(wavelength - 645) / (645 - 580)) ** gamma
            B = 0.0
        elif wavelength >= 645 and wavelength <= 750:
            attenuation = 0.3 + 0.7 * (750 - wavelength) / (750 - 645)
            R = (1.0 * attenuation) ** gamma
            G = 0.0
            B = 0.0
        else:
            R = 0.0
            G = 0.0
            B = 0.0
        R *= 255
        G *= 255
        B *= 255
        
        return (int(R), int(G), int(B))
