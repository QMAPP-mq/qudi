# -*- coding: utf-8 -*-
"""
This file contains the Qudi counter logic class.

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
import numpy as np
import time
import matplotlib.pyplot as plt

from core.module import Connector, StatusVar
from logic.generic_logic import GenericLogic
from interface.slow_counter_interface import CountingMode
from core.util.mutex import Mutex


class CounterEmulator(GenericLogic):
    """ This logic module polls a remote counter logic and emulates signals locally.

    This is required for all functionality that involves the logic communicating to
    the GUI via signals.
    """
    sigCounterUpdated = QtCore.Signal()

    start_polling_Signal = QtCore.Signal()

    sigGatedCounterFinished = QtCore.Signal()
    sigGatedCounterContinue = QtCore.Signal(bool)
    sigCountingSamplesChanged = QtCore.Signal(int)
    sigCountLengthChanged = QtCore.Signal(int)
    sigCountFrequencyChanged = QtCore.Signal(float)
    sigSavingStatusChanged = QtCore.Signal(bool)
    sigCountStatusChanged = QtCore.Signal(bool)
    sigCountingModeChanged = QtCore.Signal(CountingMode)


    _modclass = 'CounterEmulator'
    _modtype = 'logic'

    ## declare connectors
    counterlogic = Connector(interface='SlowCounterInterface')

    # # status vars
    # _count_length = StatusVar('count_length', 300)
    # _smooth_window_length = StatusVar('smooth_window_length', 10)
    # _counting_samples = StatusVar('counting_samples', 1)
    # _count_frequency = StatusVar('count_frequency', 50)
    # _saving = StatusVar('saving', False)


    def __init__(self, config, **kwargs):
        """ Create CounterLogic object with connectors.

        @param dict config: module configuration
        @param dict kwargs: optional parameters
        """
        super().__init__(config=config, **kwargs)

        #locking for thread safety
        self.threadlock = Mutex()

        return

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        # Connect to hardware and save logic
        self._counterlogic = self.get_connector('counterlogic')

        # Initialise polling interval
        self._polling_interval = 1.0

        # pass through of data arrays
        countdata_list, smoothed_list = self._counterlogic.get_countdata()
        self.countdata = np.array(countdata_list)
        self.countdata_smoothed = np.array(smoothed_list)

        # connect signals
        self.start_polling_Signal.connect(self.poll_loop, QtCore.Qt.QueuedConnection)

        self.start_polling_Signal.emit()
        # TODO open polling loop
        return

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        # TODO stop polling
        return

#######################
# Reverse signal emulation
# logic -> gui
    def poll_loop(self):
        """ Periodically poll the logic, and emit appropriate signals locally if
        things have changed
        """
        if self._counterlogic.module_state.isstate('locked'):
            # pass through of data arrays
            if not self.module_state.isstate('locked'):
                self.module_state.lock()
            countdata_list, smoothed_list = self._counterlogic.get_countdata()
            self.countdata = np.array(countdata_list)
            self.countdata_smoothed = np.array(smoothed_list)
            self.sigCounterUpdated.emit()
        else:
            if not self.module_state.isstate('idle'):
                self.module_state.unlock()

        time.sleep(self._polling_interval)
        print('polling complete')
        self.start_polling_Signal.emit()

        
        # self._counting_logic.sigCountingSamplesChanged.connect(self.update_oversampling_SpinBox)
        # self._counting_logic.sigCountLengthChanged.connect(self.update_count_length_SpinBox)
        # self._counting_logic.sigCountFrequencyChanged.connect(self.update_count_freq_SpinBox)
        # self._counting_logic.sigSavingStatusChanged.connect(self.update_saving_Action)
        # self._counting_logic.sigCountingModeChanged.connect(self.update_counting_mode_ComboBox)
        # self._counting_logic.sigCountStatusChanged.connect(self.update_count_status_Action)


########################
# Forward pass-through
# gui -> logic
    def get_hardware_constraints(self):
        return self._counterlogic.get_hardware_constraints()

    def get_count_length(self):
        return self._counterlogic.get_count_length()

    def get_count_frequency(self):
        return self._counterlogic.get_count_frequency()

    def get_counting_samples(self):
        return self._counterlogic.get_counting_samples()

    def get_saving_state(self):
        return self._counterlogic.get_saving_state()
        
    def get_counting_mode(self):
        return self._counterlogic.get_counting_mode()
    
    def get_channels(self):
        #return self._counterlogic.get_channels()
        return ['Ctr0']

    def set_counting_samples(self, samples=1):
        return self._counterlogic.set_counting_samples(samples)

    def set_count_length(self, length=300):
        return self._counterlogic.set_count_length(length)

    def set_count_frequency(self, frequency=50):
        return self._counterlogic.set_count_frequency(frequency)

    def set_counting_mode(self, mode='CONTINUOUS'):
        return self._counterlogic.set_counting_mode(mode)

    def startCount(self):
        return self._counterlogic.startCount()

    def stopCount(self):
        self._counterlogic.stopCount()

    def start_saving(self, resume=False):
        return self._counterlogic.start_saving(resume)

    def save_data(self, to_file=True, postfix=''):
        return self._counterlogic.save_data(to_file, postfix)

##############
# Emulator specific tools
    
    def _read_remote_data(self, remote_data):

        localdata = []
        for i,row in enumerate(remote_data):
            local_row = [float(i) for i in remote_data[i]]
            localdata.append(local_row) 
        
        localdata = np.array(localdata)
        return localdata

    def set_polling_interval(self, new_interval):
        self._polling_interval = new_interval