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

    sigCountDataNext = QtCore.Signal()

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

        # TODO open polling loop
        return

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        # TODO stop polling
        return