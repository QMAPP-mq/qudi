# -*- coding: utf-8 -*-
"""
This file contains the Qudi powermeter logic class.

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
# from interface.slow_counter_interface import CountingMode  # required?
from core.util.mutex import Mutex


class PowermeterLogic(GenericLogic):
    """ This logic module gathers data from a hardware powermeter device.

    @signal sigCounterUpdate: there is new counting data available
    @signal sigCountContinuousNext: used to simulate a loop in which the data
                                    acquisition runs.
    @sigmal sigCountGatedNext: ???

    @return error: 0 is OK, -1 is error
    """
    sigCounterUpdated = QtCore.Signal()

    sigpowerdataNext = QtCore.Signal()

    sigGatedCounterFinished = QtCore.Signal()
    sigGatedCounterContinue = QtCore.Signal(bool)
    sigCountingSamplesChanged = QtCore.Signal(int)
    sigCountLengthChanged = QtCore.Signal(int)
    sigCountFrequencyChanged = QtCore.Signal(float)
    sigSavingStatusChanged = QtCore.Signal(bool)
    sigCountStatusChanged = QtCore.Signal(bool)
    sigCountingModeChanged = QtCore.Signal(CountingMode)

    _modclass = 'PowermeterLogic'
    _modtype = 'logic'

    ## declare connectors
    pm1 = Connector(interface='SlowCounterInterface') # TODO make an interface
    savelogic = Connector(interface='SaveLogic')

    # status vars
    _count_length = StatusVar('count_length', 300)
    _smooth_window_length = StatusVar('smooth_window_length', 10)
    _counting_samples = StatusVar('counting_samples', 1)
    _count_frequency = StatusVar('count_frequency', 50)
    _saving = StatusVar('saving', False)

    _wavelength = StatusVar('wavelength', 532e-9)

    def __init__(self, config, **kwargs):
        """ Create CounterLogic object with connectors.

            @param dict config: module configuration
            @param dict kwargs: optional parameters
        """

        super().__init__(config=config, **kwargs)

        #locking for thread safety
        self.threadlock = Mutex()

        self.log.debug('The following configuration was found.')

        # checking for the right configuration
        for key in config.keys():
            self.log.debug('{0}: {1}'.format(key, config[key]))

        # in bins
        self._count_length = 300
        self._smooth_window_length = 10
        self._counting_samples = 1      # oversampling
        # in hertz
        self._count_frequency = 50

        # self._binned_counting = True  # UNUSED?

        self._saving = False
        return

    def on_activate(self):
        """ Initialise and activate the logic module.
        """

        # Connect to hardware and save logic
        self._powermeter_device = self.get_connector('pm1')
        self._save_logic = self.get_connector('savelogic')

        # Recall saved app-parameters
        if 'counting_mode' in self._statusVariables:
            self._counting_mode = CountingMode[self._statusVariables['counting_mode']]

        constraints = self.get_hardware_constraints()
        number_of_detectors = constraints.max_detectors

        # initialize data arrays
        self.powerdata = np.zeros([len(self.get_channels()), self._count_length])
        self.powerdata_smoothed = np.zeros([len(self.get_channels()), self._count_length])
        self.rawdata = np.zeros([len(self.get_channels()), self._counting_samples])
        self._already_counted_samples = 0  # For gated counting
        self._data_to_save = []

        # Flag to stop the loop
        self.stopRequested = False

        self._saving_start_time = time.time()

        # connect signals
        self.sigpowerdataNext.connect(self._count_loop_body, QtCore.Qt.QueuedConnection)
        return

    def on_deactivate(self):
        """ Deinitialise and deactivate the logic module.
        """

        # Save parameters to disk
        self._statusVariables['counting_mode'] = self._counting_mode.name

        # Stop measurement
        if self.getState() == 'locked':
            self._stopCount_wait()

        self.sigpowerdataNext.disconnect()
        return

########################## property declarations ##############################

    @property
    def trace_length(self):
        """ Get trace length

            @return int : the trace length
        """

        # TODO check access to _powermeter_device._sampling_time
        self.trace_length = (self._powermeter_device.get_averaging_window()
                                * self._powermeter_device._sampling_time)
        return self._trace_length
    
    @trace_length.setter
    def trace_length(self, new_length):
        """ Set the trace length

            @param int new_length : desired trace length
            
            @return error code (0:OK, -1:error)
        """

        try:
            if not isinstance(new_length, int):
                self.log.warning("trace_length must be integer."
                                "I have used {} instead of the requested {}."
                                .format(int(new_length), new_length)
                                )
                new_length = int(new_length)

            # Determine if the counter has to be restarted after setting the parameter
            if self.getState() == 'locked':
                restart = True
            else:
                restart = False

            if new_length > 0:
                self._stopCount_wait()
                self._trace_length = new_length
                # if the counter was running, restart it
                if restart:
                    self.start_trace()
            else:
                self.log.warning('trace_length has to be larger than 0! Command ignored!')

            # TODO check access to _powermeter_device._sampling_time
            self._powermeter_device.set_averaging_window(self._trace_length /
                    self._powermeter_device._sampling_time)

            self.sigCountingSamplesChanged.emit(self._trace_length)
        
            return 0
        
        except:
            self.log.warning('An error occured setting the trace length')
            return -1

    @property
    # TODO how to connect this to the hardware?
    def sampling_frequency(self):
        """ Get the currently set trace sampling frequency (resolution)

            @return float : the sampling frequency in Hz
        """
        return self._sampling_frequency

    @sampling_frequency.setter
    def sampling_frequency(self, new_frequency=50):
        """ Set the sampling frequency with which the data is acquired

            @param float frequency : desired sampling frequency in Hz

            @return error code (0:OK, -1:error)
        """

        constraints = self.get_hardware_constraints()

        try:
            if self.getState() == 'locked':
                restart = True
            else:
                restart = False

            if constraints.min_sampling_frequency <= new_frequency <= constraints.max_sampling_frequency:
                self._stopCount_wait()
                self._sampling_frequency = new_frequency
                # if the counter was running, restart it
                if restart:
                    self.start_trace()
            else:
                self.log.warning('sampling_frequency not in range! Command ignored!')

            hw_averaging_window = self._powermeter_device.set_averaging_window(1 / self._sampling_frequency)

            # TODO: logic can always run at any sampling_frequency, but HW may only have
            # discrete sampling rates.

            self.sigCountFrequencyChanged.emit(self._sampling_frequency)

            return 0

        except:
            self.log.warning('An error occured setting the sampling frequency')
            return -1

    @property
    def wavelength(self):
        """ Get the powermeter's detection wavelength

            @return float : the detection wavelength in meters
        """
        self.wavelength = self._powermeter_device.get_wavelength()
        return self._wavelength

    @wavelength.setter
    def wavelength(self, new_wavelength=532):
        """ Set the powermeter's detection wavelength

            @param new_wavelength float : desired detection wavelength in meters

            @return error code (0:OK, -1:error)
        """
        # to avoid interfacing with the hardware twice
        x = self._powermeter_device.set_wavelength(new_wavelength)

        if x == new_wavelength:
            self.wavelength = x
            return 0
        else:
            self.log.warning('An error occured setting the detection wavlength')
            return -1

########################## ############## #####################################

    def get_hardware_constraints(self):
        """
        Retrieve the hardware constrains from the counter device.

            @return SlowCounterConstraints: object with constraints for the powermeter
        """
        return self._powermeter_device.get_constraints()

    def get_channels(self):
        """ Shortcut for hardware get_counter_channels.

            @return list(str): return list of active counter channel names
        """
        return self._powermeter_device.get_counter_channels()

    def draw_figure(self, data):
        """ Draw figure to save with data file.

            @param: nparray data: a numpy array containing counts vs time for all detectors

            @return: fig fig: a matplotlib figure object to be saved to file.
        """
        
        count_data = data[:, 1:len(self.get_channels())+1]
        time_data = data[:, 0]

        # Scale count values using SI prefix
        prefix = ['', 'k', 'M', 'G']
        prefix_index = 0
        while np.max(count_data) > 1000:
            count_data = count_data / 1000
            prefix_index = prefix_index + 1
        counts_prefix = prefix[prefix_index]

        # Use qudi style
        plt.style.use(self._save_logic.mpl_qd_style)

        # Create figure
        fig, ax = plt.subplots()
        ax.plot(time_data, count_data, linestyle=':', linewidth=0.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Fluorescence (' + counts_prefix + 'c/s)')
        return fig

    # FIXME: Not implemented for self._counting_mode == 'gated'
    def start_trace(self):
        """ This is called externally, and is basically a wrapper that
            redirects to the chosen counting mode start function.

            @return error: 0 is OK, -1 is error
        """

        # Sanity checks
        constraints = self.get_hardware_constraints()

        with self.threadlock:
            # Lock module
            if self.getState() != 'locked':
                self.lock()
            else:
                self.log.warning('Trace already running. Method call ignored.')
                return 0

            # initialising the data arrays
            self.rawdata = np.zeros([len(self.get_channels()), self._counting_samples])
            self.powerdata = np.zeros([len(self.get_channels()), self._count_length])
            self.powerdata_smoothed = np.zeros([len(self.get_channels()), self._count_length])
            self._sampling_data = np.empty([len(self.get_channels()), self._counting_samples])

            # the sample index for gated counting
            self._already_counted_samples = 0

            # Start data reader loop
            self.sigCountStatusChanged.emit(True)
            self.sigpowerdataNext.emit()
            return

    def stopCount(self):
        """ Set a flag to request stopping counting.
        """

        if self.getState() == 'locked':
            with self.threadlock:
                self.stopRequested = True
        return

########################## saving methods #####################################

    def get_saving_state(self):
        """ Returns if the data is saved in the moment.

        @return bool: saving state
        """
        return self._saving

    def start_saving(self, resume=False):
        """
        Sets up start-time and initializes data array, if not resuming, and changes saving state.
        If the powermeter is not running it will be started in order to have data to save.

            @return bool: saving state
        """

        if not resume:
            self._data_to_save = []
            self._saving_start_time = time.time()

        self._saving = True

        # If the powermeter is not running, then it should start running so there is data to save
        if self.getState() != 'locked':
            self.start_trace()

        self.sigSavingStatusChanged.emit(self._saving)
        return self._saving

    def save_data(self, to_file=True, postfix=''):
        """ Save the power trace data and writes it to a file.

            @param bool to_file: indicate, whether data have to be saved to file
            @param str postfix: an additional tag, which will be added to the filename upon save

            @return dict parameters: Dictionary which contains the saving parameters
        """

        # stop saving thus saving state has to be set to False
        self._saving = False
        self._saving_stop_time = time.time()

        # write the parameters:
        parameters = OrderedDict()
        parameters['Start counting time'] = time.strftime('%d.%m.%Y %Hh:%Mmin:%Ss', time.localtime(self._saving_start_time))
        parameters['Stop counting time'] = time.strftime('%d.%m.%Y %Hh:%Mmin:%Ss', time.localtime(self._saving_stop_time))
        parameters['Count frequency (Hz)'] = self._count_frequency
        parameters['Oversampling (Samples)'] = self._counting_samples
        parameters['Smooth Window Length (# of events)'] = self._smooth_window_length

        if to_file:
            # If there is a postfix then add separating underscore
            if postfix == '':
                filelabel = 'power_trace'
            else:
                filelabel = 'power_trace_' + postfix

            # prepare the data in a dict or in an OrderedDict:
            header = 'Time (s)'
            for i, detector in enumerate(self.get_channels()):
                header = header + ',Signal{0} (counts/s)'.format(i)

            data = {header: self._data_to_save}
            filepath = self._save_logic.get_path_for_module(module_name='Powermeter')

            fig = self.draw_figure(data=np.array(self._data_to_save))
            self._save_logic.save_data(data, filepath=filepath, parameters=parameters,
                                       filelabel=filelabel, plotfig=fig, delimiter='\t')
            self.log.info('Power Trace saved to:\n{0}'.format(filepath))

        self.sigSavingStatusChanged.emit(self._saving)
        return self._data_to_save, parameters

########################## internal methods ###################################

    def _count_loop_body(self):
        """ This method gets the count data from the hardware for the continuous measurement.

        It runs repeatedly in the logic module event loop by being connected
        to sigCountContinuousNext and emitting sigCountContinuousNext through a queued connection.
        """

        if self.getState() == 'locked':
            with self.threadlock:
                # check for aborts of the thread in break if necessary
                if self.stopRequested:
                    # close off the actual counter
                    cnt_err = self._powermeter_device.close_counter()
                    clk_err = self._powermeter_device.close_clock()
                    if cnt_err < 0 or clk_err < 0:
                        self.log.error('Could not even close the hardware, giving up.')
                    # switch the state variable off again
                    self.stopRequested = False
                    self.unlock()
                    self.sigCounterUpdated.emit()
                    return

                # read the current power value
                self.rawdata = self._powermeter_device.get_power(self._counting_samples)
                if self.rawdata[0, 0] < 0:
                    self.log.error('The counting went wrong, killing the powermeter.')
                    self.stopRequested = True
                else:
                    self._process_data()

            # wait the sampling period
            time.sleep(1 / self._sampling_frequency)
            # call this again from event loop
            self.sigCounterUpdated.emit()
            self.sigpowerdataNext.emit()
        return

    def _process_data(self):
        """
        Processes the raw data from the powermeter device

            @return:
        """

        for i, ch in enumerate(self.get_channels()):
            # remember the new count data in circular array
            self.powerdata[i, 0] = np.average(self.rawdata[i])
        # move the array to the left to make space for the new data
        self.powerdata = np.roll(self.powerdata, -1, axis=1)
        # also move the smoothing array
        self.powerdata_smoothed = np.roll(self.powerdata_smoothed, -1, axis=1)
        # calculate the median and save it
        window = -int(self._smooth_window_length / 2) - 1
        for i, ch in enumerate(self.get_channels()):
            self.powerdata_smoothed[i, window:] = np.median(self.powerdata[i,
                                                            -self._smooth_window_length:])

        # save the data if necessary
        if self._saving:
             # if oversampling is necessary
            if self._counting_samples > 1:
                chans = self.get_channels()
                self._sampling_data = np.empty([len(chans) + 1, self._counting_samples])
                self._sampling_data[0, :] = time.time() - self._saving_start_time
                for i, ch in enumerate(chans):
                    self._sampling_data[i+1, 0] = self.rawdata[i]

                self._data_to_save.extend(list(self._sampling_data))
            # if we don't want to use oversampling
            else:
                # append tuple to data stream (timestamp, average counts)
                chans = self.get_channels()
                newdata = np.empty((len(chans) + 1, ))
                newdata[0] = time.time() - self._saving_start_time
                for i, ch in enumerate(chans):
                    newdata[i+1] = self.powerdata[i, -1]
                self._data_to_save.append(newdata)
        return

    def _stopCount_wait(self, timeout=5.0):
        """
        Stops the powermeter and waits until it actually has stopped.

            @param timeout: float, the max. time in seconds how long the method
                            should wait for the process to stop.

            @return: error code (0:OK, -1:error)
        """

        self.stopCount()
        start_time = time.time()
        while self.getState() == 'locked':
            time.sleep(0.1)
            if time.time() - start_time >= timeout:
                self.log.error('Stopping the powermeter timed out after {0}s'.format(timeout))
                return -1
        return 0
