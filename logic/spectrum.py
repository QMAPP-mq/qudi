# -*- coding: utf-8 -*-
"""
This file contains the Qudi logic class that captures and processes fluorescence spectra.

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
import matplotlib.pyplot as plt

from core.module import Connector, ConfigOption, StatusVar
from core.util.mutex import Mutex
from core.util.network import netobtain
from logic.generic_logic import GenericLogic


class SpectrumLogic(GenericLogic):

    """This logic module gathers data from the spectrometer.
    """

    _modclass = 'spectrumlogic'
    _modtype = 'logic'

    # declare connectors
    spectrometer = Connector(interface='SpectrometerInterface')
    odmrlogic1 = Connector(interface='ODMRLogic')
    savelogic = Connector(interface='SaveLogic')
    fitlogic = Connector(interface='FitLogic')

    # status variables
    fc = StatusVar('fits', None)

    # Internal signals
    next_diff_loop_Signal = QtCore.Signal()

    # External signals eg for GUI module
    specdata_updated_Signal = QtCore.Signal(np.ndarray)
    spectrum_fit_updated_Signal = QtCore.Signal(np.ndarray, dict, str)

    def __init__(self, **kwargs):
        """ Create SpectrometerLogic object with connectors.

          @param dict kwargs: optional parameters
        """
        super().__init__(**kwargs)

        # locking for thread safety
        self.threadlock = Mutex()

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        self.spectrum_data = np.array([])
        self.spectrum_fit = np.array([])
        self.fit_domain = np.array([])
        self.diff_spec_data_mod_on = np.array([])
        self.diff_spec_data_mod_off = np.array([])
        self.repetition_count = 0    # count loops for differential spectrum

        self._spectrometer_device = self.get_connector('spectrometer')
        self._odmr_logic = self.get_connector('odmrlogic1')
        self._save_logic = self.get_connector('savelogic')
        self._fit_logic = self.get_connector('fitlogic')

        self.next_diff_loop_Signal.connect(self._loop_differential_spectrum)

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        if self.getState() != 'idle' and self.getState() != 'deactivated':
            pass

    @fc.constructor
    def sv_set_fits(self, val):
        # Setup fit container
        fc = self.fitlogic().make_fit_container('ODMR sum', '1d')
        fc.set_units(['Hz', 'c/s'])
        if isinstance(val, dict) and len(val) > 0:
            fc.load_from_dict(val)
        else:
            d1 = OrderedDict()
            d1['Gaussian peak'] = {
                'fit_function': 'gaussian',
                'estimator': 'peak'
                }
            default_fits = OrderedDict()
            default_fits['1d'] = d1
            fc.load_from_dict(default_fits)
        return fc

    @fc.representer
    def sv_get_fits(self, val):
        """ save configured fits """
        if len(val.fit_list) > 0:
            return val.save_to_dict()
        else:
            return None

    def get_single_spectrum(self):
        """ Record a single spectrum from the spectrometer.
        """
        # Clear any previous fit - TODO check this
        self.fc.clear_result()

        self.spectrum_data = netobtain(self._spectrometer_device.recordSpectrum())

        # Clearing the differential spectra data arrays so that they do not get
        # saved with this single spectrum.
        self.diff_spec_data_mod_on = np.array([])
        self.diff_spec_data_mod_off = np.array([])

        self.specdata_updated_Signal.emit(self.spectrum_data)

    def save_raw_spectrometer_file(self, path='', postfix=''):
        """Ask the hardware device to save its own raw file.
        """
        # TODO: sanity check the passed parameters.

        self._spectrometer_device.saveSpectrum(path, postfix=postfix)

    def start_differential_spectrum(self):
        """Start a differential spectrum acquisition.  An initial spectrum is recorded to initialise the data arrays to the right size.
        """

        self._continue_differential = True

        # Taking a demo spectrum gives us the wavelength values and the length of the spectrum data.
        demo_data = netobtain(self._spectrometer_device.recordSpectrum())

        wavelengths = demo_data[0, :]
        empty_signal = np.zeros(len(wavelengths))

        # Using this information to initialise the differential spectrum data arrays.
        self.spectrum_data = np.array([wavelengths, empty_signal])
        self.diff_spec_data_mod_on = np.array([wavelengths, empty_signal])
        self.diff_spec_data_mod_off = np.array([wavelengths, empty_signal])
        self.repetition_count = 0

        # Starting the measurement loop
        self._loop_differential_spectrum()

    def resume_differential_spectrum(self):
        """Resume a differential spectrum acquisition.
        """

        self._continue_differential = True

        # Starting the measurement loop
        self._loop_differential_spectrum()

    def _loop_differential_spectrum(self):
        """ This loop toggles the modulation and iteratively records a differential spectrum.
        """

        # If the loop should not continue, then return immediately without
        # emitting any signal to repeat.
        if not self._continue_differential:
            return

        # Otherwise, we make a measurement and then emit a signal to repeat this loop.

        # Toggle on, take spectrum and add data to the mod_on data
        self.toggle_modulation(on=True)
        these_data = netobtain(self._spectrometer_device.recordSpectrum())
        self.diff_spec_data_mod_on[1, :] += these_data[1, :]

        # Toggle off, take spectrum and add data to the mod_off data
        self.toggle_modulation(on=False)
        these_data = netobtain(self._spectrometer_device.recordSpectrum())
        self.diff_spec_data_mod_off[1, :] += these_data[1, :]

        self.repetition_count += 1    # increment the loop count

        # Calculate the differential spectrum
        self.spectrum_data[1, :] = self.diff_spec_data_mod_on[
            1, :] - self.diff_spec_data_mod_off[1, :]

        self.specdata_updated_Signal.emit(self.spectrum_data)

        self.next_diff_loop_Signal.emit()

    def stop_differential_spectrum(self):
        """Stop an ongoing differential spectrum acquisition
        """

        self._continue_differential = False

    def toggle_modulation(self, on):
        """ Toggle the modulation.
        """

        if on:
            self._odmr_logic.MW_on()
        elif not on:
            self._odmr_logic.MW_off()
        else:
            print("Parameter 'on' needs to be boolean")

    def save_spectrum_data(self, name_tag='', custom_header = None):
        """ Saves the current spectrum data to a file.

        @param string name_tag: custom addition to the filename

        @param ordered dict custom_header:
            Additional custom parameters to include in the save file header.
        """
        filepath = self._save_logic.get_path_for_module(module_name='spectra')

        # If there is a postfix then add separating underscore
        if name_tag == '':
            filelabel = 'spectrum'
        else:
            filelabel = 'spectrum_' + name_tag

        # write experimental parameters
        parameters = OrderedDict()
        parameters['Spectrometer acquisition repetitions'] = self.repetition_count

        # add any custom header params
        if custom_header is not None:
            for key in custom_header:
                parameters[key] = custom_header[key]

        # prepare the data in an OrderedDict:
        data = OrderedDict()

        data['wavelength'] = self.spectrum_data[0, :]

        # If the differential spectra arrays are not empty, save them as raw data
        if len(self.diff_spec_data_mod_on) != 0 and len(self.diff_spec_data_mod_off) != 0:
            data['signal_mod_on'] = self.diff_spec_data_mod_on[1, :]
            data['signal_mod_off'] = self.diff_spec_data_mod_off[1, :]
            data['differential'] = self.spectrum_data[1, :]
        else:
            data['signal'] = self.spectrum_data[1, :]

        # Prepare the figure to save as a "data thumbnail"
        plt.style.use(self._save_logic.mpl_qd_style)

        fig, ax1 = plt.subplots()

        ax1.plot(data['wavelength'], data['signal'])

        ax1.set_xlabel('Wavelength (nm)')
        ax1.set_ylabel('Signal (arb. u.)')

        fig.tight_layout()

        # Save to file
        self._save_logic.save_data(data,
                                   filepath=filepath,
                                   parameters=parameters,
                                   filelabel=filelabel,
                                   plotfig=fig)
        self.log.debug('Spectrum saved to:\n{0}'.format(filepath))

    ################
    # Fitting things 

    def get_fit_functions(self):
        """ Return the hardware constraints/limits
        @return list(str): list of fit function names
        """
        return list(self.fc.fit_list)

    def do_fit(self, fit_function=None, x_data=None, y_data=None):
        """
        Execute the currently configured fit on the measurement data. Optionally on passed data
        """
        if (x_data is None) or (y_data is None):
            x_data = self.spectrum_data[0]
            y_data = self.spectrum_data[1]
            if self.fit_domain.any():
                start_idx = self._find_nearest_idx(x_data, self.fit_domain[0])
                stop_idx = self._find_nearest_idx(x_data, self.fit_domain[1])

                x_data = x_data[start_idx:stop_idx]
                y_data = y_data[start_idx:stop_idx]

        if fit_function is not None and isinstance(fit_function, str):
            if fit_function in self.get_fit_functions():
                self.fc.set_current_fit(fit_function)
            else:
                self.fc.set_current_fit('No Fit')
                if fit_function != 'No Fit':
                    self.log.warning('Fit function "{0}" not available in Spectrum logic '
                                     'fit container.'.format(fit_function)
                                     )

        spectrum_fit_x, spectrum_fit_y, result = self.fc.do_fit(x_data, y_data)

        print(type(spectrum_fit_x))
        print(type(self.spectrum_fit))

        self.spectrum_fit = np.array([spectrum_fit_x, spectrum_fit_y])

        if result is None:
            result_str_dict = {}
        else:
            result_str_dict = result.result_str_dict
        self.spectrum_fit_updated_Signal.emit(self.spectrum_fit,
                                    result_str_dict, self.fc.current_fit)
        return

    def _find_nearest_idx(self, array, value):
        """ Find array index of element nearest to given value

        @param list array: array to be searched.
        @param float value: desired value.

        @return index of nearest element.
        """

        idx = (np.abs(array-value)).argmin()
        return idx
