# -*- coding: utf-8 -*-
"""
Acquire a spectrum using MaxIm DL through the COM interface.
This program get and processes data from MaxIm DL, saves them and
gets the data for plotting.

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

MaxIm DL documentation can be found at <http://diffractionlimited.com/help/maximdl/MaxIm-DL.htm>

"""

from core.module import Base
from interface.spectrometer_interface import SpectrometerInterface
import numpy as np
import win32com.client as w32c
import time

import datetime


class RHEASpectrometer(Base, SpectrometerInterface):
    """ Hardware module for reading spectra from the RHEA spectrometer using an Atik camera.
    """

    def on_activate(self):
        """ Activate module.
        """
        w32c.pythoncom.CoInitialize()
        self._camera = w32c.Dispatch("MaxIm.CCDCamera")
        self._camera.DisableAutoShutdown = True
        self._camera.LinkEnabled = True
        self._camera.CoolerOn = True

        self._exposure_time = 10

    def on_deactivate(self):
        """ Deactivate module.
        """
        self._camera.CoolerOn = False
        self._camera.Quit()

    def recordSpectrum(self):
        """ Record spectrum from WinSpec32 software.

            @return []: spectrum data
        """
        w32c.pythoncom.CoInitialize()

        # ensure that the CCD is cool
        proceed = False
        tries = 0
        while proceed == False and tries < 10:
            proceed = self._confirm_cool()
            tries += 1
            if tries == 10:
                self.log.error('Unable to cool RHEA spectrometer after {} attempts'.format(tries))
                return -1

        self._camera.Expose(self._exposure_time, 1, 0)

        while not self._camera.ImageReady:
            time.sleep(1)

        img = np.array(self._camera.ImageArray).T  # Transpose to match the shape when importing the *.fit files from Maxim DL

        specdata = self._rhea_extract_image(img)
        
        return specdata

    def saveSpectrum(self, path, postfix = ''):
        """ Save spectrum from WinSpec32 software.

            @param str path: path to save origial spectrum
            @param str postfix: file posfix
        """
        savetime=datetime.datetime.now()
        w32c.pythoncom.CoInitialize()
        timestr = savetime.strftime("%Y%m%d-%H%M-%S-%f_")
        self._camera.SaveImage(
            str(path) + timestr + str(postfix) + ".fit"
        )

    def getExposure(self):
        """ Get exposure.

            @return float: exposure

            Not implemented.
        """
        return self._exposure_time

    def setExposure(self, exposureTime):
        """ Set exposure.

            @param float exposureTime: exposure
        """
        self._exposure_time = exposureTime

    def _rhea_extract_image(self, I):
        """ Process RHEA CCD image to extract linear spectrum

            @param I : MaxIm DL image
        """

        fit_dir = 'C:/Data/2018/08/20180816/rhea/'
        echelle_fit = np.loadtxt(fit_dir +'rhea_calibration_echelle_fit.csv')
        y_pix = np.arange(0, len(echelle_fit[0]))
        wlen = np.loadtxt(fit_dir + 'rhea_calibration_wavelengths_fit.csv')

        norm_factor = np.loadtxt('C:\\labdata\\project_inhouse_nds\\20180821_cryo_siv_nds\\rhea_calibration_amp_factor.csv')

        order_halfwidth = 10
        line_spectra = np.zeros(len(echelle_fit) * len(y_pix))  # Same shape as the echelle_fit array

        for order, fit in enumerate(echelle_fit):
            for y in y_pix:
                idx = np.int(np.round(fit[y]))
                signal = np.sum(I[len(y_pix) - y - 1, idx - order_halfwidth : idx + order_halfwidth])
                            
                line_spectra[order * len(y_pix) + y] = signal

        bg_offset = np.mean(line_spectra[0:2200])

        # Amplitude normalisation
        line_spectra = (line_spectra - bg_offset) * norm_factor.reshape(np.shape(line_spectra))

        spectrum_data = np.vstack((wlen*1e-9, line_spectra))

        return spectrum_data

    def _confirm_cool(self, wait_duration=1, temperature_threshold=2):
        """ Return true if the CCD temperature is within the setpoint +/- threshold bounds

            @param wait_duration : default to 1
            @param temperature_threshold : default to 2

            @return bool : True if ok, False if out-of-bounds
        """

        if self._camera.Temperature > self._camera.TemperatureSetpoint + temperature_threshold:
            self.log.warning('RHEA CCD too warm, waiting {} seconds'.format(wait_duration))
            time.sleep(wait_duration)
            return False
        elif self._camera.Temperature < self._camera.TemperatureSetpoint - temperature_threshold:
            self.log.warning('RHEA CCD too cool, waiting {} seconds'.format(wait_duration))
            time.sleep(wait_duration)
            return False
        else:
            return True
