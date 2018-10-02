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

from core.module import Base, ConfigOption
from interface.spectrometer_interface import SpectrometerInterface
from astropy.io import fits

import numpy as np
import win32com.client as w32c
import time

import datetime


class RHEASpectrometer(Base, SpectrometerInterface):
    """ Hardware module for reading spectra from the RHEA spectrometer using an Atik camera.
    """

    _calib_dir = ConfigOption('calib_dir', missing='error')

    _echelle_calib = ConfigOption('echelle_calib', missing='error')
    _wavelen_calib = ConfigOption('wavelen_calib', missing='error')
    _amplit_calib = ConfigOption('amplit_calib', missing='error')
    _background_image = ConfigOption('background_image', default=None, missing='warn')

    def on_activate(self):
        """ Activate module.
        """
        w32c.pythoncom.CoInitialize()
        self._camera = w32c.Dispatch("MaxIm.CCDCamera")
        self._camera.DisableAutoShutdown = True
        self._camera.LinkEnabled = True
        self._camera.CoolerOn = True

        self._exposure_time = 10

        # self._bg_image = _background_image
        if self.set_bg_image(self._calib_dir + self._background_image) == -1:
            self._bg_image = None
        else:
            self.log.info('Spectrometer backfround file {} loaded'.format(self._background_image))

        # Load calibration files
        self._echelle_fit = np.loadtxt(self._calib_dir + self._echelle_calib)
        self._y_pix = np.arange(0, len(self._echelle_fit[0]))
        
        self._wlen = np.loadtxt(self._calib_dir + self._wavelen_calib)
        self._norm_factor = np.loadtxt(self._calib_dir + self._amplit_calib)

    def on_deactivate(self):
        """ Deactivate module.
        """
        self._camera.CoolerOn = False
        self._camera.Quit()

    def recordSpectrum(self):
        """ Record spectrum from MaxIm DL software.

            @return []: spectrum data
        """
        w32c.pythoncom.CoInitialize()

        # ensure that the CCD is cool
        proceed = False
        tries = 0
        while proceed == False and tries < 10:
            proceed = self._confirm_cool(wait_duration = 2.5)
            tries += 1
            if tries == 10:
                self.log.error('Unable to cool RHEA spectrometer after {} attempts'.format(tries))
                return np.array([[0, 1], [0, 0]])  # Simple "empty" data

        self._camera.Expose(self._exposure_time, 1, 0)

        while not self._camera.ImageReady:
            time.sleep(1)

        img = np.array(self._camera.ImageArray).T  # Transpose to match the shape when importing the *.fit files from Maxim DL

        specdata = self._rhea_extract_image(img)
        
        return specdata

    def saveSpectrum(self, path, postfix = ''):
        """ Save spectrum from MaxIm DL software.

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

    def set_bg_image(self, filepath):
        """Set a *.fit file as a background image for subtraction

        @param string filepath: file path and name of the *.fit dark bg image.
        """
        try:
            spectral_image = fits.open(filepath)
        except:
            self.log.error('Cannot open the requested'
                           'background image at {}'.format(filepath)
                          )
            return -1

        self._bg_image = np.array(spectral_image[0].data, dtype=float)

        return 0

    def _rhea_extract_image(self, img):
        """ Process RHEA CCD image to extract linear spectrum

            @param img : MaxIm DL image
        """
        # Background subtraction
        if self._bg_image is None:
            img = img - 1600  # TODO: check whether minimum is better here. 
                              # Do *something* smarter!
        else:
            img = img - self._bg_image

        # Unwrap echelle ROIs
        order_halfwidth = 10
        line_spectra = np.zeros(len(self._echelle_fit) * len(self._y_pix))  # Same shape as the echelle_fit array

        for order, fit in enumerate(self._echelle_fit):
            for y in self._y_pix:
                idx = np.int(np.round(fit[y]))
                signal = np.sum(img[len(self._y_pix) - y - 1, idx - order_halfwidth : idx + order_halfwidth])
                            
                line_spectra[order * len(self._y_pix) + y] = signal

        # Amplitude normalisation
        line_spectra = (line_spectra) * self._norm_factor.reshape(np.shape(line_spectra))

        spectrum_data = np.vstack((self._wlen*1e-9, line_spectra))

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
