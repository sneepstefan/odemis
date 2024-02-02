import logging
from typing import Optional, List

import numpy

import odemis
from odemis import model
from odemis.driver._mccdaq import usb_1208LS, usb_1208HS_4AO
from odemis.model import HwError

MAX_VOLTAGE_VALUE = 0x3ff  # for 1208LS
MAX_VOLTAGE = 5.0  # V


class Light(model.Emitter):
    def __init__(self, name: str, role: str, device: Optional[str], ao_channels, do_channels, spectra, pwr_curve, **kwargs):
        """
        :param name (str): name of the component defined in the config file.
        :param role (str): role of the component defined in the config file.
        :param device (str or None): serial number or None (null in yaml file) for auto-detect.
        :param ao_channels: (list of (0<=int<=3)):
            The analogue output channel for each source, as numbered in the mccdaq device.
        :param do_channels: (list of (0<=int<=15)):
            The digital output (0 or 5 v) channel for each source, as numbered in the mccdaq device.
        """
        super().__init__(name, role, **kwargs)
        self._shape = ()

        try:
            # self._device = usb_1208LS(device)
            # if device and len(device) < 8:
            #     device = "0" + device
            self._device = usb_1208LS(device)
        except IOError:
            raise HwError("Failed to open MCC DAQ device with s/n '%s'" % device)

        if len(ao_channels) != len(spectra):
            raise ValueError("spectra argument should have the same length as ao_channels (%d)" % len(ao_channels))
        if len(ao_channels) != len(pwr_curve):
            raise ValueError("pwr_curve argument should have the same length as ao_channels (%d)" % len(ao_channels))
        if len(ao_channels) != len(do_channels):
            raise ValueError("do_channels argument should have the same length as ao_channels (%d)" % len(ao_channels))

        self._ao_channels = ao_channels
        self._do_channels = do_channels

        # Check and store the power curves
        self._pwr_curve = []
        for c, crv in zip(ao_channels, pwr_curve):
            crv = [v for v in crv.items()]
            # Add 0W = 0V if nothing = 0W
            if 0 not in [w for v, w in crv]:
                crv.append((0, 0))
                logging.info("Adding 0V -> 0W mapping to pwr_curve for channel %d", c)
            # At least beginning and end values
            if len(crv) < 2:
                raise ValueError("pwr_curve for channel %d has less than 2 values: %s" % (c, crv))
            # Check it's monotonic
            crv = sorted(crv, key=lambda v: v[0])
            if crv[0][1] < 0:
                raise ValueError("pwr_curve for channel %d has negative power: %g W" % (c, crv[0][1]))
            if crv[0][1] > 5:
                raise ValueError("pwr_curve for channel %d has more than 5V power: %g W" % (c, crv[0][1]))
            if len(crv) != len(set(v for v, w in crv)):
                raise ValueError("pwr_curve for channel %d has identical voltages: %s" % (c, crv))
            if not all((crv[i][1] < crv[i + 1][1]) for i in range(len(crv) - 1)):
                raise ValueError("pwr_curve for channel %d is not monotonic: %s" % (c, crv))

            self._pwr_curve.append(crv)

        # Check the spectra
        spect = []  # list of the 5 wavelength points
        for c, wls in zip(ao_channels, spectra):
            if len(wls) != 5:
                raise ValueError("Spectra for ao_channel %d doesn't have exactly 5 wavelength points: %s" % (c, wls))
            if list(wls) != sorted(wls):
                raise ValueError("Spectra for ao_channel %d has unsorted wavelengths: %s" % (c, wls))
            for wl in wls:
                if not 0 < wl < 100e-6:
                    raise ValueError("Spectra for ao_channel %d has unexpected wavelength = %f nm"
                                     % (c, wl * 1e9))
            spect.append(tuple(wls))

            # Maximum power for channel to be used as a range for power
            max_power = tuple([crv[-1][1] for crv in self._pwr_curve])
            # Power value for each channel of the device
            self.power = model.ListContinuous(value=[0.] * len(ao_channels),
                                              range=(tuple([0.] * len(ao_channels)), max_power,),
                                              unit="W", cls=(int, float), )
            self.power.subscribe(self._update_power)

            # info on which channel is which wavelength
            self.spectra = model.ListVA(spect, unit="m", readonly=True)

            # make sure everything is off (turning on the HUB will turn on the lights)
            self.power.value = self.power.range[0]

            if self._device:
                self._metadata = {model.MD_HW_NAME: f"{self._device.getManufacturer()} {self._device.getProduct()} {self._device.getSerialNumber()}"}
            self._swVersion = odemis.__version__

            self._metadata[model.MD_SW_VERSION] = self._swVersion
            self._metadata[model.MD_HW_VERSION] = self._hwVersion

    def _power_to_volt(self, power: float, curve) -> float:
        """
        power (0 < float)
        curve (list of tuple (float, float)): the mapping between volt -> power
        return (float): voltage for outputting the given power
        raise (ValueError): if the requested power value is out of the power curve range limits
        """
        if power < curve[0][1]:
            raise ValueError("Power requested %g < %g" % (power, curve[0][1]))

        # Find the segment that correspond to that power
        for i, (v, w) in enumerate(curve[1:]):
            if power <= w:
                seg = i
                break
        else:
            raise ValueError("Power requested %g > %g" % (power, curve[-1][1]))

        logging.debug("Converting %g W using segment %d: %s -> %s",
                      power, seg, curve[seg], curve[seg + 1])

        basev, basew = curve[seg]
        endv, endw = curve[seg + 1]

        ratio = (power - basew) / (endw - basew)
        v = basev + ratio * (endv - basev)
        return v

    def _volt_to_data(self, volt: float):
        return (volt / MAX_VOLTAGE) * MAX_VOLTAGE_VALUE

    def _channel_to_port(self, channel):
        if channel < 0 or channel > 15:
            raise ValueError("DIO channel value has to be between 0 and 15")

        # convert the channel number to port and bit
        if channel in range(0, 8):
            # channel 0 - 7 in config file
            port = self._device.DIO_PORTA
            bit = channel
        else:
            port = self._device.DIO_PORTB
            # channel 8 - 15 in config file
            bit = channel - 8

        return port, bit

    def _update_power(self, value: List[float]):
        for ao_ch, do_ch, crv, pwr in zip(self._ao_channels, self._do_channels, self._pwr_curve, value):
            pwr = min(pwr, crv[-1][1])
            volt = self._power_to_volt(pwr, crv)
            port, bit = self._channel_to_port(do_ch)
            if isinstance(self._device, usb_1208LS):
                d = int(self._volt_to_data(volt))  # cast to uint16
            elif isinstance(self._device, usb_1208HS_4AO):
                d = volt
            logging.debug("Setting ao_channel %d to %g V = %g W", ao_ch, volt, pwr)
            # update the analogue output value
            self._device.AOut(ao_ch, d)
            # update the digital output value by using a direct digital port bit
            self._device.DBitOut(port, bit, int(pwr > 0))
