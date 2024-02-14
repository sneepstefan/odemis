import logging
import time
from threading import Thread
from typing import Optional, List, Tuple, Dict

import odemis
from odemis import model
from odemis.driver._mccdaq import usb_1208LS
from odemis.model import HwError, Emitter, HwComponent

MAX_VOLTAGE_VALUE = 0x3ff  # for 1208LS
MAX_VOLTAGE = 5.0  # V
INTERLOCK_POLL_INTERVAL = 0.1


class MCCDevice(HwComponent):
    """
    Basic version of the Class for MCC DAQ device functionality with support for Interlock.
    This class can be inherited for more functionality to support more complex components.
    """
    def __init__(self, name: str, role: str, pwr_device: str,
                 di_channels: Optional[Dict[int:List[str, bool]]], **kwargs):
        """
        :param pwr_device (str or None): serial number or None (null in yaml file) for auto-detect.
            the device is considered a USB-powered MCC DAQ 1208LS device.
        :param il_channel: the DIO channel used for interlock through the MCC device
        :param interval: polling interval for interlock status in s
        """
        super().__init__(name, role, **kwargs)
        # force add a trailing 0 or just add a comment to config file?
        # if device and len(device) < 8:
        #     device = "0" + device
        self._name = name
        self._di_channels = di_channels  # ex: di_ports: {2: ["interlockTriggered", False]
        self.device = None
        self._status_threads = []
        self._channel_vas = {}

        if pwr_device == "fake":
            self.device = MCCDeviceSimulator()
        else:
            try:
                self.device = usb_1208LS(pwr_device)
            except HwError:
                raise HwError("Failed to open MCC DAQ device with s/n '%s'" % pwr_device)

        for channel, (va_name, high_is_true) in self._di_channels.items():
            # Create a VA with False (off), True (on) and None (auto, the default)
            va = model.BooleanVA(False, readonly=True)
            self._channel_vas[channel] = va_name, high_is_true, va
            setattr(self, va_name, va)

            new_status_thread = MCCDeviceInterlock(self.device, self._channel_vas)
            new_status_thread.start()
            self._status_threads.append(new_status_thread)

            logging.info(f"{va_name} status activated for component {self._name} on channel {channel}")

    @classmethod
    def channel_to_port(cls, channel: int):
        """
        This is a support method to return the port and bit of a selected channel
        which is used for the DBitIn and DBitOut commands of the MCC device
        :param channel (int):
        :return (tuple(int, int)):
        """
        if channel < 0 or channel > 15:
            raise ValueError("DIO channel value has to be between 0 and 15")

        # convert the channel number to port and bit
        if channel in range(0, 8):
            # channel 0 - 7 in config file
            port = usb_1208LS.DIO_PORTA
            bit = channel
        else:
            port = usb_1208LS.DIO_PORTB
            # channel 8 - 15 in config file
            bit = channel - 8

        return port, bit

    def terminate(self):
        # release the running status threads
        for t in self._status_threads:
            t.terminated = True
            # wait for the tread to be really suspended
            t.join()
        super().terminate()


class MCCDeviceInterlock(Thread):
    """
    If the InterLock TTL is LOW: TTL signal to the light device should also be low. Notify the user in de GUI.
    If the Interlock TTL was HIGH, and becomes LOW: send a TTL signal to the light device as well as
    notifying the user through the GUI.
    """
    def __init__(self, mcc_device, channel_vas):
        # threaded polling of the interlock status bit of the MCC device
        Thread.__init__(self)
        self._channel_vas = channel_vas
        self._device = mcc_device
        self._init = True
        self.terminated = False

    def run(self):
        port, _ = MCCDevice.channel_to_port(list(self._channel_vas.keys())[0])
        # set the start bit status
        # TODO convert the status value to bitwise notation
        self._bit_status = self._device.DIn(port)

        while not self.terminated:
            time.sleep(INTERLOCK_POLL_INTERVAL)
            if self._init:
                logging.debug(f"Started polling TTL status for {self.il_name}...")
                self._init = False

            # bit status = 0 -> TTL LOW | bit status = 1 -> TTL HIGH
            new_bit_status = self._device.DBitIn(port, bit)
            if self._bit_status != new_bit_status:
                if self.il_va:
                    # change the interlockTriggered VA of the parent component
                    self.il_va.value._set_value(bool(self._bit_status), force_write=True)
                    logging.info(f"interlock triggered status = {bool(self._bit_status)}")
                self._bit_status = new_bit_status


class MCCDeviceLight(Emitter, MCCDevice):
    def __init__(self, name: str, role: str, pwr_device: str, ao_channels: List[int],
                 do_channels: List[int], dio_channels: Optional[Dict[int:List[str, bool]]],
                 spectra, pwr_curve, **kwargs):
        """
        :param name (str): name of the component defined in the config file.
        :param role (str): role of the component defined in the config file.
        :param ao_channels: (list of (0<=int<=3)):
            The analogue output channel for each source, as numbered in the mccdaq device.
        :param do_channels: (list of (0<=int<=15)):
            The digital output (0 or 5 v) channel for each source, as numbered in the mccdaq device.
        """
        # super().__init__(name, role, **kwargs)
        Emitter.__init__(self, name, role, **kwargs)
        MCCDevice.__init__(self, name, role, pwr_device, dio_channels)

        self._shape = ()
        self._name = name
        # self._pwr_device = pwr_device
        # self._il_channel = il_channel

        if len(ao_channels) != len(spectra):
            raise ValueError("spectra argument should have the same length as ao_channels (%d)" % len(ao_channels))
        if len(ao_channels) != len(pwr_curve):
            raise ValueError("pwr_curve argument should have the same length as ao_channels (%d)" % len(ao_channels))
        if len(ao_channels) != len(do_channels):
            raise ValueError("do_channels argument should have the same length as ao_channels (%d)" % len(ao_channels))

        self._ao_channels = ao_channels
        self._do_channels = do_channels

        # if self._pwr_device == "fake":
        #     self.device = MCCDeviceSimulator()
        # else:
        #     try:
        #         self.device = usb_1208LS(self._pwr_device)
        #     except HwError:
        #         raise HwError("Failed to open MCC DAQ device with s/n '%s'" % self._pwr_device)
        #
        # if self._il_channel is not None:
        #     self.interlockTriggered = model.BooleanVA(False)
        #     self._status_thread = MCCDeviceInterlock(self.device,
        #                                              self._il_channel,
        #                                              self._name,
        #                                              self.interlockTriggered)
        #     self._status_thread.start()
        #     logging.info(f"Interlock status activated for component {self._name} on channel {self._il_channel}")
        # else:
        #     logging.debug("Not setting interlock VA as no interlock channel is given.")

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

        self._metadata = {model.MD_HW_NAME: f"{self.device.getManufacturer()} "
                                            f"{self.device.getProduct()} "
                                            f"{self.device.getSerialNumber()}"}
        self._swVersion = odemis.__version__

        self._metadata[model.MD_SW_VERSION] = self._swVersion
        self._metadata[model.MD_HW_VERSION] = self._hwVersion

    def _power_to_volt(self, power: float, curve: List[Tuple[float, float]]) -> float:
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

    def _update_power(self, value: List[float]):
        for ao_ch, do_ch, crv, pwr in zip(self._ao_channels, self._do_channels, self._pwr_curve, value):
            pwr = min(pwr, crv[-1][1])
            volt = self._power_to_volt(pwr, crv)
            data = int(self._volt_to_data(volt))
            # update the analogue output value
            logging.debug(f"Setting ao_channel {ao_ch} to {volt} V = {pwr} W")
            self.device.AOut(ao_ch, data)

            port, bit = MCCDevice.channel_to_port(do_ch)
            old_bit_value = self.device.DBitIn(port, bit)
            new_bit_value = int(pwr > 0)
            # update the digital output value by using a direct digital port bit
            if old_bit_value != new_bit_value:
                logging.debug(f"Setting dio_channel {do_ch} from {old_bit_value} to {new_bit_value}")
                self.device.DBitOut(port, bit, new_bit_value)

    def _on_interlock_change(self):
        # update user in the GUI that interlock has been triggered
        pass


class MCCDeviceSimulator(object):
    """
    A really basic and simple interface to simulate a USB-1208LS device with
    support for DIO pin read/write commands and a single AO write command.
    """
    def __init__(self):
        # initialize values
        self.productID = 0x0007a  # USB-1208LS
        self.port_a_bit_config = {"0": 0, "1": 0, "2": 0, "3": 0, "4": 0, "5": 0, "6": 0, "7": 0}
        self.port_b_bit_config = {"0": 1, "1": 1, "2": 1, "3": 1, "4": 1, "5": 1, "6": 1, "7": 1}
        self.AO_channels = {"0": 0, "1": 0}  # value (uint16) in counts to output [10-bits 0-5V]

        # set default configuration
        self.DConfig(usb_1208LS.DIO_PORTA, 0x00)  # Port A output
        self.DConfig(usb_1208LS.DIO_PORTB, 0xff)  # Port B input
        self.DOut(usb_1208LS.DIO_PORTA, 0x0)
        self.AOut(0, 0x0)
        self.AOut(1, 0x0)

    def getManufacturer(self):
        return "MCC"

    def getProduct(self):
        return "USB-1208LS"

    def getSerialNumber(self):
        return "021B0CB8"

    def DConfig(self, port_number, bit_mask):
        """
        This command sets the direction of the digital bits for a port.
        :param port_number: AUXPORT = 0x10 | Port A = 0x01 | Port B = 0x04
        :param bit_mask (int:bit value): 0 = output | 1 = input
        """
        if self.productID == 0x0075 and port_number == usb_1208LS.DIO_AUXPORT:
            bit_mask = ((bit_mask ^ 0xff) & 0xff)

        self.DOut(port_number, bit_mask)

    def DIn(self, port_number):
        """
        :param port_number: AUXPORT = 0x10 | Port A = 0x01 | Port B = 0x04
        :return: the value seen at the port pins
        """
        ret_val = "0b"
        DIO_port = self._return_port_config(port_number)

        for bit in DIO_port.keys():
            ret_val += str(DIO_port[bit])

        return int(ret_val, 2)

    def DOut(self, port_number, value):
        """
        This command writes data to the DIO port bits that are configured as outputs.
        :param port_number: AUXPORT = 0x10 | Port A = 0x01 | Port B = 0x04
        :param value: value to write to the port (0-255)
        """
        if value < 0 or value > 255:
            raise ValueError("Value to set is not between 0 and 255")

        DIO_port = self._return_port_config(port_number)
        in_val = format(value, '#010b')
        in_val = in_val.replace("0b", "")

        for num, c in enumerate(in_val[::-1]):
            DIO_port[str(num)] = int(c)

    def DBitIn(self, port_number, bit):
        """
        This command reads an individual digital port bit.  It will return the
        value seen at the port pin, so may be used for an input or output bit.
        :param port_number: AUXPORT = 0x10 | Port A = 0x01 | Port B = 0x04
        :param bit: the bit to read (0-7)
        :return (int): value 0 or 1
        """
        if bit < 0 or bit > 7:
            raise ValueError("Bit value is not between 0 and 7")

        DIO_port = self._return_port_config(port_number)

        return DIO_port[str(bit)]

    def DBitOut(self, port_number, bit, value):
        """
        This command writes an individual digital port bit.
        :param port_number: AUXPORT = 0x10 | Port A = 0x01 | Port B = 0x04
        :param bit: the bit to read (0-7)
        :param value: the value to write to the bit (0 or 1)
        """
        if bit < 0 or bit > 7:
            raise ValueError("Bit value is not between 0 and 7")
        if value < 0 or value > 1:
            raise ValueError("Value to set should be either 0 or 1")

        DIO_port = self._return_port_config(port_number)
        DIO_port[str(bit)] = value

    def AOut(self, channel, value):
        """
        This command sets the voltage output of the specified analog output channel
        :param channel: selects output channel (0 or 1)
        :param value: value (uint16) in counts to output [10-bits 0-5V]
        """
        channel = 0 if channel > 1 or channel < 0 else None
        # force automatic clipping
        if (value > 0x3ff):
            value = 0x3ff
        if (value < 0):
            value = 0
        self.AO_channels[str(channel)] = value

    def _return_port_config(self, port_number):
        if port_number == usb_1208LS.DIO_PORTB:
            return self.port_b_bit_config
        else:
            # Port A as default
            return self.port_a_bit_config
