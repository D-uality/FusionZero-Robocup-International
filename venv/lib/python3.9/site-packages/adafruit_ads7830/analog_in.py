# SPDX-FileCopyrightText: 2023 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
:py:class:`~adafruit_ads7830.analog_in.AnalogIn`
======================================================
AnalogIn for ADC readings.

* Author(s): Liz Clark

"""

from adafruit_ads7830.ads7830 import ADS7830


class AnalogIn:
    """AnalogIn Mock Implementation for ADC Reads.

    :param ADS7830 adc: The ADC object.
    :param int pin: Required pin for reading.
    """

    def __init__(self, adc: ADS7830, pin: int) -> None:
        if not isinstance(adc, ADS7830):
            raise ValueError("ADC object is from the ADS7830 class.")
        self._adc = adc
        self._pin = pin

    @property
    def value(self) -> int:
        """Returns the value of an ADC pin as an integer in the range [0, 65535]."""
        result = self._adc.read(self._pin)
        return result
