"""Driver fault codes.

The manual defines **two incompatible fault tables** and they must never be mixed:

* :class:`CanFault` - the 0-7 code in byte 7 of a CAN feedback frame (manual v1.0.18 p.45),
  used by both MIT and servo-over-CAN.
* :class:`SerialFault` - the much longer ``mc_fault_code`` enum returned by the serial
  ``GET_VALUES`` reply (p.52), where 1 means over-voltage rather than over-temperature.

Serial is out of scope for v1; :class:`SerialFault` is defined anyway so that nobody
later reaches for :class:`CanFault` on the serial path, which is the mistake the table
layout invites.
"""

from __future__ import annotations

from enum import IntEnum


class CanFault(IntEnum):
    """Fault code from a CAN feedback frame (MIT and servo). Manual v1.0.18 p.45."""

    NONE = 0
    MOTOR_OVER_TEMPERATURE = 1
    OVER_CURRENT = 2
    OVER_VOLTAGE = 3
    UNDER_VOLTAGE = 4
    ENCODER = 5
    MOSFET_OVER_TEMPERATURE = 6
    MOTOR_STALL = 7


_CAN_FAULT_TEXT = {
    CanFault.NONE: "no fault",
    CanFault.MOTOR_OVER_TEMPERATURE: "motor over-temperature",
    CanFault.OVER_CURRENT: "over-current",
    CanFault.OVER_VOLTAGE: "over-voltage",
    CanFault.UNDER_VOLTAGE: "under-voltage",
    CanFault.ENCODER: "encoder fault",
    CanFault.MOSFET_OVER_TEMPERATURE: "MOSFET over-temperature",
    CanFault.MOTOR_STALL: "motor stall",
}


def describe_can_fault(code: int) -> tuple[CanFault | None, str]:
    """Map a raw fault byte to ``(enum_or_None, human_text)``.

    Never raises, never throws ``KeyError``. Codes outside 0-7 are reported verbatim
    rather than crashing the receive path. TMotorCANControl indexes a dict directly here
    and dies with ``KeyError: 7`` on the motor-stall code that v1.0.18 added.
    """
    try:
        fault = CanFault(code)
    except ValueError:
        return None, f"unknown fault code {code} (not defined in manual v1.0.18)"
    return fault, _CAN_FAULT_TEXT[fault]


class SerialFault(IntEnum):
    """``mc_fault_code`` from the serial GET_VALUES reply. Manual v1.0.18 p.52.

    NOT interchangeable with :class:`CanFault`. Reserved for v1.1.
    """

    NONE = 0
    OVER_VOLTAGE = 1
    UNDER_VOLTAGE = 2
    DRV = 3
    ABS_OVER_CURRENT = 4
    OVER_TEMP_FET = 5
    OVER_TEMP_MOTOR = 6
    GATE_DRIVER_OVER_VOLTAGE = 7
    GATE_DRIVER_UNDER_VOLTAGE = 8
    MCU_UNDER_VOLTAGE = 9
    BOOTING_FROM_WATCHDOG_RESET = 10
    ENCODER_SPI = 11
    ENCODER_SINCOS_BELOW_MIN_AMPLITUDE = 12
    ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE = 13
    FLASH_CORRUPTION = 14
    HIGH_OFFSET_CURRENT_SENSOR_1 = 15
    HIGH_OFFSET_CURRENT_SENSOR_2 = 16
    HIGH_OFFSET_CURRENT_SENSOR_3 = 17
    UNBALANCED_CURRENTS = 18
