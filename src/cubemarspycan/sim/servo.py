"""A fake AK driver speaking servo mode over CAN.

Like the MIT sim, it decodes with its own ``struct`` calls rather than the library codec.

Two behaviours here exist because they are the traps a real bench session hits:

* ``status_rate_hz=0`` reproduces a driver whose CAN status messages are disabled in
  CubeMarsTool. Nothing is wrong with the wiring, no frames ever arrive, and a naive
  library reports zeros forever.
* ``reject_permanent`` reproduces a single-encoder motor refusing origin mode 1, so the
  library's capability check is validated end to end rather than only at its own boundary.
"""

from __future__ import annotations

import math
import struct

from ..codec.servo_can import (
    SERVO_MODE_ACK_PAYLOAD,
    OriginMode,
    ServoFunction,
    ServoPacket,
    arbitration_id,
    split_arbitration_id,
)
from ..frame import Frame
from ..spec import MotorSpec
from .plant import FIRMWARE_LOOP_DT, Plant


class SimServoDriver:
    """Servo-mode driver with a periodic status upload."""

    def __init__(
        self,
        spec: MotorSpec,
        motor_id: int = 1,
        plant: Plant | None = None,
        *,
        status_rate_hz: float = 100.0,
        ack_on_first_command: bool = True,
        reject_permanent: bool | None = None,
        temperature_c: int = 30,
        fault_code: int = 0,
    ) -> None:
        self.spec = spec
        self.motor_id = motor_id
        self.plant = plant if plant is not None else Plant()
        self.status_rate_hz = status_rate_hz
        self.ack_on_first_command = ack_on_first_command
        self.reject_permanent = (
            not spec.capabilities.permanent_zero if reject_permanent is None else reject_permanent
        )
        self.temperature_c = temperature_c
        self.fault_code = fault_code

        self.commands_seen = 0
        self.last_packet: ServoPacket | None = None
        self.last_values: tuple[float, ...] = ()
        self.origin_calls: list[int] = []
        self.rejected_origin_calls = 0
        self.acked = False
        self._torque = 0.0
        self._since_status = 0.0
        self._target_deg: float | None = None
        self._target_erpm: float | None = None

    # --- protocol -------------------------------------------------------------------

    def handle(self, frame: Frame) -> list[Frame]:
        if not frame.is_extended_id:
            return []
        packet_id, motor_id = split_arbitration_id(frame.arbitration_id)
        if motor_id != self.motor_id:
            return []
        try:
            packet = ServoPacket(packet_id)
        except ValueError:
            return []

        replies: list[Frame] = []
        if self.ack_on_first_command and not self.acked:
            self.acked = True
            replies.append(
                Frame(
                    arbitration_id(ServoFunction.SERVO_MODE_ACK, self.motor_id),
                    SERVO_MODE_ACK_PAYLOAD,
                    is_extended_id=True,
                )
            )

        self.commands_seen += 1
        self.last_packet = packet
        data = frame.data
        scaling = self.spec.servo

        if packet is ServoPacket.SET_ORIGIN and len(data) >= 1:
            mode = data[0]
            if mode == OriginMode.PERMANENT and self.reject_permanent:
                self.rejected_origin_calls += 1
            else:
                self.origin_calls.append(mode)
                self.plant.zero_here()
            self.last_values = (float(mode),)
            return replies

        if packet is ServoPacket.SET_POS_SPD and len(data) == 8:
            pos, spd, acc = struct.unpack(">ihh", data)
            self.last_values = (
                pos / scaling.pos_spd_position_scale,
                spd * scaling.pos_spd_speed_divisor,
                acc * scaling.pos_spd_accel_divisor,
            )
            self._target_deg = self.last_values[0]
            self._target_erpm = None
            return replies

        if len(data) == 4:
            (raw,) = struct.unpack(">i", data)
            if packet is ServoPacket.SET_DUTY:
                self.last_values = (raw / scaling.duty_scale,)
                self._torque = self.last_values[0] * 2.0
                self._target_deg = self._target_erpm = None
            elif packet in (ServoPacket.SET_CURRENT, ServoPacket.SET_CURRENT_BRAKE):
                amps = raw / scaling.current_scale
                self.last_values = (amps,)
                kt = self.spec.drivetrain.kt_nm_per_a
                gear = self.spec.drivetrain.gear_ratio
                if kt.known and gear.known:
                    self._torque = amps * float(kt.value or 0.0) * float(gear.value or 1.0)
                self._target_deg = self._target_erpm = None
            elif packet is ServoPacket.SET_RPM:
                self.last_values = (float(raw),)
                self._target_erpm = float(raw)
                self._target_deg = None
            elif packet is ServoPacket.SET_POS:
                self.last_values = (raw / scaling.position_scale,)
                self._target_deg = self.last_values[0]
                self._target_erpm = None
        return replies

    def _inner_loop_torque(self) -> float:
        """The driver's own position or speed loop.

        Servo mode closes these inside the driver, off the last setpoint received, at a
        rate far above any CAN command rate. Modelling it as a one-shot per frame would
        make the simulator ring at the command period rather than behave like the motor.
        """
        if self._target_deg is not None:
            error_rad = (self._target_deg * math.pi / 180.0) - self.plant.position
            return max(-4.0, min(4.0, 30.0 * error_rad - 1.0 * self.plant.velocity))
        if self._target_erpm is not None and self.spec.drivetrain.pole_pairs.known:
            target = self.spec.erpm_to_radps_output(self._target_erpm)
            return max(-4.0, min(4.0, 1.0 * (target - self.plant.velocity)))
        return self._torque

    def step(self, dt: float) -> list[Frame]:
        """Advance the plant and emit any due status frames."""
        steps = max(1, int(dt / FIRMWARE_LOOP_DT + 0.5))
        for _ in range(steps):
            self._torque = self._inner_loop_torque()
            self.plant.step(dt / steps, self._torque)
        if self.status_rate_hz <= 0.0:
            return []
        self._since_status += dt
        period = 1.0 / self.status_rate_hz
        frames: list[Frame] = []
        while self._since_status >= period:
            self._since_status -= period
            frames.append(self.status_frame())
        return frames

    def status_frame(self) -> Frame:
        scaling = self.spec.servo
        degrees = self.plant.position * 180.0 / 3.141592653589793
        erpm = 0.0
        if self.spec.drivetrain.pole_pairs.known and self.spec.drivetrain.gear_ratio.known:
            erpm = self.spec.radps_output_to_erpm(self.plant.velocity)
        current = 0.0
        kt = self.spec.drivetrain.kt_nm_per_a
        gear = self.spec.drivetrain.gear_ratio
        if kt.known and gear.known:
            current = self._torque / (float(kt.value or 1.0) * float(gear.value or 1.0))

        pos_raw = _clamp16(round(degrees / scaling.feedback_deg_per_lsb))
        spd_raw = _clamp16(round(erpm / scaling.feedback_erpm_per_lsb))
        cur_raw = _clamp16(round(current / scaling.feedback_amps_per_lsb))
        return Frame(
            arbitration_id(ServoFunction.STATUS, self.motor_id),
            struct.pack(
                ">hhhbB",
                pos_raw,
                spd_raw,
                cur_raw,
                max(-128, min(127, self.temperature_c)),
                self.fault_code & 0xFF,
            ),
            is_extended_id=True,
        )

    def bootloader_frame(self) -> Frame:
        return Frame(
            arbitration_id(ServoFunction.BOOTLOADER_JUMP, self.motor_id),
            bytes(8),
            is_extended_id=True,
        )


def _clamp16(value: int) -> int:
    return max(-32768, min(32767, value))
