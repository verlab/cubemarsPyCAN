"""Immutable state snapshots.

Every class here is ``frozen=True, slots=True``, and that is load-bearing rather than
decorative. The receive thread builds a **new** state object per frame and never mutates
one it has published, so handing a reference back to the control thread is semantically a
copy. Tearing is therefore impossible by construction.

TMotorCANControl gets this wrong in two different ways: ``mit_can.py:774`` and
``servo_can.py:705`` copy field-by-field out of an object the receive thread is mutating
concurrently (so you can read position from frame N and velocity from frame N+1), and
``servo_serial.py:783`` rebinds the name instead of copying, collapsing the double buffer
entirely.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass

from .errors import UnresolvedFrameError
from .faults import CanFault, describe_can_fault
from .spec import MotorSpec, Side
from .units import rad_to_deg

_kt_warned: set[str] = set()


@dataclass(frozen=True, slots=True)
class FaultEvent:
    """A latched driver fault. Data, never an exception, until the control thread acts."""

    code: int
    text: str
    fault: CanFault | None
    source: str
    """"mit", "servo" or "link"."""
    rx_monotonic: float
    seq: int

    @classmethod
    def from_code(cls, code: int, source: str, rx_monotonic: float, seq: int) -> FaultEvent:
        """Build an event from a raw wire fault code, resolving its text.

        An unrecognised code is preserved verbatim with a generic description rather than
        raising - a driver reporting something this library has not seen is exactly when
        the caller most needs the number.
        """
        fault, text = describe_can_fault(code)
        return cls(code, text, fault, source, rx_monotonic, seq)

    def __str__(self) -> str:
        return f"{self.source} fault {self.code}: {self.text}"


@dataclass(frozen=True, slots=True)
class MitState:
    """One decoded MIT feedback frame.

    Position, velocity and torque are output-side. For the AK40-10 that is confirmed by
    the velocity field matching the datasheet no-load speed, not assumed.
    """

    spec: MotorSpec
    """The spec this frame was decoded against, and the source of every conversion."""
    position_rad: float
    """Angle in radians, **output-side**, as decoded from the 16-bit position field."""
    velocity_radps: float
    """Speed in rad/s, **output-side**, from the 12-bit velocity field."""
    torque_nm: float
    """Torque in N*m, **output-side**.

    The wire quantity is torque, not current: the manual's own reply decoder names it so
    and scales it by the torque field. TMotorCANControl converts it to a "q-axis current"
    through Kt, the gear ratio and an undocumented 0.59 factor and presents *that* as the
    primary reading.
    """
    temperature_c: int
    """Driver temperature in degrees Celsius, decoded as ``D6 - 40``."""
    fault_code: int
    """Raw driver fault code; 0 is healthy. See :attr:`fault_text` for a description."""
    rx_monotonic: float
    """Arrival time from :func:`time.monotonic`, stamped on the receive thread.

    Never the bus timestamp, which is epoch-based and on some backends comes from the
    driver with an unrelated origin. Staleness is measured against this.
    """
    seq: int
    """Monotonic counter, incremented once per published frame. 0 means none yet.

    A jump of more than 1 between two reads means several frames arrived while the loop
    was busy; the latch keeps the newest and nothing is lost.
    """

    # --- faults ---------------------------------------------------------------------

    @property
    def fault(self) -> CanFault | None:
        """The decoded fault, or ``None`` for code 0 **or** an unrecognised code.

        ``None`` is therefore not the same as healthy - check :attr:`is_faulted` for that,
        and :attr:`fault_text` for something printable either way.
        """
        return describe_can_fault(self.fault_code)[0]

    @property
    def fault_text(self) -> str:
        """A printable description of :attr:`fault_code`, always non-empty.

        Falls back to naming the raw code when the driver reports something this library
        does not recognise.
        """
        return describe_can_fault(self.fault_code)[1]

    @property
    def is_faulted(self) -> bool:
        """Whether the driver reported any non-zero fault code.

        The authoritative check: unlike :attr:`fault`, this is true for codes the library
        cannot name.
        """
        return self.fault_code != 0

    # --- rotor side -----------------------------------------------------------------

    @property
    def position_rotor_rad(self) -> float:
        """Rotor-side angle in radians: the output angle times the gear ratio.

        **Ten times** :attr:`position_rad` on an AK40-10. Derived, not measured: the wire
        carries the output-side value, bench-confirmed at 6.3867 rad for one hand-turn.
        Raises :class:`~cubemarspycan.errors.SpecIncompleteError` if the gear ratio is
        unknown.
        """
        gr = self.spec.drivetrain.gear_ratio.require("rotor-side position")
        return self.position_rad * gr

    @property
    def velocity_rotor_radps(self) -> float:
        """Rotor-side speed in rad/s: the output speed times the gear ratio.

        Raises :class:`~cubemarspycan.errors.SpecIncompleteError` if the gear ratio is
        unknown.
        """
        gr = self.spec.drivetrain.gear_ratio.require("rotor-side velocity")
        return self.velocity_radps * gr

    @property
    def position_deg(self) -> float:
        """Output-shaft angle in degrees. A pure unit change on :attr:`position_rad`.

        Needs no spec constant, so unlike the rotor-side properties it can never refuse.
        """
        return rad_to_deg(self.position_rad)

    # --- derived, and honest about it -----------------------------------------------

    @property
    def estimated_current_a(self) -> float:
        """q-axis current implied by the reported torque.

        The MIT reply field *is* torque; the manual's own ``unpack_reply`` names it so.
        This inverts an idealised lossless model, so it is an estimate and says so. It
        warns once per spec while Kt is not ``Source.MEASURED``.

        TMotorCANControl presents this quantity as the primary reading, having passed it
        through Kt, the gear ratio and an undocumented 0.59 fudge factor copied to every
        motor - three modelling assumptions wearing the costume of a raw measurement.
        """
        kt = self.spec.drivetrain.kt_nm_per_a
        if kt.source.value != "measured" and self.spec.name not in _kt_warned:
            _kt_warned.add(self.spec.name)
            warnings.warn(
                f"estimated_current_a for {self.spec.name} uses Kt from "
                f"{kt.source.value}, not a bench measurement, and ignores gearbox "
                f"losses. Treat it as an estimate.",
                stacklevel=2,
            )
        return self.spec.current_for_output_torque(self.torque_nm)

    def __str__(self) -> str:
        return (
            f"{self.position_rad:+8.4f} rad  {self.velocity_radps:+8.3f} rad/s  "
            f"{self.torque_nm:+7.3f} Nm  {self.temperature_c:4d} C  {self.fault_text}"
        )


@dataclass(frozen=True, slots=True)
class ServoStatus:
    """One decoded servo feedback frame (function id 0x29).

    Wire units are degrees, ERPM and amps. Which side of the gearbox the position refers
    to is not documented, so :attr:`output_rad` refuses until the spec says.
    """

    spec: MotorSpec
    """The spec this frame was decoded against, and the source of every conversion."""
    position_deg: float
    """Angle in degrees, straight off the wire.

    Always available, because it assumes nothing. Which side of the gearbox it refers to
    is undocumented, which is why :attr:`output_rad` refuses until the spec records a
    measurement.
    """
    velocity_erpm: float
    """Speed in **electrical** RPM, not mechanical. See :attr:`velocity_radps`."""
    current_a: float
    """q-axis current in amps, as reported by the driver."""
    temperature_c: int
    """Driver temperature in degrees Celsius."""
    fault_code: int
    """Raw driver fault code; 0 is healthy. See :attr:`fault_text` for a description."""
    rx_monotonic: float
    """Arrival time from :func:`time.monotonic`, stamped on the receive thread."""
    seq: int
    """Monotonic counter, incremented once per published frame. 0 means none yet."""

    @property
    def fault(self) -> CanFault | None:
        """The decoded fault, or ``None`` for code 0 **or** an unrecognised code.

        ``None`` is therefore not the same as healthy - check :attr:`is_faulted` for that,
        and :attr:`fault_text` for something printable either way.
        """
        return describe_can_fault(self.fault_code)[0]

    @property
    def fault_text(self) -> str:
        """A printable description of :attr:`fault_code`, always non-empty.

        Falls back to naming the raw code when the driver reports something this library
        does not recognise.
        """
        return describe_can_fault(self.fault_code)[1]

    @property
    def is_faulted(self) -> bool:
        """Whether the driver reported any non-zero fault code.

        The authoritative check: unlike :attr:`fault`, this is true for codes the library
        cannot name.
        """
        return self.fault_code != 0

    @property
    def velocity_radps(self) -> float:
        """Output-shaft rad/s. Needs pole pairs and gear ratio; refuses without them."""
        return self.spec.erpm_to_radps_output(self.velocity_erpm)

    @property
    def output_rad(self) -> float:
        """Output-shaft angle in radians.

        Raises until ``spec.servo.position_side`` has been established on a bench
        (step B7: rotate the output one turn; 360 deg means output-side, 3600 deg means
        rotor-side on a 10:1). :attr:`position_deg` is always available meanwhile.
        """
        side = self.spec.servo.position_side
        if not side.known:
            raise UnresolvedFrameError(
                f"servo position side is unknown for {self.spec.name}, so degrees cannot "
                f"be referred to the output shaft. Use .position_deg for the raw wire "
                f"value, or settle it with bench step B7 and record it via spec.evolve()."
            )
        rad = self.position_deg * 3.141592653589793 / 180.0
        if side.require("servo position side") is Side.ROTOR:
            rad /= self.spec.drivetrain.gear_ratio.require("servo output angle")
        return rad

    def __str__(self) -> str:
        return (
            f"{self.position_deg:+9.1f} deg  {self.velocity_erpm:+9.0f} ERPM  "
            f"{self.current_a:+7.2f} A  {self.temperature_c:4d} C  {self.fault_text}"
        )


@dataclass(frozen=True, slots=True)
class ServoEvent:
    """A servo frame that is not state.

    Function id ``0x2C`` is the "entered servo mode" handshake, payload ``FA FB FC FD``;
    ``0x09`` signals a jump to the bootloader. TMotorCANControl decodes both as position,
    turning the handshake into a bogus -128.5 degree reading.
    """

    kind: str
    """"servo_mode_ack" or "bootloader_jump"."""
    function_id: int
    payload: bytes
    rx_monotonic: float

    def __str__(self) -> str:
        return f"{self.kind} (fn 0x{self.function_id:02X})"
