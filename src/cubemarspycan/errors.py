"""Exception hierarchy.

Every exception in this module is raised on the **caller's** thread. Nothing in the
receive path ever raises: a driver fault becomes a latched
:class:`~cubemarspycan.state.FaultEvent`, and only ``update()`` turns it into control
flow. That is the fix for the single most safety-relevant defect in TMotorCANControl,
where a fault raised inside the python-can notifier thread never reached the control loop
and the motor kept being commanded (PLAN.md 2.6).
"""

from __future__ import annotations

# ruff: noqa: N818
# Several names here deliberately omit the "Error" suffix. They are domain vocabulary and
# read better at the call site -- `except MotorFault:` and `except MalformedFrame:` say
# what happened; `MotorFaultError` does not say more. These names are public API.


class CubemarsError(Exception):
    """Base class for every error this library raises."""


# --- spec / configuration ----------------------------------------------------------


class SpecError(CubemarsError):
    """The motor specification cannot support what was asked of it."""


class SpecIncompleteError(SpecError):
    """A conversion needs a constant this spec does not know.

    Raised instead of guessing. Guessing is what produced the 8.6x position error and the
    78x torque error in the reference library.
    """


class CapabilityError(SpecError):
    """This motor variant does not support the requested operation.

    For example, permanent-zero (origin mode 1) on a single-encoder model such as the
    AK40-10. No frame is emitted when this is raised.
    """


class UnresolvedFrameError(SpecError):
    """The value exists on the wire but which side of the gearbox it refers to is unknown."""


# --- protocol ----------------------------------------------------------------------


class ProtocolError(CubemarsError):
    """Something on the wire did not match the protocol."""


class MalformedFrame(ProtocolError):
    """A frame could not be decoded. The only exception a codec may raise."""


# --- transport ---------------------------------------------------------------------


class TransportError(CubemarsError):
    """The CAN link failed."""


class UnsupportedPlatform(TransportError):
    """The requested backend does not exist on this platform (e.g. socketcan on macOS)."""


class SendFailed(TransportError):
    """A frame could not be put on the bus."""


# --- motor lifecycle ---------------------------------------------------------------


class MotorError(CubemarsError):
    """Base for runtime problems with a specific motor."""


class MotorFault(MotorError):
    """The driver reported a non-zero fault code.

    Raised on the control thread by ``update()``, after a safe-stop frame has been sent.
    """


class StaleFeedbackError(MotorError):
    """No fresh feedback within the configured window; the motor may be gone."""


class NotInControlMode(MotorError):
    """Commanded a motor that is not inside its ``control()`` block."""


class ServoModeNotConfirmed(MotorError):
    """The driver never confirmed it is in servo mode.

    Carries triage, because the usual cause is not wiring: CubeMarsTool defaults the CAN
    status-message rate to 0 on some drivers, in which case no 0x29 frames are ever sent
    and a naive library reports zeros forever.
    """
