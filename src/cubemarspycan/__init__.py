"""cubemarsPyCAN - CAN control for CubeMars AK-series actuators.

Importing this package pulls in ``python-can`` and nothing else. In particular it never
imports ``pyserial``, which lives behind the ``[slcan]`` extra.

Every constant carries its own provenance - the manual page, the datasheet, or the bench
run it came from - readable through :class:`~cubemarspycan.spec.Sourced` and printed by
``cubemars dump-spec``.
"""

from __future__ import annotations

from . import servo
from .bus import MotorBus
from .codec.servo_can import OriginMode
from .errors import (
    CapabilityError,
    CubemarsError,
    MalformedFrame,
    MotorError,
    MotorFault,
    NotInControlMode,
    ProtocolError,
    SendFailed,
    ServoModeNotConfirmed,
    SpecError,
    SpecIncompleteError,
    StaleFeedbackError,
    TransportError,
    UnresolvedFrameError,
    UnsupportedPlatform,
)
from .faults import CanFault, SerialFault, describe_can_fault
from .frame import Frame
from .motor import MitMotor, MitReplyMode, ServoMotor
from .policy import ClampMode, ClampReport, FaultAction, SafetyPolicy
from .registry import SPECS, models, variants
from .registry import get as get_spec
from .spec import (
    Capabilities,
    Drivetrain,
    FieldRange,
    MitFields,
    MotorSpec,
    PhysicalLimits,
    ServoScaling,
    Side,
    Source,
    Sourced,
)
from .state import FaultEvent, MitState, ServoEvent, ServoStatus
from .transport.can_bus import CanTransport
from .unwrap import TurnCounter, WrapMode

__version__ = "0.1.0.dev0"

__all__ = [
    "SPECS",
    "CanFault",
    "CanTransport",
    "Capabilities",
    "CapabilityError",
    "ClampMode",
    "ClampReport",
    "CubemarsError",
    "Drivetrain",
    "FaultAction",
    "FaultEvent",
    "FieldRange",
    "Frame",
    "MalformedFrame",
    "MitFields",
    "MitMotor",
    "MitReplyMode",
    "MitState",
    "MotorBus",
    "MotorError",
    "MotorFault",
    "MotorSpec",
    "NotInControlMode",
    "OriginMode",
    "PhysicalLimits",
    "ProtocolError",
    "SafetyPolicy",
    "SendFailed",
    "SerialFault",
    "ServoEvent",
    "ServoModeNotConfirmed",
    "ServoMotor",
    "ServoScaling",
    "ServoStatus",
    "Side",
    "Source",
    "Sourced",
    "SpecError",
    "SpecIncompleteError",
    "StaleFeedbackError",
    "TransportError",
    "TurnCounter",
    "UnresolvedFrameError",
    "UnsupportedPlatform",
    "WrapMode",
    "__version__",
    "describe_can_fault",
    "get_spec",
    "models",
    "servo",
    "variants",
]
