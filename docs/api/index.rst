Public API
==========

.. automodule:: cubemarspycan
   :no-members:

Everything below is importable directly from ``cubemarspycan``. The headings are the
modules where each name is *defined*, which is also the only place it is documented — see
the note on :doc:`motors` for why that matters.

Motors and transport
--------------------

.. currentmodule:: cubemarspycan.bus

.. autosummary::

   MotorBus

.. currentmodule:: cubemarspycan.motor.mit

.. autosummary::

   MitMotor
   MitReplyMode

.. currentmodule:: cubemarspycan.motor.servo

.. autosummary::

   ServoMotor

.. currentmodule:: cubemarspycan.transport.can_bus

.. autosummary::

   CanTransport

The motor data model
--------------------

.. currentmodule:: cubemarspycan.spec

.. autosummary::

   MotorSpec
   Sourced
   Source
   FieldRange
   PhysicalLimits
   Drivetrain
   Capabilities
   MitFields
   ServoScaling
   Side
   WrapMode

State and faults
----------------

.. currentmodule:: cubemarspycan.state

.. autosummary::

   MitState
   ServoStatus
   ServoEvent
   FaultEvent

.. currentmodule:: cubemarspycan.faults

.. autosummary::

   CanFault
   SerialFault
   describe_can_fault

Policy, frames and tracking
---------------------------

.. currentmodule:: cubemarspycan.policy

.. autosummary::

   SafetyPolicy
   ClampMode
   ClampReport
   FaultAction

.. currentmodule:: cubemarspycan.frame

.. autosummary::

   Frame

.. currentmodule:: cubemarspycan.unwrap

.. autosummary::

   TurnCounter

.. currentmodule:: cubemarspycan.codec.servo_can

.. autosummary::

   OriginMode

Exceptions
----------

.. currentmodule:: cubemarspycan.errors

.. autosummary::

   CubemarsError
   MotorError
   MotorFault
   StaleFeedbackError
   NotInControlMode
   CapabilityError
   ProtocolError
   MalformedFrame
   ServoModeNotConfirmed
   SpecError
   SpecIncompleteError
   UnresolvedFrameError
   TransportError
   SendFailed
   UnsupportedPlatform

The registry
------------

.. currentmodule:: cubemarspycan.registry

.. autosummary::

   models
   variants

``get_spec`` is the public name for the registry's ``get``; the alias exists because a
bare ``get`` reads badly at a call site. It is documented here rather than under
:doc:`core`, so that there is exactly one entry for it.

.. autofunction:: cubemarspycan.get_spec
