"""A minimal mechanical plant, output-side.

``J * omega_dot = tau - b * omega - tau_load``, integrated semi-implicitly.

It exists so the simulator produces motion that is *plausible* rather than scripted: a
position step overshoots, velocity is genuinely non-zero, and a bad unwrapper or a sign
error has something to fail against. A simulator that always reported zero velocity would
pass tests that should fail.
"""

from __future__ import annotations

from dataclasses import dataclass

# AK40-10: 97.35 g*cm^2 rotor inertia, reflected through 10:1 is ~9.7e-4 kg*m^2 at the
# output. Rounded, plus a little for the gearbox and a token load.
DEFAULT_INERTIA = 1.0e-3
DEFAULT_DAMPING = 2.0e-3

FIRMWARE_LOOP_DT = 1.0e-4
"""Inner-loop period of the fake firmware, 10 kHz.

The real driver closes its impedance loop internally at tens of kilohertz off the last
setpoint it received - it does *not* recompute only when a CAN frame arrives. Modelling
it the naive way makes the simulator ring at the control period and turns every gain test
into a measurement of the integrator instead of the protocol."""


@dataclass
class Plant:
    """Rigid single-axis plant at the output shaft."""

    inertia: float = DEFAULT_INERTIA
    damping: float = DEFAULT_DAMPING
    position: float = 0.0
    velocity: float = 0.0
    load_torque: float = 0.0
    """Constant external torque, e.g. gravity on a lever."""
    limit_lo: float | None = None
    """Hard stop in the negative direction, rad. ``None`` for free travel."""
    limit_hi: float | None = None
    """Hard stop in the positive direction, rad."""

    def step(self, dt: float, torque: float) -> None:
        """One explicit Euler step. Callers choose the rate; see FIRMWARE_LOOP_DT."""
        if dt <= 0.0:
            return
        accel = (torque - self.damping * self.velocity - self.load_torque) / self.inertia
        self.velocity += accel * dt
        self.position += self.velocity * dt
        # Inelastic hard stops: the mechanism reaches the end of its travel and the
        # motor pushes against it. That is what a homing routine looks for.
        if self.limit_hi is not None and self.position >= self.limit_hi:
            self.position = self.limit_hi
            self.velocity = min(self.velocity, 0.0)
        if self.limit_lo is not None and self.position <= self.limit_lo:
            self.position = self.limit_lo
            self.velocity = max(self.velocity, 0.0)

    @property
    def at_limit(self) -> bool:
        """Whether the position is resting on either hard stop.

        True while a stop is being pushed against, which is the condition a homing routine
        looks for. ``limit_lo``/``limit_hi`` default to ``None``, so a plant with free
        travel always reports ``False``.
        """
        return (self.limit_hi is not None and self.position >= self.limit_hi) or (
            self.limit_lo is not None and self.position <= self.limit_lo
        )

    def zero_here(self) -> None:
        """Shift the origin to here, carrying any travel limits with it."""
        if self.limit_hi is not None:
            self.limit_hi -= self.position
        if self.limit_lo is not None:
            self.limit_lo -= self.position
        self.position = 0.0
