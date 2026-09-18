cubemarsPyCAN
=============

Correct, tested CAN control for CubeMars AK-series actuators, in MIT and servo modes.

Every constant is traceable to the *AK Series Module Driver Manual* v1.0.18, a CubeMars
datasheet, or a bench measurement. See :class:`~cubemarspycan.spec.Sourced`.

.. code-block:: python

   from cubemarspycan import SPECS, CanTransport, MitMotor, MotorBus

   with CanTransport.open("socketcan:can0") as tp, MotorBus(tp) as bus:
       m = MitMotor(bus, motor_id=1, spec=SPECS["AK40-10"], supply_voltage=24.0)
       with m.control():
           m.zero_here()
           m.settle(1.5)
           state = m.update(position=0.5, velocity=0.0, kp=20.0, kd=0.5, torque=0.0)

.. toctree::
   :maxdepth: 2
   :caption: Guides

   can-setup
   cli
   units
   ak-2-0
   adding-a-motor
   migration
   troubleshooting
   bench

.. toctree::
   :maxdepth: 2
   :caption: API reference

   api/index
   api/motors
   api/core
   api/transport
   api/codec
   api/sim
   api/cli

Indices
-------

* :ref:`genindex`
* :ref:`modindex`
