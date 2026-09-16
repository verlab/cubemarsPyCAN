# CAN setup

AK drivers run at **1 Mbit/s**. The manual says changing it is not recommended.

## Linux (production)

socketcan is the production path. The kernel owns the bitrate — python-can cannot set it —
so bring the interface up first:

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
```

Then, with no privileges:

```python
from cubemarspycan import CanTransport, MotorBus

with CanTransport.open("socketcan:can0") as tp, MotorBus(tp) as bus:
    ...
```

`CanTransport` reads the configured bitrate back from
`/sys/class/net/can0/can_bittiming/bitrate` and warns if it is not 1 Mbit/s, because a
mismatch looks exactly like a wiring fault.

`cubemars doctor` lists your interfaces, their state and their bitrates, and prints the
bring-up lines. **It never runs them.** This library does not shell out to `sudo`, ever.

### Persisting it

With systemd-networkd, `/etc/systemd/network/80-can.network`:

```ini
[Match]
Name=can0

[CAN]
BitRate=1M
```

### No hardware? Use vcan

A virtual CAN interface behaves like a real one, which is how this project's CI exercises
the socketcan code path:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
cubemars scan --url socketcan:vcan0
```

## macOS and Windows (development)

socketcan is a Linux kernel facility and does not exist elsewhere. Asking for it raises
`UnsupportedPlatform` with that explanation rather than a confusing `OSError`.

Use a USB-CAN adapter:

```bash
pip install "cubemarspycan[slcan]"
cubemars scan --url "slcan:/dev/tty.usbmodem1101@1M"

pip install "cubemarspycan[gs-usb]"      # CANable / candleLight native
cubemars scan --url "gs_usb:0@1M"
```

### Loop rates on slcan

An slcan send is an ASCII line over a USB CDC endpoint: typically 0.5–2 ms, with macOS
scheduling spikes into the tens of milliseconds. Plan for **200–500 Hz**; 1 kHz needs
gs_usb or Linux socketcan.

`MotorBus.transport.stats.tx_percentiles()` reports p50/p95/max send time in milliseconds,
so this is measurable rather than mysterious.

## Bringing your own bus

The URL forms are sugar. Any `can.BusABC` works:

```python
import can
from cubemarspycan import CanTransport, MotorBus

bus = can.interface.Bus(channel="can0", interface="socketcan", fd=False)
with MotorBus(CanTransport(bus)) as motor_bus:
    ...
```

`CanTransport` only shuts down buses it opened itself.
