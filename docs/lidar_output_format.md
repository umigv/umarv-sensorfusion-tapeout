# Lidar output: data format and timing

For whoever writes the program on the computer that receives the chip's lidar output and
publishes it to ROS.

**Status: draft, version 2 of the format.** The part of the chip that produces these frames is
not built yet, so details can still change. The version byte in every frame tells you which
layout you have. Items still open are listed at the end.

## What the chip sends

The VLP-16 plugs into the board instead of the computer. The chip converts each lidar return to
x, y, z, corrects it for the robot's motion using the pose from its Kalman filter, and sends the
result to the computer as ordinary UDP packets over 100 Mbit Ethernet.

```
VLP-16 --Ethernet--> board (chip) --Ethernet--> computer --> your program --> ROS topics
```

You do not need a raw socket or a special driver. Open a normal UDP socket on the destination
port and read datagrams.

| Setting | Default (proposed) | Notes |
|---|---|---|
| Destination IP | 255.255.255.255 (broadcast) | settable on the chip |
| Destination UDP port | 2370 | settable |
| Source IP, port | 192.168.1.50, 2370 | settable |
| UDP checksum | 0 (not used) | allowed for IPv4; do not reject on it |

## Datagram layout

All fields are little-endian. One datagram is a 40-byte header followed by the points.

### Header (40 bytes)

| Offset | Size | Type | Field | Meaning |
|---|---|---|---|---|
| 0 | 4 | bytes | `magic` | ASCII `PC2X` |
| 4 | 1 | uint8 | `version` | 2 |
| 5 | 1 | uint8 | `flags` | see below |
| 6 | 2 | uint16 | `num_points` | points in this datagram, normally 192 |
| 8 | 4 | uint32 | `seq` | counts datagrams, wraps; a gap means a datagram was lost |
| 12 | 4 | uint32 | `time_us` | chip time of this datagram's first firing, microseconds |
| 16 | 4 | int32 | `ref_x` | reference position x, metres x 65536 |
| 20 | 4 | int32 | `ref_y` | reference position y, metres x 65536 |
| 24 | 4 | int32 | `ref_theta` | robot heading at `time_us`, radians x 65536 |
| 28 | 4 | int32 | `v` | forward speed, metres per second x 65536 |
| 32 | 4 | int32 | `omega` | turn rate, radians per second x 65536 |
| 36 | 2 | uint16 | `first_azimuth` | lidar's own azimuth of the first firing, 0.01 degree, 0 to 35999 |
| 38 | 1 | uint8 | `first_firing` | which firing of the lidar packet comes first: 0 or 12 |
| 39 | 1 | uint8 | `point_step` | bytes per point: 6 |

To get real units, divide the `int32` values by 65536.

### Flags

| Bit | Name | Meaning when 1 |
|---|---|---|
| 0 | `pose_valid` | the Kalman filter has produced a pose. When 0, points are not motion-corrected and the pose fields are zero |
| 1 | `scale_4mm` | one count is 4 mm. When 0, one count is 2 mm |
| 2 | `prev_bad` | the previous lidar packet arrived corrupted; discard the previous two datagrams |
| 3 | `overflow` | the chip dropped points since the last datagram |
| 4-7 | | reserved, ignore |

### Points (`num_points` x `point_step` bytes)

| Offset in point | Size | Type | Field |
|---|---|---|---|
| 0 | 2 | int16 | `x` |
| 2 | 2 | int16 | `y` |
| 4 | 2 | int16 | `z` |

- Multiply by the scale from the flags (0.002 or 0.004) to get metres.
- **A point with x = y = z = -32768 is "no return". Skip it.** Empty returns are kept in the
  stream so that position in the datagram always tells you which laser a point came from.
- Point `i` came from laser `i % 16` (the VLP-16's laser ID, which is the "ring") and from
  firing `first_firing + i // 16` of the lidar packet.
- Read `point_step` from the header and step by it. If intensity is added later, the points get
  longer and the first six bytes stay the same.

## What the coordinates mean

Each point has already been placed using the robot's pose **at the moment that laser fired**, so
a turn during the sweep does not smear the cloud. The values are in the Kalman filter's world
frame, measured from the reference position in the header:

```
world_x = ref_x + x * scale
world_y = ref_y + y * scale
world_z =         z * scale          (height relative to the lidar)
```

The offset is there because 16 bits only cover +/-65 m (or +/-131 m at 4 mm), and the robot may be
further than that from its origin.

If you would rather publish the cloud in the robot's frame, as most ROS navigation code expects,
rotate the points back by the heading. With `c = cos(ref_theta)` and `s = sin(ref_theta)`:

```
robot_x =  c * (x * scale) + s * (y * scale)
robot_y = -s * (x * scale) + c * (y * scale)
robot_z =       z * scale
```

That gives a motion-corrected scan as seen from the robot at time `time_us`. Publish it with that
timestamp, and publish the pose from the header as odometry or a transform with the same stamp.

## Timing

| Quantity | Value |
|---|---|
| Lidar packets in | about 754 per second, one every 1.33 ms |
| Datagrams out | 2 per lidar packet, about 1,507 per second |
| Points per datagram | 192 (12 firings x 16 lasers) |
| Datagram size | 40 + 1,152 = 1,192 bytes of UDP payload |
| Point rate | about 289,000 per second, including "no return" entries |
| Data rate | about 15 Mbit/s |
| One full rotation at 600 RPM | 100 ms, about 151 datagrams, about 29,000 points |
| Pose behind the header | updated 400 times per second, then extrapolated to each firing |
| Delay added by the chip | expected under 1 ms; not measured yet |

### Building a full sweep

`first_azimuth` increases through a rotation and drops back toward 0 when the lidar passes its
zero mark. Collect datagrams until `first_azimuth` is smaller than in the previous datagram, then
publish what you have as one `sensor_msgs/PointCloud2`.

### Timestamps

`time_us` is the chip's own clock. It starts at 0 when the chip powers up and wraps every 71.6
minutes. It is not the computer's time.

- For time differences (between datagrams, or between a datagram and its pose), use it directly
  with unsigned 32-bit subtraction, which stays correct across the wrap.
- For a ROS timestamp, keep a running estimate of `computer_receive_time - time_us` (the smallest
  value seen recently is the best estimate, since network delay only ever adds) and add it to
  `time_us`.

### Lost data

UDP does not retransmit. Use `seq` to count gaps. A lost datagram is 192 missing points out of
about 29,000 in a sweep, so publish the sweep anyway.

## Bypass mode

If the chip's lidar block is switched to bypass, it forwards the VLP-16's own packets unchanged
(UDP port 2368), and the standard Velodyne ROS driver works as if the lidar were plugged straight
into the computer. This is the fallback if the correction has a problem.

## Minimal receiver (Python)

```python
import socket, struct
import numpy as np

HDR = struct.Struct("<4sBBHIIiiiiiHBB")          # 40 bytes

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", 2370))

while True:
    data, _ = sock.recvfrom(2048)
    (magic, version, flags, n, seq, time_us, ref_x, ref_y, ref_theta,
     v, omega, first_az, first_firing, step) = HDR.unpack_from(data)
    if magic != b"PC2X" or version != 2:
        continue
    scale = 0.004 if flags & 0x02 else 0.002
    raw = np.frombuffer(data, dtype=np.uint8, offset=HDR.size, count=n * step).reshape(n, step)
    xyz = raw[:, :6].copy().view("<i2")           # n x 3 int16
    keep = ~np.all(xyz == -32768, axis=1)          # drop "no return"
    ring = (np.arange(n) % 16)[keep]
    pts = xyz[keep].astype(np.float32) * scale     # metres, relative to the reference position
    pts[:, 0] += ref_x / 65536.0                   # into the world frame
    pts[:, 1] += ref_y / 65536.0
    # ... collect until first_az wraps, then publish a PointCloud2
```

`genesys2/host/pc2x_bridge.py` in the UMARV-SensorFusion repository is a complete ROS bridge for
version 1 of this format (the FPGA prototype) and is a good starting point.

## Still open

| Item | Why it matters to you |
|---|---|
| Which way the world axes point, and which way heading is positive | decides how the cloud lines up with the map. Proposed: x east, y north, heading counter-clockwise from x, as in ROS REP-103 |
| Whether intensity is sent | if yes, `point_step` becomes 7 or 8 and each point gains an intensity byte |
| The default addresses and port | may change to suit the robot's network |
| Whether zero position on z should be the lidar or the ground | currently the lidar |
| Dual-return mode | not supported; the VLP-16 must be set to single return |

Tell the chip team if any of these would make your side harder. They are cheap to change now and
impossible after the chip is made.
