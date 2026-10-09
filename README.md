# Mira Control Mavlink Node

A Go rewrite of the `pymav_master` node from
[`mira2_control_master`](https://github.com/davidnoronha1/mira/tree/master/src/mira2_control_master)
(`alt_master.py`). It talks to the Pixhawk (ArduSub) over MAVLink and is meant to be a
drop-in replacement for the Python node: same node name, topics, service,
parameters and launch arguments.

The original ROS 1 implementation is in [original.py](original.py).

The rewrite moves away from the Python node's single blocking `recv_match` loop.
That loop missed telemetry messages and used a large share of the Raspberry Pi's CPU.
Here, MAVLink reading, RC override output, arm and mode commands, and telemetry
publishing each run in their own goroutine.

## Architecture
![image](https://github.com/user-attachments/assets/e4dc372e-b92d-4a27-974f-8fb3b3c09561)

## ROS interface

| Kind | Name | Type | Notes |
|---|---|---|---|
| sub | `/master/commands` | `custom_msgs/Commands` | arm, mode and 8 PWM channels (all workspace publishers use this) |
| sub | `/rov/commands` | `custom_msgs/Commands` | same handling, last message wins (from `master.py`; nothing publishes it today) |
| pub | `/master/telemetry` | `custom_msgs/Telemetry` | same fields as `alt_master.py`, plus IMU fields from SCALED_IMU2 |
| pub | `/master/imu_ned` | `sensor_msgs/Imu` | from SCALED_IMU2 (used by `flare_imu`) |
| pub | `/master/depth` | `custom_msgs/Depth` | from `master.py` |
| pub | `/master/heading` | `custom_msgs/Heading` | from `master.py` |
| srv | `/emergency_kill` | `std_srvs/Trigger` | engages the emergency lock and force disarms; arming stays blocked until cleared |
| srv | `/clear_emergency` | `std_srvs/Trigger` | releases the emergency lock |
| srv | `/toggle_emergency` | `std_srvs/Trigger` | engages/clears the lock (used by `killswitch`) |

ARM, DISARM and emergency disarm go through a priority queue. They are sent
before any queued mode change or message-rate request, and a mode change or
message-rate request that is waiting for its ack gives way immediately.

Parameters (`--ros-args -p name:=value` or launch `<param>`):

| Parameter | Default | |
|---|---|---|
| `pixhawk_address` | `/dev/Pixhawk` | pymavlink style: serial device (`/dev/ttyACM0[:baud]`), `tcp:host:port` (SITL), `tcpin:`, `udp:`/`udpin:`, `udpout:` |
| `initial_mode` | `STABILIZE` | mode assumed at start (not sent to the Pixhawk, same as Python) |
| `baud` | `57600` | serial baud rate |
| `source_system` | `255` | MAVLink system id we send as. Must equal ArduSub's `SYSID_MYGCS` |
| `source_component` | `0` | |
| `rc_rate_hz` | `25` | RC_CHANNELS_OVERRIDE send rate |
| `stream_rate_hz` | `100` | rate requested for each telemetry message |
| `telemetry_rate_hz` | `50` | `/master/telemetry` publish rate |

`master.py`'s `-p/--port` and `-m/--mode` flags are also accepted.

## Building

The node uses [rclgo](https://github.com/tiiuae/rclgo), so it needs ROS 2 Jazzy, Go 1.23 or later,
and a built `custom_msgs`. Go bindings for the messages are generated at build
time into `msgs/`, which is not committed.

As a colcon package, which is the easiest way: clone this repository into the mira workspace (for example
`src/mira2_control_master_go`), then

```bash
make b P=mira2_control_master_go
ros2 launch mira2_control_master_go master.launch pixhawk_address:=/dev/Pixhawk
# SITL
ros2 run mira2_control_master_go master --ros-args -p pixhawk_address:=tcp:127.0.0.1:5760
```

Standalone:

```bash
source /opt/ros/jazzy/setup.bash
source <mira>/install/setup.bash   # provides custom_msgs
./scripts/build.sh                 # produces ./master
./master --ros-args -p pixhawk_address:=/dev/Pixhawk
```

## Why commands did not reach the Pixhawk in the previous Go version

Telemetry worked because ArduSub streams it to anyone listening. Commands
failed for these reasons, in rough order of impact:

1. **Wrong MAVLink system id.** The node sent as system `11`. ArduPilot
   discards `RC_CHANNELS_OVERRIDE` (and `MANUAL_CONTROL`) unless the sender's
   system id equals `SYSID_MYGCS`, which defaults to 255. pymavlink sends as 255
   by default, which is why the Python node worked. All thruster commands were
   silently ignored.
2. **The write queue was always full.** `updateRCChannels` called `Actuate()` in a
   loop with no delay, sending 8 messages per iteration. gomavlib drops messages
   when its 64-entry write queue is full (`WriteMessageAll` still returns `nil`),
   so ARM, DISARM and SET_MODE were usually dropped before reaching the serial port.
3. **`/master/commands` was ignored.** `autonomous` was always `false`, so only
   `/rov/commands` was handled. Everything in the ROS 2 workspace, including
   the joystick node, publishes to `/master/commands`.
4. **It was a ROS 1 node.** goroslib only speaks ROS 1, so it cannot talk to the
   ROS 2 Jazzy nodes in the workspace at all. Its hand-written message types
   (`my_package/Commands`, an outdated `Telemetry`) also did not match `custom_msgs`.
5. **Message interval requests were wrong.** It sent a `MESSAGE_INTERVAL`
   message, which is the autopilot's reply, instead of `COMMAND_LONG`
   `MAV_CMD_SET_MESSAGE_INTERVAL`. Seven goroutines then waited forever on the
   shared, unbuffered ack channel and took the ARM acknowledgement. That made
   every arm attempt block the ROS callback for about 10 s with retries.
6. Smaller issues: RC overrides were sent to target 0/0, the target id was
   taken from whichever component sent the last frame, the default PWM was 0
   (release to RC) instead of 1500, arming used the force-arm magic number
   (21196) that skips pre-arm checks, and every received frame started a
   goroutine that could block forever on the unbuffered heartbeat channel.

## Differences from alt_master.py

These are deliberate changes; everything else matches the Python node.

- An unknown mode string logs an error instead of calling `exit(1)`.
- When the Pixhawk rejects ARM or DISARM (for example, a pre-arm check fails),
  the stored arm state is rolled back. The next command message retries, at
  most every 2 s. The Python node assumed every command succeeded. The
  Pixhawk's STATUSTEXT messages are logged, so the reason for a rejection
  is shown.
- Telemetry is published at a fixed rate once the required messages have been
  received. A missing depth sensor gives `external_pressure = -1` instead of blocking.
- RC overrides are sent as one message with all 8 channels instead of 8
  single-channel messages. The effect on the autopilot is the same.
- The emergency lock still uses `/toggle_emergency`, so `killswitch` works unchanged.
