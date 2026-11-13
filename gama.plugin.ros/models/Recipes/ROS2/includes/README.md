# Running the ROS 2 models

These files support `05 GAMA in RViz.gaml` and `06 Gazebo in the loop.gaml`. The other models in this
project need nothing installed. Model 05 needs only ROS 2 and RViz; model 06 additionally needs Gazebo
Sim and `ros_gz_bridge`.

| File | Used by | What it is |
|---|---|---|
| `gama_markers.rviz` | model 05 | an RViz2 layout with a MarkerArray display on `/gama/markers` |
| `gama_loop.sdf` | model 06 | one differential-drive robot with real physics, walls and a pillar |
| `gama_loop_bridge.yaml` | model 06 | `cmd_vel` out, `odometry` back |

## Prerequisites

Model 05 needs RViz, which comes with any desktop ROS 2 install:

```bash
sudo apt install ros-${ROS_DISTRO}-rviz2
```

Model 06 additionally needs Gazebo Sim (Harmonic or later) and the bridge:

```bash
sudo apt install ros-${ROS_DISTRO}-ros-gz-bridge
```

If you are on Fortress, rename the plugins in `gama_loop.sdf`: `gz-sim-*-system` becomes
`ignition-gazebo-*-system` and `gz::sim::systems::` becomes `ignition::gazebo::systems::`.

## Model 05 — GAMA simulates, RViz shows

One terminal:

```bash
rviz2 -d gama_markers.rviz
```

Then run the `rviz` experiment in GAMA. No bridge and no world file: RViz is a ROS node, so it reads
GAMA's topics directly.

With a bare `rviz2`, three steps by hand: set **Fixed Frame** to `map`, then *Add → By topic →
/gama/markers → MarkerArray*.

GAMA publishes two topics here:

| Topic | Type | Why |
|---|---|---|
| `/gama/markers` | `visualization_msgs/msg/MarkerArray` | one arrow per agent, plus the arena outline and a label |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | one static transform `map` → `gama_world`, sent once |

**The transform is not optional.** RViz resolves every marker against its fixed frame, so that frame has
to exist in TF; with no TF at all, `map` is an unknown frame and nothing is drawn, with no error beyond
a warning in the Global Status panel. GAMA publishes it itself with `ros_world_tf("gama_world")`, once,
on a `"latched"` topic, so an RViz started after GAMA still receives it. If you would rather supply it
from ROS:

```bash
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id gama_world
```

Markers here are given a lifetime of half a second, so stopping GAMA makes them fade from the display
instead of freezing on screen. That is the opposite of the Gazebo behaviour described below.

## Model 06 — the full loop

```bash
gz sim -r gama_loop.sdf
```

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=gama_loop_bridge.yaml
```

Then run the `closed_loop` experiment. Its `init` waits up to five seconds for odometry and prints what
it found, so a setup problem is reported rather than looking like a robot that will not move.

**`gz sim -r` matters**: without `-r` the world starts paused, no odometry is published, and the model
reports that nothing publishes on `/model/gama_bot/odometry`.

## The robot keeps moving after GAMA stops (model 06)

Expected, and worth understanding once. `cmd_vel` carries a **setpoint, not an impulse**: `DiffDrive`
re-applies the last twist it received at every step of the physics, forever. Stopping GAMA only stops
the messages, and "no more messages" is indistinguishable from "the same command still holds".

Real robots close this hole with a watchdog that zeroes the motors after a fraction of a second without
a command. gz-sim has none, so the stop has to be sent deliberately. The experiment carries a **Stop**
user command for that, usable while it is still open; once the experiment is closed its ROS node is gone
with it and the stop has to come from a terminal:

```bash
ros2 topic pub --once /model/gama_bot/cmd_vel geometry_msgs/msg/Twist "{}"
```

`Ctrl+R` in Gazebo resets the world, which also works.

Model 05 does not have this problem at all: a marker carries a pose rather than a velocity, and it
expires. Nothing in RViz outlives the simulation that described it.

## GAMA's frame is not ROS's frame

GAMA's y axis points **down** the screen and its headings turn **clockwise**; ROS is right-handed (REP 103),
y to the left of x, yaws counter-clockwise, in radians. The two differ by a **reflection**, which no TF can
express, since a transform is a rotation plus a translation. So the plugin does the conversion, and the
models never write it by hand:

| GAMA → ROS | ROS → GAMA | What |
|---|---|---|
| `ros_point(location)` | `ros_location(msg)` | positions, placed in the frame of the world |
| `ros_orientation(heading)` | `ros_heading(msg)` | heading in degrees ↔ quaternion |
| `ros_pose(location, heading)` | both of the above | `geometry_msgs/Pose` |
| `ros_twist(speed, turn_rate)` | `ros_speed(msg)`, `ros_turn_rate(msg)` | m/s and degrees/s ↔ `Twist` |

The readers search the message, so `ros_location` takes a `Point`, a `Pose`, a `PoseStamped`, an
`Odometry` (`pose.pose.position`) or a `TransformStamped` alike.

Where the GAMA world sits is said once, with `ros_world_frame(frame_id, centre)`: the ROS frame, and the
point of that frame the **centre** of the GAMA world goes to. The default is the centre of `map` at the
origin, which is where a Gazebo world, the RViz grid and the odometry of a freshly booted robot all put
theirs, so most models never call it.

## Timestamps: left to the plugin

Anything with a `header` — TF, odometry, markers, scans — carries a stamp that ROS reads as **seconds and
nanoseconds since the UNIX epoch**. tf2 in particular assumes stamps only ever move forward.

**A header left out of a published message is filled in**: the wall-clock time, and the frame of the
world. A header given without `stamp` or `frameId` gets the missing part. A stamp written explicitly is
left alone, so `"stamp"::["sec"::0]` still means zero.

What not to write instead is GAMA's `time`. It is simulation time and **restarts at 0 on every run**.
Publishing it as a stamp means the second run of a model dates its transforms before the ones the first
run left in RViz's buffer, and time going backwards is the one thing a TF buffer cannot survive. What you
see is the second run displaying correctly for a moment, then a stream of `no transform to fixed frame`,
then an alternation between working and frozen as the buffer is discarded and refilled. A long
`lifetime` on markers papers over it, which makes it easy to misread as a lifetime problem. Likewise
`"nanosec"::0` quantises every stamp to a whole second, so twenty messages published at 20 Hz all carry
the same date.

## Checking each link separately

The chain is GAMA → DDS → bridge → Gazebo, and it is worth testing one hop at a time when nothing
moves.

```bash
# Is GAMA's traffic visible to ROS at all? (model 05)
ros2 topic echo /gama/markers --once
ros2 topic echo /tf_static --once --qos-durability transient_local   # map -> gama_world

# Does Gazebo publish and listen where we think? (model 06)
gz topic -l | grep gama

# Is the bridge exposing the ROS side?
ros2 topic list | grep gama

# Does Gazebo obey a command sent by hand?
ros2 topic pub -r 10 /model/gama_bot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"

# Is odometry coming back?
ros2 topic echo /model/gama_bot/odometry --once
```

If `ros2 topic echo /chatter` shows GAMA's messages, the DDS link is fine and the problem is in the
bridge or the SDF. If it shows nothing, the problem is between GAMA and ROS — see below.

The single most useful command when a model reports that nobody is listening:

```bash
ros2 topic info -v /gama/markers
```

It prints `Publisher count`, `Subscription count` and the full QoS of both ends. `Publisher count: 1`
with `Subscription count: 0` means GAMA is fine and nothing is listening — for model 06, most often a
bridge running happily with the *other* config file, so check that it printed the `Creating ROS->GZ
Bridge` lines you expect. GAMA appears as `_CREATED_BY_BARE_DDS_APP_` with
`Topic type hash: INVALID`, which is normal: jros2 is a bare DDS application, not an rclcpp node, and
it does not prevent matching.

## When GAMA and ROS cannot see each other

**`ROS_DOMAIN_ID`** must match. GAMA reads it the way ROS does; `ros_node("name", 42)` overrides it
explicitly, which is the reliable way to be sure.

**RMW implementation.** GAMA talks DDS directly through jros2, which embeds Fast DDS. ROS 2 Jazzy and
Kilted default to `rmw_fastrtps_cpp`, which interoperates. If your setup uses CycloneDDS, the safest fix
is to run the bridge under Fast DDS:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:=gama_loop_bridge.yaml
```

**GAMA on Windows, ROS in WSL2.** This is the case that fails silently. By default WSL2 sits behind NAT,
so DDS discovery between the Windows host and the Linux guest does not work in either direction. On
Windows 11 22H2 and later, put this in `%UserProfile%\.wslconfig` and run `wsl --shutdown`:

```ini
[wsl2]
networkingMode=mirrored
```

Then the two sides share the host's network stack and discovery works. Allowing `java.exe` through the
Windows firewall on private networks may also be needed. Running GAMA inside WSL2 alongside ROS avoids
the question entirely.

## Adding agents to model 05

Just raise the "Number of agents" parameter. Nothing outside the model declares them: one MarkerArray
carries the whole population, however large. This is the main practical difference with driving entities
in a physics engine, where each one had to exist in the world file first.
