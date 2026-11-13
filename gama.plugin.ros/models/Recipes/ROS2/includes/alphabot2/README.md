# AlphaBot2 with GAMA

Supports `10 Skill - AlphaBot2.gaml` and the `alphabot2` profile of the `ros_moving` skill.

## What the robot has, and what it lacks

The [Waveshare AlphaBot2](https://www.waveshare.com/wiki/AlphaBot2) is a differential-drive base: two
N20 gear motors (1:30, 6 V / 600 RPM) on 42 mm wheels, driven by a TB6612FNG, plus a caster. It has IR
obstacle sensors, a line-tracking array and a camera, but **no wheel encoder and no IMU**, and Waveshare
ships Python demos, not a ROS driver.

So two things are needed that other robots provide on their own:

| Missing | Provided by |
|---|---|
| a node that obeys `cmd_vel` | `alphabot2_bridge.py`, in this folder, run on the Pi |
| a pose | the skill **estimates** it from the commands it sent (dead reckoning), unless a tracker publishes one |

An estimated pose drifts: every slip, uneven floor or tired battery adds up, and nothing corrects it.
`ros_estimated` is `true` in that case, and the model must say where the robot was put down by writing
`location` and `heading`. With a tracker — a camera looking down with ArUco tags, a motion-capture system
— set `pose_topic` and `pose_type` in the profile and the pose is measured instead.

## On the Pi

The bridge needs ROS 2 and the GPIO. With Ubuntu 24.04 and ROS 2 Jazzy on a 64-bit Pi (3B, 4, Zero 2 W):

```bash
sudo apt install ros-jazzy-ros-base python3-pip
pip install rpi-lgpio          # RPi.GPIO for recent kernels and the Pi 5; plain RPi.GPIO works on older systems
source /opt/ros/jazzy/setup.bash
python3 alphabot2_bridge.py --ros-args -r __ns:=/alphabot2
```

The original Pi Zero / Zero W is ARMv6 and cannot run ROS 2 binaries.

Same `ROS_DOMAIN_ID` on the Pi and on the machine running GAMA, same network, and multicast allowed
between them (DDS discovery uses it).

## Calibrating

Three parameters turn a velocity into PWM; all are guesses until measured. Calibrate them in this order,
with charged batteries and on the floor the robot will drive on; no GAMA is involved.

**`min_duty`**, the smallest duty cycle that moves the robot. Below it the motors hum but do not turn:
friction, the gearbox and the caster eat the torque, and turning in place needs the most. Without it, slow
commands do nothing at all: the half turn below asks each wheel for 0.08 m/s, which is 8 % of full power.
`alphabot2_duty_test.py` drives the motors directly, with no ROS, raising the duty cycle by 5 % every 2 s:

```bash
python3 alphabot2_duty_test.py spin
```

Note the first value at which the robot turns steadily, not just twitches, and add 5. Run it again with
`straight` out of curiosity: it should start lower. The bridge sends `min_duty` for the slowest non-zero
command and spreads the others linearly up to 100 %.

**`max_wheel_speed`**, the speed of a wheel at 100 % duty cycle. Ask for 0.2 m/s during 5 s, which should
cover 1 m:

```bash
ros2 topic pub -t 50 -r 10 /alphabot2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

Measure the distance travelled, `d` metres. The right value is the current one times `d`.

**`wheel_separation`**, centre to centre of the two wheels. Ask for half a turn, π/2 rad/s during 2 s:

```bash
ros2 topic pub -t 20 -r 10 /alphabot2/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.5708}}"
```

If it turned `a` degrees instead of 180, the right value is the current one times `180 / a`.

Each test runs the bridge with what was measured so far, e.g. `-p min_duty:=35.0` for the two above.
At the end, run it with all three:

```bash
python3 alphabot2_bridge.py --ros-args -r __ns:=/alphabot2 \
    -p min_duty:=35.0 -p max_wheel_speed:=0.85 -p wheel_separation:=0.092
```

If the robot still stalls or jerks at the slowest speeds, raise `min_duty` a little, then measure
`max_wheel_speed` again: it depends on it.

If a wheel turns backwards, add `-p invert_left:=true` or `-p invert_right:=true`.

## In GAMA

The built-in `alphabot2` profile sends `{ns}/cmd_vel`, reliable, and names no pose and no odometry
topic. Tune it per kind of robot rather than per agent, by deriving a profile, as model 10 does:

```gaml
string profile <- ros_define_profile("my_alphabot2", "alphabot2", ["max_speed"::0.2, "max_turn_rate"::90.0]);
```

`ros_profile("alphabot2")` returns every setting there is.
