#!/usr/bin/env python3
"""cmd_vel -> motors, for a Waveshare AlphaBot2-Pi.

The AlphaBot2 has no ROS driver. This node is the smallest one that lets a ROS 2 controller -- GAMA's
ros_moving skill with the 'alphabot2' profile, a teleop, Nav2 -- drive it: it subscribes to cmd_vel
(geometry_msgs/msg/Twist), splits it into a left and a right wheel speed, and sets the PWM of the
TB6612FNG driver accordingly. Open loop: the robot has no encoder, so nothing checks that the wheels
actually turn at that speed, which is why the two speed parameters below must be calibrated (see
README.md).

Run it in the namespace GAMA expects, here /alphabot2:

    python3 alphabot2_bridge.py --ros-args -r __ns:=/alphabot2

It stops the motors when no command arrived for 'watchdog' seconds, and when it exits.

Pins are the BCM numbers of Waveshare's own AlphaBot2.py demo. If a wheel turns backwards, set
invert_left or invert_right rather than rewiring.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import RPi.GPIO as GPIO  # on a Pi 5, or Ubuntu 24.04: `pip install rpi-lgpio`, a drop-in replacement


class AlphaBot2Bridge(Node):

    def __init__(self):
        super().__init__('alphabot2_bridge')
        param = lambda name, default: self.declare_parameter(name, default).value
        # --- geometry and speed: measure and calibrate these on YOUR robot -------------------------------
        self.wheel_separation = param('wheel_separation', 0.10)   # m, centre to centre of the two wheels
        self.max_wheel_speed = param('max_wheel_speed', 1.0)      # m/s of a wheel at 100 % duty cycle
        # A DC gear motor does not start below some duty cycle: friction and the caster eat the torque, and
        # turning in place needs the most. Any non-zero speed starts at min_duty; find it with
        # alphabot2_duty_test.py (see README.md)
        self.min_duty = param('min_duty', 30.0)                   # %, the smallest duty that turns the robot
        # --- wiring, from Waveshare's AlphaBot2.py: A drives the left wheel, B the right one -------------
        self.ain1, self.ain2, self.ena = param('ain1', 12), param('ain2', 13), param('ena', 6)
        self.bin1, self.bin2, self.enb = param('bin1', 20), param('bin2', 21), param('enb', 26)
        self.invert_left = param('invert_left', False)
        self.invert_right = param('invert_right', False)
        pwm_frequency = param('pwm_frequency', 500)
        # --- safety --------------------------------------------------------------------------------------
        self.watchdog = param('watchdog', 0.5)                    # s without command before stopping

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in (self.ain1, self.ain2, self.ena, self.bin1, self.bin2, self.enb):
            try:
                # initial=LOW: motors off from the start, and rpi-lgpio does not need to read the pin first
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
            except Exception as e:
                raise SystemExit(
                    f'cannot drive GPIO{pin} ({e}). Another process holds it: is another bridge, or '
                    f'alphabot2_duty_test.py, running here or in another container? `gpioinfo` on the Pi names '
                    f'who uses each line. In a container, also check that the right /dev/gpiochip* is passed '
                    f'(RPI_LGPIO_CHIP picks it).')
        self.pwm_left = GPIO.PWM(self.ena, pwm_frequency)
        self.pwm_right = GPIO.PWM(self.enb, pwm_frequency)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

        # Reliable, like the 'cmd_qos' of the profiles: a dropped command is a command the robot never hears
        self.create_subscription(Twist, 'cmd_vel', self.on_command, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.last_command = None
        self.create_timer(0.1, self.check_watchdog)
        self.get_logger().info(
            f'listening on {self.get_namespace().rstrip("/")}/cmd_vel; wheel_separation={self.wheel_separation} m, '
            f'max_wheel_speed={self.max_wheel_speed} m/s, min_duty={self.min_duty} %')

    def on_command(self, msg: Twist):
        # Differential drive: ROS turns counter-clockwise for a positive angular.z, so the right wheel goes faster
        v, w = msg.linear.x, msg.angular.z
        left = v - w * self.wheel_separation / 2.0
        right = v + w * self.wheel_separation / 2.0
        self.set_wheel(left, self.ain1, self.ain2, self.pwm_left, self.invert_left)
        self.set_wheel(right, self.bin1, self.bin2, self.pwm_right, self.invert_right)
        self.last_command = time.monotonic()

    def set_wheel(self, speed, in1, in2, pwm, invert):
        forward = (speed >= 0) != invert
        GPIO.output(in1, GPIO.LOW if forward else GPIO.HIGH)
        GPIO.output(in2, GPIO.HIGH if forward else GPIO.LOW)
        pwm.ChangeDutyCycle(self.duty(speed))

    def duty(self, speed):
        # 0 stays 0; anything else is spread linearly between min_duty and 100 %, so that the speed the wheel
        # actually reaches stays proportional to the one asked for
        if abs(speed) < 1e-4:
            return 0.0
        return min(100.0, self.min_duty + (100.0 - self.min_duty) * abs(speed) / self.max_wheel_speed)

    def stop(self):
        self.pwm_left.ChangeDutyCycle(0)
        self.pwm_right.ChangeDutyCycle(0)

    def check_watchdog(self):
        if self.last_command is not None and time.monotonic() - self.last_command > self.watchdog:
            self.stop()
            self.last_command = None
            self.get_logger().info('no command for %.1f s: stopped' % self.watchdog)


def main():
    rclpy.init()
    node = AlphaBot2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        # Drop the PWM objects BEFORE cleanup: rpi-lgpio stops them again when they are garbage collected,
        # which fails noisily once cleanup has closed the chip
        node.pwm_left = node.pwm_right = None
        GPIO.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
