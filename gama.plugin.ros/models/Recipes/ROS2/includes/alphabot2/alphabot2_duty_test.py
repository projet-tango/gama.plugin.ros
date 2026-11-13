#!/usr/bin/env python3
"""Find the smallest PWM duty cycle that moves a Waveshare AlphaBot2: the bridge's min_duty parameter.

No ROS involved: this drives the motors directly, raising the duty cycle by a step every few seconds and
printing it. Watch the robot and note the first value at which it moves steadily (not just twitches).

    python3 alphabot2_duty_test.py spin       # turning in place: the hardest case, use this one for min_duty
    python3 alphabot2_duty_test.py straight   # going forward, for comparison

Put the robot on the floor it will drive on (carpet and tiles differ), with charged batteries. Ctrl+C stops
the motors at any time. Pins are the same defaults as alphabot2_bridge.py; edit them below if you changed
those, and set INVERT_LEFT / INVERT_RIGHT the same way as the bridge's invert_left / invert_right.
"""

import argparse
import time

import RPi.GPIO as GPIO  # on a Pi 5, or Ubuntu 24.04: `pip install rpi-lgpio`, a drop-in replacement

AIN1, AIN2, ENA = 12, 13, 6     # left wheel
BIN1, BIN2, ENB = 20, 21, 26    # right wheel
INVERT_LEFT, INVERT_RIGHT = False, False


def claim_output(pin):
    try:
        # initial=LOW: motors off from the start, and rpi-lgpio does not need to read the pin first
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    except Exception as e:
        raise SystemExit(
            f'cannot drive GPIO{pin} ({e}). Another process holds it: is alphabot2_bridge.py still running, '
            f'here or in another container? `gpioinfo` on the Pi names who uses each line. In a container, '
            f'also check that the right /dev/gpiochip* is passed (RPI_LGPIO_CHIP picks it).')


def direction(in1, in2, forward, invert):
    forward = forward != invert
    GPIO.output(in1, GPIO.LOW if forward else GPIO.HIGH)
    GPIO.output(in2, GPIO.HIGH if forward else GPIO.LOW)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('mode', choices=['spin', 'straight'])
    parser.add_argument('--start', type=float, default=10.0, help='first duty cycle, %% (default 10)')
    parser.add_argument('--step', type=float, default=5.0, help='increment, %% (default 5)')
    parser.add_argument('--hold', type=float, default=2.0, help='seconds at each value (default 2)')
    parser.add_argument('--max', type=float, default=80.0, help='last duty cycle, %% (default 80)')
    args = parser.parse_args()

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pin in (AIN1, AIN2, ENA, BIN1, BIN2, ENB):
        claim_output(pin)
    left, right = GPIO.PWM(ENA, 500), GPIO.PWM(ENB, 500)
    left.start(0)
    right.start(0)
    try:
        # spin: counter-clockwise, left wheel backwards and right wheel forwards, as the bridge does it
        direction(AIN1, AIN2, args.mode == 'straight', INVERT_LEFT)
        direction(BIN1, BIN2, True, INVERT_RIGHT)
        duty = args.start
        while duty <= args.max:
            print(f'duty {duty:5.1f} %', flush=True)
            left.ChangeDutyCycle(duty)
            right.ChangeDutyCycle(duty)
            time.sleep(args.hold)
            duty += args.step
        print('reached --max without being stopped')
    except KeyboardInterrupt:
        print()
    finally:
        left.ChangeDutyCycle(0)
        right.ChangeDutyCycle(0)
        # Drop the PWM objects BEFORE cleanup: rpi-lgpio stops them again when they are garbage collected,
        # which fails noisily once cleanup has closed the chip
        del left, right
        GPIO.cleanup()
        print('motors stopped')


if __name__ == '__main__':
    main()
