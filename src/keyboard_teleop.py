#!/usr/bin/env python

# Author: generated
# Description: Keyboard teleoperation for gripper movement in the XY plane and Z axis.

try:
    import rospy
    import tf
    ros_enabled = True
except ImportError:
    print("Couldn't import ROS. I assume you're running this on your laptop")
    ros_enabled = False

import os
import sys
import termios
import tty
import argparse
import numpy as np
import select

current_dir = os.path.dirname(os.path.abspath(__file__))
helper_path = os.path.join(current_dir, 'helperFunction')
if helper_path not in sys.path:
    sys.path.append(helper_path)

from rtde_helper import rtdeHelp
import config


def getch(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, w, e = select.select([sys.stdin], [], [], timeout)
        if r:
            ch = sys.stdin.read(1)
        else:
            ch = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def read_key():
    char = getch()
    if char is None:
        return None

    if char == '\x1b':
        next1 = getch()
        if next1 == '[':
            next2 = getch()
            return '\x1b[' + next2
        return char + next1
    return char


def print_help():
    print('Keyboard teleop commands:')
    print('  Arrow Up    : move +X (forward)')
    print('  Arrow Down  : move -X (backward)')
    print('  Arrow Left  : move +Y (left)')
    print('  Arrow Right : move -Y (right)')
    print('  q           : move +Z (up)')
    print('  a           : move -Z (down)')
    print('  h           : show this help message')
    print('  Ctrl+C      : exit')
    print('')


def parse_args():
    parser = argparse.ArgumentParser(description='Keyboard teleoperation for UR10 gripper pose control')
    parser.add_argument('--step', type=float, default=0.01, help='motion step size in meters')
    parser.add_argument('--speed', type=float, default=0.05, help='motion speed for goToPose')
    parser.add_argument('--acc', type=float, default=0.05, help='motion acceleration for goToPose')
    return parser.parse_args()


def main():
    if not ros_enabled:
        print('ROS is required to run this teleop script.')
        return

    args = parse_args()

    rospy.init_node('keyboard_teleop', anonymous=True)
    rtde_help = rtdeHelp(125)
    rospy.sleep(0.5)
    try:
        rtde_help.setTCPoffset(config.GRIPPER_OFFSET)
    except Exception as e:
        print('Warning: could not set TCP offset:', e)
    rospy.sleep(0.2)

    print_help()

    current_pose = rtde_help.getCurrentPoseTF()
    orientation = [
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w,
    ]

    print('Starting pose: x=%.4f y=%.4f z=%.4f' % (
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z,
    ))

    try:
        while not rospy.is_shutdown():
            key = read_key()
            if key is None:
                continue
            print(' %s' % repr(key))


            if key == '\x03':  # Hex code for Ctrl+C
                print('\nCtrl+C detected, exiting teleop.')
                rospy.signal_shutdown("User requested exit.")
                break
            elif key == 'h':
                print_help()
                continue

            pose = rtde_help.getCurrentPoseTF()
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z

            if key == '\x1b[A':
                x += args.step
            elif key == '\x1b[B':
                x -= args.step
            elif key == '\x1b[D':
                y += args.step
            elif key == '\x1b[C':
                y -= args.step
            elif key == 'q':
                z += args.step
            elif key == 'a':
                z -= args.step
            else:
                print('Unknown key. Press h for help.')
                continue

            target_pose = rtde_help.getPoseObj([x, y, z], orientation)
            rtde_help.goToPose(target_pose, speed=args.speed, acc=args.acc)
            print('Moved to x=%.4f y=%.4f z=%.4f' % (x, y, z))
    except KeyboardInterrupt:
        print('\nKeyboard interrupt received, exiting teleop.')
    except rospy.ROSInterruptException:
        print('\nROS interrupt received, exiting teleop.')


if __name__ == '__main__':
    main()
