#!/usr/bin/env python3
# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)

import sys
import threading

import geometry_msgs.msg
import rcl_interfaces.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

# Ensure virtualenv libraries are loaded first if needed (matching obstacle_detector.py)
venv_path = "/home/charuka/Documents/uni pro/venv/lib/python3.12/site-packages"
if venv_path not in sys.path:
    sys.path.insert(0, venv_path)

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It is configured for WASD control layout.
---------------------------
Moving around:
        w
   a    s    d

q : forward-left
e : forward-right
z : backward-left
c : backward-right

anything else : stop

Speed adjustments:
u/j : increase/decrease max speed (both linear and angular) by 10%
i/k : increase/decrease only linear speed by 10%
o/l : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'w': (1.0, 0.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0, 0.0),
    'a': (0.0, 0.0, 0.0, 1.0),
    'd': (0.0, 0.0, 0.0, -1.0),
    'q': (1.0, 0.0, 0.0, 1.0),
    'e': (1.0, 0.0, 0.0, -1.0),
    'z': (-1.0, 0.0, 0.0, -1.0),
    'c': (-1.0, 0.0, 0.0, 1.0),
}

speedBindings = {
    'u': (1.1, 1.1),
    'j': (0.9, 0.9),
    'i': (1.1, 1.0),
    'k': (0.9, 1.0),
    'o': (1.0, 1.1),
    'l': (1.0, 0.9),
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %.2f\tturn %.2f ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard_wasd')

    # parameters
    read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    stamped = node.declare_parameter('stamped', False, read_only_descriptor).value
    frame_id = node.declare_parameter('frame_id', '', read_only_descriptor).value
    speed = node.declare_parameter('speed', 0.5, read_only_descriptor).value
    turn = node.declare_parameter('turn', 1.0, read_only_descriptor).value

    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist_msg)

    except Exception as e:
        print(e)

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
