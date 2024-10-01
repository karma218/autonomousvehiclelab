# # Copyright 2011 Brown University Robotics.
# # Copyright 2017 Open Source Robotics Foundation, Inc.
# # All rights reserved.
# #
# # Software License Agreement (BSD License 2.0)
# #
# # Redistribution and use in source and binary forms, with or without
# # modification, are permitted provided that the following conditions
# # are met:
# #
# #  * Redistributions of source code must retain the above copyright
# #    notice, this list of conditions and the following disclaimer.
# #  * Redistributions in binary form must reproduce the above
# #    copyright notice, this list of conditions and the following
# #    disclaimer in the documentation and/or other materials provided
# #    with the distribution.
# #  * Neither the name of the Willow Garage nor the names of its
# #    contributors may be used to endorse or promote products derived
# #    from this software without specific prior written permission.
# #
# # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# # POSSIBILITY OF SUCH DAMAGE.

# import sys

# import geometry_msgs.msg
# import rclpy

# if sys.platform == 'win32':
#     import msvcrt
# else:
#     import termios
#     import tty


# msg = """
# This node takes keypresses from the keyboard and publishes them
# as Twist messages. It works best with a US keyboard layout.
# ---------------------------
# Moving around:
#    u    i    o
#    j    k    l
#    m    ,    .

# For Holonomic mode (strafing), hold down the shift key:
# ---------------------------
#    U    I    O
#    J    K    L
#    M    <    >

# t : up (+z)
# b : down (-z)

# anything else : stop

# q/z : increase/decrease max speeds by 10%
# w/x : increase/decrease only linear speed by 10%
# e/c : increase/decrease only angular speed by 10%

# CTRL-C to quit
# """

# moveBindings = {
#     'i': (1, 0, 0, 0),
#     'o': (1, 0, 0, -1),
#     'j': (0, 0, 0, 1),
#     'l': (0, 0, 0, -1),
#     'u': (1, 0, 0, 1),
#     ',': (-1, 0, 0, 0),
#     '.': (-1, 0, 0, 1),
#     'm': (-1, 0, 0, -1),
#     'O': (1, -1, 0, 0),
#     'I': (1, 0, 0, 0),
#     'J': (0, 1, 0, 0),
#     'L': (0, -1, 0, 0),
#     'U': (1, 1, 0, 0),
#     '<': (-1, 0, 0, 0),
#     '>': (-1, -1, 0, 0),
#     'M': (-1, 1, 0, 0),
#     't': (0, 0, 1, 0),
#     'b': (0, 0, -1, 0),
# }

# speedBindings = {
#     'q': (1.1, 1.1),
#     'z': (.9, .9),
#     'w': (1.1, 1),
#     'x': (.9, 1),
#     'e': (1, 1.1),
#     'c': (1, .9),
# }


# def getKey(settings):
#     if sys.platform == 'win32':
#         # getwch() returns a string on Windows
#         key = msvcrt.getwch()
#     else:
#         tty.setraw(sys.stdin.fileno())
#         # sys.stdin.read() returns a string on Linux
#         key = sys.stdin.read(1)
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     return key


# def saveTerminalSettings():
#     if sys.platform == 'win32':
#         return None
#     return termios.tcgetattr(sys.stdin)


# def restoreTerminalSettings(old_settings):
#     if sys.platform == 'win32':
#         return
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# def vels(speed, turn):
#     return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


# def main():
#     settings = saveTerminalSettings()

#     rclpy.init()

#     node = rclpy.create_node('teleop_twist_keyboard')
#     pub = node.create_publisher(geometry_msgs.msg.Twist, '/keyboard/cmd_vel', 10)

#     speed = 0.3
#     turn = 1.0
#     x = 0.0
#     y = 0.0
#     z = 0.0
#     th = 0.0
#     status = 0.0

#     try:
#         print(msg)
#         print(vels(speed, turn))
#         while True:
#             key = getKey(settings)
#             if key in moveBindings.keys():
#                 x = moveBindings[key][0]
#                 y = moveBindings[key][1]
#                 z = moveBindings[key][2]
#                 th = moveBindings[key][3]
#             elif key in speedBindings.keys():
#                 speed = speed * speedBindings[key][0]
#                 turn = turn * speedBindings[key][1]

#                 print(vels(speed, turn))
#                 if (status == 14):
#                     print(msg)
#                 status = (status + 1) % 15
#             else:
#                 x = 0.0
#                 y = 0.0
#                 z = 0.0
#                 th = 0.0
#                 if (key == '\x03'):
#                     break

#             twist = geometry_msgs.msg.Twist()
#             twist.linear.x = x * speed
#             twist.linear.y = y * speed
#             twist.linear.z = z * speed
#             twist.angular.x = 0.0
#             twist.angular.y = 0.0
#             twist.angular.z = th * turn
#             print(f'key pressed: {key},', vels(speed, turn))
#             pub.publish(twist)

#     except Exception as e:
#         print(e)

#     finally:
#         twist = geometry_msgs.msg.Twist()
#         twist.linear.x = 0.0
#         twist.linear.y = 0.0
#         twist.linear.z = 0.0
#         twist.angular.x = 0.0
#         twist.angular.y = 0.0
#         twist.angular.z = 0.0
#         pub.publish(twist)

#         restoreTerminalSettings(settings)


# if __name__ == '__main__':
#     main()

import sys
import geometry_msgs.msg
import rclpy
from pynput import keyboard

# Messages for instructions
msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   w
a  s  d

Up arrow: increase speed
Down arrow: decrease speed
CTRL-C to quit
"""

move = {
    'w': (1, 0, 0, 0),  # Move forward
    's': (-1, 0, 0, 0),  # Move backward
}
turn = {
    'a': (0, 0, 0, 1),  # Turn left
    'd': (0, 0, 0, -1),  # Turn right
}
# Use arrow keys for speed adjustment
speedBindings = {
    'up': (1.1, 1),  # Up arrow for increasing speed
    'down': (0.9, 1),  # Down arrow for decreasing speed
}

# Set to track current keys pressed
pressed_keys = set()


def on_press(key):
    """Callback function for key press events."""
    try:
        # Handle regular keys (e.g., 'a', 'w', etc.)
        if hasattr(key, 'char'):
            pressed_keys.add(key.char)
        # Handle special keys (e.g., arrow keys)
        else:
            if key == keyboard.Key.up:
                pressed_keys.add('up')
            elif key == keyboard.Key.down:
                pressed_keys.add('down')

    except AttributeError:
        pass


def on_release(key):
    """Callback function for key release events."""
    try:
        # Handle regular keys
        if hasattr(key, 'char'):
            pressed_keys.discard(key.char)
        # Handle special keys
        else:
            if key == keyboard.Key.up:
                pressed_keys.discard('up')
            elif key == keyboard.Key.down:
                pressed_keys.discard('down')

        # Stop listener if 'esc' is pressed
        if key == keyboard.Key.esc:
            return False
    except KeyError:
        pass


def main():
    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(geometry_msgs.msg.Twist, '/keyboard/cmd_vel', 10)

    speed = 0.3
    turn_speed = 1.0  # renamed to avoid confusion with turn dictionary
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    
    try:
        print(msg)
        
        # Start the listener for keyboard events in a background thread
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        while rclpy.ok():
            x = 0.0  # Reset movement each loop
            y = 0.0
            z = 0.0
            th = 0.0

            # Check if any movement keys are pressed
            if pressed_keys:
                for key in pressed_keys.copy():
                    if key in move:
                        move_x, move_y, move_z, move_th = move[key]
                        x += move_x  # Add movement to x
                        y += move_y
                        z += move_z
                        th += move_th  # Not expected for movement keys but keeps structure
                    elif key in turn:
                        turn_x, turn_y, turn_z, turn_th = turn[key]
                        x += turn_x  # Likely 0
                        y += turn_y  # Likely 0
                        z += turn_z  # Likely 0
                        th += turn_th  # Add turning to th
                    elif key in speedBindings:
                        speed = speed * speedBindings[key][0]
                        if status == 14:
                            print(msg)
                        status = (status + 1) % 15
            else:
                # Stop movement if no keys are pressed
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0

            # Create the Twist message to publish
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = float(x) * speed
            twist.linear.y = float(y) * speed
            twist.linear.z = float(z) * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(th) * turn_speed  # Ensure it's a float
            pub.publish(twist)
            print(f'Direction: {twist.linear.x},{twist.linear.y},{twist.linear.z}\nTurn: {twist.angular.z}')
            
            # Reduce CPU load with a short sleep
            rclpy.spin_once(node, timeout_sec=0.1)

    except Exception as e:
        print(e)

    finally:
        # Stop the robot on exit
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        # Stop the listener when the program exits
        listener.stop()


if __name__ == '__main__':
    main()