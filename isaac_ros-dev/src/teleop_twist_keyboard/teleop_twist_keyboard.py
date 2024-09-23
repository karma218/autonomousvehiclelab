import sys
import select
import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   w
a  s  d

Up arrow: increase speed
Down arrow: decrease speed

t : up (+z)
b : down (-z)

anything else : stop

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0),     # Move forward
    'a': (0, 0, 0, 1),     # Turn left
    's': (-1, 0, 0, 0),    # Move backward
    'd': (0, 0, 0, -1),    # Turn right
    't': (0, 0, 1, 0),     # Move up
    'b': (0, 0, -1, 0),    # Move down
}

# Use arrow keys for speed adjustment
speedBindings = {
    '\x1b[A': (1.1, 1),  # Up arrow for increasing speed
    '\x1b[B': (0.9, 1),  # Down arrow for decreasing speed
}


def getKey(settings):
    if sys.platform == 'win32':
        # Check if a key is pressed on Windows
        if msvcrt.kbhit():
            return msvcrt.getwch()
        return ''
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # Non-blocking input
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':  # Check for escape sequences (arrow keys)
                key += sys.stdin.read(2)
        else:
            key = ''
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
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(geometry_msgs.msg.Twist, '/keyboard/cmd_vel', 10)

    speed = 0.3
    turn = 1.0
    status = 0.0
    active_keys = set()  # Track pressed keys

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            
            if key == '\x03':  # CTRL+C to quit
                break

            # Add key to active keys if valid for movement
            if key in moveBindings:
                active_keys.add(key)

            # Handle speed adjustment
            elif key in speedBindings:
                speed = speed * speedBindings[key][0]
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15

            # Handle key release for move keys
            if key == '':  # No key pressed, stop movement
                active_keys.clear()

            # Calculate movement by summing over active keys
            x, y, z, th = 0, 0, 0, 0
            for k in active_keys:
                dx, dy, dz, dth = moveBindings[k]
                x += dx
                y += dy
                z += dz
                th += dth

            # Publish the movement message
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.z = th * turn
            print(f'movement: {twist.linear.x}, {twist.linear.y}, {twist.linear.z}, {twist.angular.z}')
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
