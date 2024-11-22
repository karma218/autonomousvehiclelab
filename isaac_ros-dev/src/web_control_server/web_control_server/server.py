import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, jsonify, Response, stream_with_context
from flask_socketio import SocketIO
from threading import Thread, Lock, Event

# Flask app setup
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# ROS2 Node Class
class Server2Node(Node):
    def __init__(self):
        super().__init__('server2')
        self.publisher_ = self.create_publisher(Twist, '/keyboard/cmd_vel', 10)
        self.pressed_keys = set()
        self.speed = 0.3
        self.turn_speed = 1.0
        self.pressed_keys_lock = Lock()
        self.max_speed = 1.5  # Maximum speed limit
        self.min_speed = 0.1  # Minimum speed limit

        # Movement and Turn Bindings
        self.move_bindings = {
            'w': (1, 0, 0, 0),  # Forward along x-axis
            's': (-1, 0, 0, 0),  # Backward along x-axis
            'a': (0, 0, 0, 1),  # Rotate left (yaw)
            'd': (0, 0, 0, -1)  # Rotate right (yaw)
        }
        self.speed_bindings = {
            'up': (1.1, 1),  # Increase speed
            'down': (0.9, 1)  # Decrease speed
        }

        # Camera-related attributes
        self.bridge = CvBridge()
        self.latest_frame = None
        self.camera_lock = Lock()
        self.new_frame_event = Event()
        self.create_subscription(Image, '/video/front_camera', self.camera_callback, 10)

    def update_keys(self, key, action):
        """
        Safely update the pressed keys set based on incoming WebSocket events.
        """
        with self.pressed_keys_lock:
            if action == 'press':
                if key not in self.pressed_keys:  # Only add if not already pressed
                    self.pressed_keys.add(key)
            elif action == 'release':
                self.pressed_keys.discard(key)

    def process_keys(self):
        """
        Process currently pressed keys and publish a Twist message.
        """
        x, y, z, th = 0.0, 0.0, 0.0, 0.0

        with self.pressed_keys_lock:
            keys_to_process = self.pressed_keys.copy()

        for key in keys_to_process:
            if key in self.move_bindings:
                move_x, move_y, move_z, move_th = self.move_bindings[key]
                x += move_x
                y += move_y
                z += move_z
                th += move_th
            elif key in self.speed_bindings:
                self.speed *= self.speed_bindings[key][0]
                # Limit the speed
                self.speed = max(min(self.speed, self.max_speed), self.min_speed)

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = x * self.speed
        twist.angular.z = th * self.turn_speed
        self.publisher_.publish(twist)

        # Log only when something is happening
        if x != 0.0 or th != 0.0:
            print(f"Published Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

    def camera_callback(self, msg):
        """
        Handle incoming camera frames from ROS.
        """
        with self.camera_lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.new_frame_event.set()  # Signal that a new frame is available


server2_node = None

# Flask Endpoints
@app.route('/')
def index():
    return jsonify({"status": "Server2 is running"})

@app.route('/stop', methods=['POST'])
def stop_robot():
    twist = Twist()
    server2_node.publisher_.publish(twist)
    return jsonify({"status": "Robot stopped"})

@app.route('/camera/stream')
def camera_stream():
    def generate():
        while True:
            if server2_node.new_frame_event.wait(timeout=1.0):
                server2_node.new_frame_event.clear()

                with server2_node.camera_lock:
                    if server2_node.latest_frame is not None:
                        _, jpeg = cv2.imencode('.jpg', server2_node.latest_frame)
                        frame = jpeg.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else:
                print("No new frame received within timeout")

    return Response(stream_with_context(generate()), content_type='multipart/x-mixed-replace; boundary=frame')


# SocketIO Handlers
@socketio.on('keypress')
def handle_keypress(data):
    """
    Handle keypress events from the frontend.
    """
    key = data.get('key')
    action = data.get('action')  # 'press' or 'release'
    if key and action:
        print(f"Received keypress: {key}, action: {action}")
        server2_node.update_keys(key, action)


def run_ros2_node():
    global server2_node
    rclpy.init()
    server2_node = Server2Node()

    try:
        while rclpy.ok():
            server2_node.process_keys()
            rclpy.spin_once(server2_node, timeout_sec=0.1)
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        twist.linear.x = twist.linear.y = twist.linear.z = 0.0
        twist.angular.x = twist.angular.y = twist.angular.z = 0.0
        server2_node.publisher_.publish(twist)
        server2_node.destroy_node()
        rclpy.shutdown()


def main():
    print("Starting Server2...")

    # Run Flask server in a separate thread
    flask_thread = Thread(target=socketio.run, kwargs={'app': app, 'host': '10.110.194.54', 'port': 5002})
    flask_thread.start()

    # Run ROS2 node in the main thread
    run_ros2_node()


if __name__ == '__main__':
    main()
