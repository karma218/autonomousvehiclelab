import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, jsonify, Response, stream_with_context
from flask_socketio import SocketIO
from threading import Thread, Lock, Event


# Flask-SocketIO setup
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

        # Camera-related attributes
        self.bridge = CvBridge()
        self.latest_frame = None
        self.camera_lock = Lock()
        self.new_frame_event = Event()  # Event to signal new frames
        self.create_subscription(Image, '/video/front_camera', self.camera_callback, 10)

    def update_keys(self, key, action):
        """
        Safely update the pressed keys set.
        """
        with self.pressed_keys_lock:
            if action == 'press':
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
            if key == 'w':
                x += 1.0
            elif key == 's':
                x -= 1.0
            elif key == 'a':
                th += 1.0
            elif key == 'd':
                th -= 1.0
            elif key == 'up':
                self.speed *= 1.1
            elif key == 'down':
                self.speed *= 0.9

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = x * self.speed
        twist.angular.z = th * self.turn_speed
        self.publisher_.publish(twist)

    def camera_callback(self, msg):
        """
        Handle incoming camera frames from ROS.
        """
        with self.camera_lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.new_frame_event.set()  # Signal that a new frame is available
        # self.get_logger().info("Received new frame")


server2_node = None

# Flask Endpoints and WebSocket Handlers
@app.route('/')
def index():
    """
    Basic health check.
    """
    return jsonify({"status": "Server2 is running"})

@app.route('/stop', methods=['POST'])
def stop_robot():
    """
    Stop the robot by publishing a zero Twist.
    """
    twist = Twist()
    server2_node.publisher_.publish(twist)
    return jsonify({"status": "Robot stopped"})


@app.route('/camera/stream')
def camera_stream():
    """
    Serve the camera stream as an MJPEG feed.
    Continuously updates with new frames from the ROS topic.
    """
    def generate():
        while True:
            # Wait for a new frame (with a timeout to prevent indefinite blocking)
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

    # Use a generator to stream frames
    return Response(stream_with_context(generate()), content_type='multipart/x-mixed-replace; boundary=frame')


@socketio.on('keypress')
def handle_keypress(data):
    """
    Handle WebSocket keypress events.
    """
    key = data.get('key')
    action = data.get('action')  # 'press' or 'release'
    if key and action:
        server2_node.update_keys(key, action)


def run_ros2_node():
    """
    Start the ROS2 node in a separate thread.
    """
    global server2_node
    rclpy.init()
    server2_node = Server2Node()

    while rclpy.ok():
        server2_node.process_keys()
        rclpy.spin_once(server2_node, timeout_sec=0.1)


def main():
    """
    Entry point for the server.
    """
    print("Starting Server2...")

    # Run ROS2 node in a separate thread
    ros_thread = Thread(target=run_ros2_node)
    ros_thread.start()

    # Run Flask-SocketIO server
    socketio.run(app, host='0.0.0.0', port=5000)


if __name__ == '__main__':
    main()
