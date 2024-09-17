import asyncio
import websockets
from aiohttp import web
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import defaultdict

# Global dictionary to store image data for each topic
image_data = defaultdict(bytes)
image_data_lock = asyncio.Lock()

class ROS2ImageServer(Node):
    def __init__(self):
        super().__init__('ros2_image_server')
        self.bridge = CvBridge()
        self.setup_parameters()
        self.start_image_subscribers()

    def setup_parameters(self):
        # Load parameters if necessary
        pass

    def start_image_subscribers(self):
        # Get a list of all image topics and create subscriptions for them
        topic_names = self.get_topic_names_and_types()
        image_topics = [topic for topic, types in topic_names.items() if 'sensor_msgs/msg/Image' in types]

        for topic in image_topics:
            self.create_subscription(
                Image,
                topic,
                lambda msg, t=topic: self.image_callback(msg, t),
                10
            )

    def image_callback(self, msg, topic):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        with image_data_lock:
            _, buffer = cv2.imencode('.jpg', cv_image)
            image_data[topic] = buffer.tobytes()

async def handle_websocket(websocket, path):
    while True:
        try:
            message = await websocket.recv()
            if message.startswith("GET_IMAGE"):
                topic = message.split(" ")[1]
                with image_data_lock:
                    if topic in image_data:
                        await websocket.send(image_data[topic])
                    else:
                        await websocket.send(b"Topic not found")
            elif message == "GET_TOPICS":
                topics = list(image_data.keys())
                await websocket.send(str(topics))
        except websockets.ConnectionClosed:
            break

async def list_topics(request):
    topics = list(image_data.keys())
    html = '<html><head><title>Available ROS2 Image Topics</title></head><body>'
    html += '<h1>Available ROS2 Image Topics</h1>'
    html += '<ul>'
    for topic in topics:
        html += f'<li><a href="/image?topic={topic}">{topic}</a></li>'
    html += '</ul></body></html>'
    return web.Response(text=html, content_type='text/html')

async def serve_image(request):
    topic = request.query.get('topic')
    if topic in image_data:
        return web.Response(body=image_data[topic], content_type='image/jpeg')
    else:
        return web.Response(text='Image not found', status=404)

async def start_http_server():
    app = web.Application()
    app.router.add_get('/', list_topics)
    app.router.add_get('/image', serve_image)
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, 'localhost', 8081)
    await site.start()

async def main():
    rclpy.init()
    ros_image_server = ROS2ImageServer()
    
    # Start the WebSocket server
    ws_server = websockets.serve(handle_websocket, 'localhost', 8080)

    # Start the HTTP server
    await asyncio.gather(
        ws_server,
        start_http_server()
    )

if __name__ == '__main__':
    asyncio.run(main())
